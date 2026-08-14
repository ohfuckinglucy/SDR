#include "common.hpp"
#include "fec.hpp"
#include "functions.hpp"
#include "logger.hpp"
#include "modulation.hpp"
#include "ofdm_core.hpp"

#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

void DSPThread(SharedData &sd)
{
    std::chrono::high_resolution_clock::time_point t_start, t_end;

    std::vector<std::complex<float>> raw_buffer;
    std::vector<std::complex<float>> allbuff;
    std::vector<std::complex<float>> dsp_buffer;
    std::vector<int16_t> bits;
    std::vector<float> spectrum_local;

    static SignalModulation last_mod = (SignalModulation)-1;
    static std::vector<std::complex<float>> ref_constelation;

    sd.stats.EVM_vec.resize(sd.stats.vec_size, 0);
    sd.stats.SNR_vec.resize(sd.stats.vec_size, 0);

    sd.stats.vec_offset = sd.stats.vec_size - 1;

    ensure_fft_plans(sd);
    generate_zc_preamble(sd);

    while (sd.allRunning)
    {
        sd.pipe.read(raw_buffer);
        dsp_buffer.clear();

        OFDMcfg local_cfg;
        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            local_cfg = sd.ofdmcfg;
        }

        ensure_fft_plans(sd);

        size_t zc_size;
        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            zc_size = sd.ofdmcfg.zc_reference.size();
        }
        if (zc_size != static_cast<size_t>(local_cfg.N + local_cfg.CP))
            generate_zc_preamble(sd);

        t_start = std::chrono::high_resolution_clock::now();

        spectrum_local = Spectrum_calulations(std::ref(sd), raw_buffer);

        if (sd.hdr.modulation != last_mod)
        {
            ref_constelation = get_reference_constellation(sd.hdr.modulation);
            last_mod = sd.hdr.modulation;
        }

        if (sd.dspflags.PSS)
        {
            local_cfg.best_idx = zadoff_sync(raw_buffer, std::ref(sd));
            if (local_cfg.best_idx >= 0 &&
                local_cfg.best_idx + local_cfg.N + local_cfg.CP + local_cfg.mystery_offset < (int)raw_buffer.size())
            {
                int start = local_cfg.best_idx + local_cfg.N + local_cfg.CP + local_cfg.mystery_offset;
                if (start < 0)
                    start = 0;
                dsp_buffer.insert(dsp_buffer.end(), raw_buffer.begin() + start, raw_buffer.end());
            }
            else
                dsp_buffer = std::move(raw_buffer);
        }
        else
            dsp_buffer = std::move(raw_buffer);

        if (sd.dspflags.CFO)
            dsp_buffer = cfo_est(dsp_buffer, sd);

        if (sd.dspflags.FFT)
        {
            Header hdr = parse_header(dsp_buffer, sd);
            if (!hdr.is_valid)
                continue;

            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                sd.hdr = hdr;
            }

            int usable = usable_subcarriers(local_cfg.N, local_cfg.pilot_idx.size());
            size_t header_syms = (HEADER_BITS + usable - 1) / usable;

            allbuff.insert(allbuff.end(), dsp_buffer.begin(), dsp_buffer.end());

            uint16_t bps = bits_per_sym(hdr.modulation);
            if (bps > 0 && usable > 0)
            {
                size_t bits_per_ofdm = static_cast<size_t>(usable) * bps;
                size_t num_data_syms = (hdr.num_samples + bits_per_ofdm - 1) / bits_per_ofdm;
                size_t needed = (num_data_syms + header_syms) * (local_cfg.N + local_cfg.CP);
                if (allbuff.size() < needed)
                    continue;
            }
            else
            {
                allbuff.clear();
                continue;
            }

            dsp_buffer = std::move(allbuff);
            allbuff.clear();

            size_t header_len = header_syms * (local_cfg.N + local_cfg.CP);
            if (dsp_buffer.size() >= header_len)
                dsp_buffer.erase(dsp_buffer.begin(), dsp_buffer.begin() + header_len);

            dsp_buffer = FFT_ofdm(dsp_buffer, sd);
        }

        if (sd.dspflags.EQ)
        {
            dsp_buffer = ofdm_equalize(dsp_buffer, sd);

            uint16_t bps = bits_per_sym(sd.hdr.modulation);
            if (bps > 0)
            {
                size_t syms_needed = (sd.hdr.num_samples + bps - 1) / bps;
                if (dsp_buffer.size() > syms_needed)
                    dsp_buffer.resize(syms_needed);
            }

            bits = demodulator(dsp_buffer, sd.hdr.modulation);

            if (bits.size() > sd.hdr.num_samples)
                bits.resize(sd.hdr.num_samples);

            bool crc_ok = verifyCRC16(bits);

            sd.stats.total_block_proccesed++;
            if (!crc_ok)
                sd.stats.error_block++;
            sd.stats.BLER = sd.stats.total_block_proccesed > 0 ? static_cast<float>(sd.stats.error_block) / static_cast<float>(sd.stats.total_block_proccesed) : 0.0f;

            if (sd.hdr.sig_type == SignalType::Text && crc_ok)
            {
                std::string text;
                text.reserve(bits.size() / 8);
                for (size_t i = 0; i + 8 <= bits.size(); i += 8)
                {
                    char c = 0;
                    for (int b = 0; b < 8; ++b)
                        c |= (bits[i + b] << b);
                    text += c;
                }
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    sd.decoded_text = text;
                }
            }

            if (sd.hdr.sig_type == SignalType::File && crc_ok)
            {
                std::vector<uint8_t> bytes;
                bytes.reserve(bits.size() / 8);
                for (size_t i = 0; i + 8 <= bits.size(); i += 8)
                {
                    uint8_t c = 0;
                    for (int b = 0; b < 8; ++b)
                        c |= (bits[i + b] << b);
                    bytes.push_back(c);
                }

                std::lock_guard<std::mutex> lock(sd.mtx);
                if (sd.hdr.flag & FrameFlag::IsFirst)
                    sd.rx_file_chunks_buf.clear();
                sd.rx_file_chunks_buf.insert(
                    sd.rx_file_chunks_buf.end(),
                    bytes.begin(), bytes.end()
                );
                if (sd.hdr.flag & FrameFlag::IsLast && crc_ok)
                {
                    SaveReceivedFile(sd);
                    sd.file_received = true;
                }
            }
        }

        if (!sd.dspflags.EQ)
        {
            sd.stats.total_block_proccesed = 0;
            sd.stats.error_block = 0;
            sd.stats.BLER = 0;
        }

        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            sd.stats.EVM = EVM_calculate(dsp_buffer, ref_constelation);
            sd.stats.SNR = sd.stats.EVM > 1e-6f ? -20.0f * log10f(sd.stats.EVM / 100.0f) : 0.0f;

            sd.stats.vec_offset = (sd.stats.vec_offset - 1 + sd.stats.vec_size) % sd.stats.vec_size;
            sd.stats.EVM_vec[sd.stats.vec_offset] = sd.stats.EVM;
            sd.stats.SNR_vec[sd.stats.vec_offset] = sd.stats.SNR;

            sd.stats.frames_processed++;

            sd.gui_buffer = dsp_buffer;
            sd.spectrum = spectrum_local;
            sd.gui_timing_offsets = sd.timing_offsets;
        }

        t_end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        sd.avg_dsp_time = static_cast<float>(dur);
    }
}

void SDRStream(SharedData &sd, SDR &sdr)
{
    size_t blk = 0;
    bool transmission_active = false;

    while (sd.allRunning)
    {
        auto t_start = std::chrono::high_resolution_clock::now();

        sdr.updateConfig(sd);
        int sr = sdr.receive();
        if (sr < 0)
        {
            logs::sdr.critical("Failed to read stream");
            exit(1);
        }

        bool tx_once = false, tx_continuous = false, sig_changed = false;
        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            tx_once = sd.tx_once;
            tx_continuous = sd.tx_continuous;
            sig_changed = sd.sig_changed;
            sd.tx_once = false;
        }

        if (tx_once)
        {
            blk = 0;
            transmission_active = true;
            sig_changed = true;

            if (sd.type_of_signal == SignalType::File && sd.tx_file_loaded)
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                sd.tx_file_chunk_idx = 0;
            }
        }

        if (tx_continuous)
            transmission_active = true;

        if (transmission_active && blk == 0 && sig_changed)
        {
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                if (sd.type_of_signal == SignalType::File && sd.tx_file_loaded)
                {
                    sd.is_first = (sd.tx_file_chunk_idx == 0);
                    sd.is_last = (sd.tx_file_chunk_idx == sd.tx_file_total_chunks - 1);
                }
                else
                {
                    sd.is_first = true;
                    sd.is_last = !sd.tx_continuous;
                }
            }

            sd.tx_samples = GenerateSignal(sd);

            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                sd.sig_changed = false;
                if (sd.type_of_signal == SignalType::File && sd.tx_file_loaded)
                    sd.tx_file_chunk_idx++;
            }
        }

        if (transmission_active && !sd.tx_samples.empty())
        {
            size_t frame_len = sd.tx_samples.size() / 2;
            size_t num_blocks = (frame_len + sdr.tx_mtu - 1) / sdr.tx_mtu;

            if (blk < num_blocks)
            {
                size_t offset = 2 * blk * sdr.tx_mtu;
                size_t count = std::min((size_t)(2 * sdr.tx_mtu), sd.tx_samples.size() - offset);

                std::fill(sdr.tx_buffer.begin(), sdr.tx_buffer.end(), 0);
                std::copy(sd.tx_samples.begin() + offset, sd.tx_samples.begin() + offset + count, sdr.tx_buffer.begin());

                sdr.send();
                blk++;
            }

            if (blk >= num_blocks)
            {
                blk = 0;

                if (sd.type_of_signal == SignalType::File && sd.tx_file_loaded)
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    if (sd.tx_file_chunk_idx < sd.tx_file_total_chunks)
                        sd.sig_changed = true;
                    else
                    {
                        sd.tx_file_chunk_idx = 0;
                        transmission_active = false;
                        logs::sdr.info("File TX complete");
                    }
                }
                else if (!tx_continuous)
                    transmission_active = false;
                else
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    sd.sig_changed = true;
                }
            }
        }

        std::vector<std::complex<float>> tmp;
        tmp.reserve(sr);
        for (int i = 0; i < sr; ++i)
            tmp.emplace_back(static_cast<float>(sdr.rx_buffer[2 * i]), static_cast<float>(sdr.rx_buffer[2 * i + 1]));

        sd.pipe.write(tmp);

        auto t_end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        sd.avg_stream_time = static_cast<float>(dur);
    }
}
