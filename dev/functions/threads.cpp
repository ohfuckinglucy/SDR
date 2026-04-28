#include "common.h"
#include "error_interleaving.hpp"
#include "logger.hpp"
#include "modulator.h"
#include "ofdm_core.h"
#include "sdr_hw.h"
#include "sync_freq.h"
#include "sync_time.h"

void rx_back(SharedData &sd)
{
    std::vector<std::complex<float>> local_raw_buffer;
    std::vector<std::complex<float>> rx_buffer;
    std::vector<std::complex<float>> prev_buf;

    std::vector<float> local_fft_mag(sd.fft.FFT_SIZE);
    sd.ofdm_sync.reference = generate_zc_preamble(sd);

    std::chrono::high_resolution_clock::time_point t_start, t_end;

    long long total_duration_us = 0;
    int frame_count = 0;

    sd.SNR_vec.resize(sd.snr_vec_size, 0);
    sd.EVM_vec.resize(sd.snr_vec_size, 0);
    sd.snr_vec_offset = sd.snr_vec_size - 1;

    std::vector<int16_t> bits_bpsk = { 0, 1 };
    std::vector<std::complex<float>> constellation_bpsk = modulator(bits_bpsk, 2, "QAM::2");

    std::vector<int16_t> bits_qpsk = { 0, 0, 0, 1, 1, 0, 1, 1 };
    std::vector<std::complex<float>> constellation_qpsk = modulator(bits_qpsk, 8, "QAM::4");

    std::vector<int16_t> bits_16qam;
    for (int i = 0; i < 16; ++i)
    {
        bits_16qam.push_back((i >> 3) & 1);
        bits_16qam.push_back((i >> 2) & 1);
        bits_16qam.push_back((i >> 1) & 1);
        bits_16qam.push_back(i & 1);
    }
    std::vector<std::complex<float>> constellation_16qam = modulator(bits_16qam, 64, "QAM::16");

    std::vector<int16_t> bits_64qam;
    for (int i = 0; i < 64; ++i)
    {
        for (int b = 5; b >= 0; --b)
        {
            bits_64qam.push_back((i >> b) & 1);
        }
    }
    std::vector<std::complex<float>> constellation_64qam = modulator(bits_64qam, 384, "QAM::64");

    while (sd.flags.g_running)
    {
        t_start = std::chrono::high_resolution_clock::now();
        std::string mod_type;
        if (sd.flags.modulation_index == 0)
            mod_type = "QAM::2";
        else if (sd.flags.modulation_index == 1)
            mod_type = "QAM::4";
        else if (sd.flags.modulation_index == 2)
            mod_type = "QAM::16";
        else if (sd.flags.modulation_index == 3)
            mod_type = "QAM::64";
        else
            mod_type = "QAM::2";

        if (!sd.pipe.read(rx_buffer))
        {
            asm volatile("pause" ::: "memory");
            continue;
        }

        if (sd.flags.ofdm_time_est)
        {
            sd.ofdm.sig_begin = zadoff_sync(rx_buffer, sd) + sd.ofdm_sync.timing_offset;
            if (sd.ofdm.sig_begin > (rx_buffer.size() / 3) - (sd.ofdm.n_subcarriers + sd.ofdm.cp_len))
            {
                if (prev_buf.empty())
                {
                    prev_buf = std::move(rx_buffer);
                    continue;
                }
                else
                {
                    rx_buffer.insert(rx_buffer.begin(), prev_buf.begin(), prev_buf.end());
                    prev_buf.clear();
                }
            }
        }

        sd.buffer_without_dsp = rx_buffer;

        if (sd.ofdm.sig_begin >= 0 && sd.flags.cut_begin && sd.ofdm.sig_begin < 1920 - (sd.ofdm.n_subcarriers + sd.ofdm.cp_len))
        {
            local_raw_buffer = remove_pss(std::ref(sd), rx_buffer);
        }
        else
        {
            local_raw_buffer = std::move(rx_buffer);
        }

        if (sd.flags.cfo_est_enabled)
            local_raw_buffer = cfo_est(local_raw_buffer, sd);

        if (local_raw_buffer.empty() || local_raw_buffer.size() < 128)
            continue;

        sd.ofdm_sync.packet_len = 0;
        if (sd.flags.header_dec)
        {
            sd.ofdm_sync.packet_len = decode_header(local_raw_buffer, sd);
            if (sd.ofdm.n_subcarriers + sd.ofdm.cp_len < (int)local_raw_buffer.size())
                local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.n_subcarriers + sd.ofdm.cp_len);
        }

        if (sd.flags.ofdm_fft_enabled)
            local_raw_buffer = discard_cp(local_raw_buffer, sd);

        if (sd.flags.ofdm_eq_enabled)
            local_raw_buffer = ofdm_equalize(local_raw_buffer, sd);

        if (sd.flags.header_dec && sd.flags.ofdm_fft_enabled && sd.flags.ofdm_eq_enabled)
            if (sd.ofdm_sync.packet_len > 0 && sd.ofdm_sync.packet_len < (int)local_raw_buffer.size())
                local_raw_buffer.erase(local_raw_buffer.begin() + sd.ofdm_sync.packet_len, local_raw_buffer.end());
        if (mod_type == "QAM::2")
            sd.EVM = calculate_EVM(local_raw_buffer, constellation_bpsk);
        else if (mod_type == "QAM::4")
            sd.EVM = calculate_EVM(local_raw_buffer, constellation_qpsk);
        else if (mod_type == "QAM::16")
            sd.EVM = calculate_EVM(local_raw_buffer, constellation_16qam);
        else if (mod_type == "QAM::64")
            sd.EVM = calculate_EVM(local_raw_buffer, constellation_64qam);

        sd.SNR_DB = SNR_calculation(sd.buffer_without_dsp);

        sd.SNR_vec[sd.snr_vec_offset] = sd.SNR_DB;
        sd.EVM_vec[sd.snr_vec_offset] = sd.EVM;

        sd.snr_vec_offset = (sd.snr_vec_offset - 1 + sd.snr_vec_size) % sd.snr_vec_size;
        sd.frames_processed++;

        size_t n = std::min(sd.buffer_without_dsp.size(), sd.fft.FFT_SIZE);
        for (size_t i = 0; i < n; i++)
        {
            sd.fft.fft_in[i][0] = sd.buffer_without_dsp[sd.buffer_without_dsp.size() - n + i].real();
            sd.fft.fft_in[i][1] = sd.buffer_without_dsp[sd.buffer_without_dsp.size() - n + i].imag();
        }
        for (size_t i = n; i < sd.fft.FFT_SIZE; i++)
        {
            sd.fft.fft_in[i][0] = 0.0;
            sd.fft.fft_in[i][1] = 0.0;
        }
        fftw_execute(sd.fft.spectrum_plan);

        for (size_t i = 0; i < sd.fft.FFT_SIZE; i++)
        {
            float re = sd.fft.fft_out[i][0];
            float im = sd.fft.fft_out[i][1];
            local_fft_mag[i] = log10(re * re + im * im + 1e-10);
        }
        sd.flags.fft_ready = true;

        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            sd.buffer = std::move(local_raw_buffer);

            sd.interleaved_rx_bits = demodulator(sd.buffer, mod_type);

            if (!sd.interleaved_rx_bits.empty() && sd.flags.ofdm_eq_enabled)
            {
                sd.rx_bits = hamming_decoder(sd.interleaved_rx_bits, std::ref(sd));

                if (!sd.rx_bits.empty())
                {
                    
                }

                bool crc_ok = verifyCRC16(sd.rx_bits);

                if (crc_ok)
                {
                    pic_write(sd.rx_bits);
                }

                sd.bler_total_blocks++;
                if (!crc_ok)
                    sd.bler_error_blocks++;

                if (sd.bler_total_blocks > 0)
                    sd.bler_value = (float)sd.bler_error_blocks / sd.bler_total_blocks;

                if (sd.bler_total_blocks > 10000)
                {
                    sd.bler_total_blocks = 0;
                    sd.bler_error_blocks = 0;
                    sd.bler_value = 0;
                }
            }
            else
            {
                sd.bler_total_blocks = 0;
                sd.bler_error_blocks = 0;
                sd.bler_value = 0;
            }

            sd.fft.fft_magnitude = local_fft_mag;
        }

        t_end = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        total_duration_us += duration;
        frame_count++;

        sd.avg_time = (float)total_duration_us / frame_count;
        total_duration_us = 0;
        frame_count = 0;
    }
}

void SDRStream(SharedData &sd, SDRConfig &config)
{
    if (!config.sdr || !config.rxStream || !config.rx_buffer)
    {
        logs::sdr.error("ERROR: SDR config!");
        sd.flags.g_running = false;
        return;
    }

    size_t blk = 0;

    std::chrono::high_resolution_clock::time_point t_start, t_end;
    long long total_duration_us = 0;
    int frame_count = 0;

    while (sd.flags.g_running)
    {
        t_start = std::chrono::high_resolution_clock::now();

        reconfig_sdr(std::ref(sd), std::ref(config));
        signal_generate(std::ref(sd), std::ref(config));

        size_t frame_len = sd.tx_samples.size() / 2;
        size_t num_blocks = (frame_len + config.tx_mtu - 1) / config.tx_mtu;

        if (blk < num_blocks)
            blk = 0;

        void *rx_buffs[] = { config.rx_buffer };
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
        if (sd.flags.loopback_flag)
        {
            const void *tx_buffs[] = { sd.tx_samples.data() + 2 * blk * config.tx_mtu };
            int flags = SOAPY_SDR_HAS_TIME;
            long long tx_time = timeNs + TX_DELAY;

            SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, config.tx_mtu, &flags, tx_time, TIMEOUT);
            ++blk;
        }

        int16_t *data_ptr = static_cast<int16_t *>(config.rx_buffer);
        if (!data_ptr)
            continue;

        if (sr < 0)
        {
            logs::sdr.error("Failed to read stream!");
            continue;
        }

        std::vector<std::complex<float>> tmp;
        tmp.reserve(sr);
        for (int i = 0; i < sr; ++i)
            tmp.emplace_back((float)data_ptr[2 * i], (float)data_ptr[2 * i + 1]);
        sd.pipe.write(tmp);

        t_end = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        total_duration_us += duration;
        frame_count++;

        sd.avg_stream_time = (float)total_duration_us / frame_count;
        total_duration_us = 0;
        frame_count = 0;
    }
}