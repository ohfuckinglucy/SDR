#include "common.h"
#include "sdr_hw.h"
#include "modulator.h"
#include "ofdm_core.h"
#include "sync_time.h"
#include "sync_freq.h"
#include <iostream>

void rx_back(SharedData &sd, SDRConfig &config)
{
    vector<complex<double>> local_symbols;
    vector<double> local_fft_mag(sd.fft.FFT_SIZE);
    sd.ofdm_sync.reference = generate_zc_preamble(sd);

    chrono::high_resolution_clock::time_point t_start, t_end;

    long long total_duration_us = 0;
    int frame_count = 0;

    while (sd.flags.g_running)
    {
        t_start = chrono::high_resolution_clock::now();
        string mod_type;
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

        size_t filled = sd.pipe.filled_count.load(memory_order_acquire);

        if (filled == 0)
        {
            asm volatile("pause" ::: "memory");
            continue;
        }

        size_t current_read = sd.pipe.read_idx.load(memory_order_relaxed);

        lock_guard<mutex> lock(sd.pipe.buf_mutex[current_read]);

        vector<complex<double>> local_raw_buffer = move(sd.pipe.buffers[current_read]);
        sd.pipe.buffers[current_read].clear();

        sd.raw_buffer_without_dsp = local_raw_buffer;

        size_t next_read = (current_read + 1) % Dbuf::NUM_BUFFERS;
        sd.pipe.read_idx.store(next_read, memory_order_release);
        sd.pipe.filled_count.fetch_sub(1, memory_order_acq_rel);

        if (local_raw_buffer.empty())
            continue;

        local_symbols.clear();

        if (sd.flags.ofdm_time_est)
        {
            sd.ofdm.sig_begin = zc_sync(local_raw_buffer, sd);
            if (sd.flags.loopback_flag)
                sd.flags.ofdm_time_est = false;
        }

        if (sd.ofdm.sig_begin >= 0 && sd.flags.cut_begin && sd.ofdm.sig_begin < (int)local_raw_buffer.size())
        {
            local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.sig_begin + sd.ofdm.n_subcarriers + sd.ofdm.cp_len);
        }

        sd.ofdm_sync.packet_len = 0;
        if (sd.flags.header_dec)
        {
            sd.ofdm_sync.packet_len = decode_header(local_raw_buffer, sd);
            if (sd.ofdm.n_subcarriers + sd.ofdm.cp_len < (int)local_raw_buffer.size())
            {
                local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.n_subcarriers + sd.ofdm.cp_len);
            }
        }

        if (sd.flags.cfo_est_enabled && sd.ofdm.sym_begin >= 0)
            local_raw_buffer = cfo_est(local_raw_buffer, sd);

        if (sd.flags.ofdm_fft_enabled)
            local_raw_buffer = discard_cp(local_raw_buffer, sd);

        if (sd.flags.ofdm_eq_enabled)
            local_raw_buffer = ofdm_equalize(local_raw_buffer, sd);

        if (sd.flags.header_dec && sd.flags.ofdm_fft_enabled && sd.flags.ofdm_eq_enabled){
            if (sd.ofdm_sync.packet_len > 0 && sd.ofdm_sync.packet_len < (int)local_raw_buffer.size())
            {
                local_raw_buffer.erase(local_raw_buffer.begin() + sd.ofdm_sync.packet_len, local_raw_buffer.end());
            }
        }

        local_symbols = local_raw_buffer;

        size_t n = min(local_raw_buffer.size(), sd.fft.FFT_SIZE);
        for (size_t i = 0; i < n; i++)
        {
            sd.fft.fft_in[i][0] = local_raw_buffer[local_raw_buffer.size() - n + i].real();
            sd.fft.fft_in[i][1] = local_raw_buffer[local_raw_buffer.size() - n + i].imag();
        }
        for (size_t i = n; i < sd.fft.FFT_SIZE; i++)
        {
            sd.fft.fft_in[i][0] = 0.0;
            sd.fft.fft_in[i][1] = 0.0;
        }
        fftw_execute(sd.fft.spectrum_plan);

        for (size_t i = 0; i < sd.fft.FFT_SIZE; i++)
        {
            double re = sd.fft.fft_out[i][0];
            double im = sd.fft.fft_out[i][1];
            local_fft_mag[i] = log10(re * re + im * im + 1e-10);
        }
        sd.flags.fft_ready = true;

        {
            lock_guard<mutex> lock(sd.mtx);
            sd.raw_buffer = move(local_raw_buffer);
            sd.rx_bits = demodulator(sd.raw_buffer, mod_type);

            if (!sd.rx_bits.empty() && sd.flags.ofdm_eq_enabled){
                // sd.rx_bits = hamming_decoder_from_Bits(sd.rx_bits);

                bool crc_ok = verifyCRC16(sd.rx_bits);

                sd.bler_total_blocks++;
                if (!crc_ok) {
                    sd.bler_error_blocks++;
                }

                if (sd.bler_total_blocks > 0) {
                    sd.bler_value = (double)sd.bler_error_blocks / sd.bler_total_blocks;
                }

                if (sd.bler_total_blocks > 1000){
                    sd.bler_total_blocks = 0;
                    sd.bler_error_blocks = 0;
                    sd.bler_value = 0;
                }
            } else {
                sd.bler_total_blocks = 0;
                sd.bler_error_blocks = 0;
                sd.bler_value = 0;
            }

            sd.symbols = move(local_symbols);
            sd.fft.fft_magnitude = local_fft_mag;
        }

        t_end = chrono::high_resolution_clock::now();

        auto duration = chrono::duration_cast<chrono::microseconds>(t_end - t_start).count();
        total_duration_us += duration;
        frame_count++;

        sd.avg_time = (double)total_duration_us / frame_count;
        total_duration_us = 0;
        frame_count = 0;
    }
}

void SDRStream(SharedData &sd, SDRConfig &config)
{
    if (!config.sdr || !config.rxStream || !config.rx_buffer)
    {
        cerr << "ERROR: SDR config!" << endl;
        sd.flags.g_running = false;
        return;
    }

    static bool buffers_initialized = false;
    if (!buffers_initialized)
    {
        for (size_t i = 0; i < Dbuf::NUM_BUFFERS; ++i)
        {
            sd.pipe.buffers[i].reserve(sd.pipe.buffer_size);
            sd.pipe.buffers[i].clear();
        }
        buffers_initialized = true;
    }

    size_t blk = 0;

    chrono::high_resolution_clock::time_point t_start, t_end;
    long long total_duration_us = 0;
    int frame_count = 0;

    while (sd.flags.g_running)
    {
        t_start = chrono::high_resolution_clock::now();

        reconfig_sdr(ref(sd), ref(config));
        signal_generate(ref(sd), ref(config));

        size_t frame_len = sd.tx_samples.size() / 2;
        size_t num_blocks = (frame_len + config.tx_mtu - 1) / config.tx_mtu;
        size_t total_samples = num_blocks * config.tx_mtu;

        if (blk < num_blocks)
            blk = 0;

        void *rx_buffs[] = {config.rx_buffer};
        int flags = 0;
        long long timeNs = 0;

        size_t buf_count = total_samples / config.tx_mtu;

        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
        if (sd.flags.loopback_flag)
        {
            const void *tx_buffs[] = {sd.tx_samples.data() + 2 * blk * config.tx_mtu};
            int flags = SOAPY_SDR_HAS_TIME;
            long long tx_time = timeNs + TX_DELAY;

            SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, config.tx_mtu, &flags, tx_time, TIMEOUT);
            ++blk;
        }

        size_t current_write = sd.pipe.write_idx.load(memory_order_acquire);

        if (current_write >= Dbuf::NUM_BUFFERS)
        {
            cerr << "[SDRStream] CORRUPTED: write_idx=" << current_write << ", resetting!" << endl;
            sd.pipe.write_idx.store(0, memory_order_release);
            sd.pipe.filled_count.store(0, memory_order_release);
            continue;
        }

        size_t current_filled = sd.pipe.filled_count.load(memory_order_acquire);

        if (current_filled >= Dbuf::NUM_BUFFERS)
        {
            size_t old_read = sd.pipe.read_idx.load(memory_order_acquire);
            size_t new_read = (old_read + 1) % Dbuf::NUM_BUFFERS;
            sd.pipe.read_idx.store(new_read, memory_order_release);
            sd.pipe.filled_count.fetch_sub(1, memory_order_acq_rel);
            sd.pipe.overwritten.fetch_add(1, memory_order_relaxed);
        }

        auto &buf = sd.pipe.buffers[current_write];
        lock_guard<mutex> lock(sd.pipe.buf_mutex[current_write]);

        buf.clear();

        int16_t *data_ptr = static_cast<int16_t *>(config.rx_buffer);
        if (!data_ptr)
            continue;

        for (int i = 0; i < sr; ++i)
        {
            if (2 * i + 1 >= config.rx_mtu * 2)
                break;
            buf.emplace_back(
                static_cast<double>(data_ptr[2 * i]),
                static_cast<double>(data_ptr[2 * i + 1]));
        }

        size_t next_write = (current_write + 1) % Dbuf::NUM_BUFFERS;
        sd.pipe.write_idx.store(next_write, memory_order_release);
        sd.pipe.filled_count.fetch_add(1, memory_order_acq_rel);

        t_end = chrono::high_resolution_clock::now();

        auto duration = chrono::duration_cast<chrono::microseconds>(t_end - t_start).count();
        total_duration_us += duration;
        frame_count++;

        sd.avg_stream_time = (double)total_duration_us / frame_count;
        total_duration_us = 0;
        frame_count = 0;
    }
}