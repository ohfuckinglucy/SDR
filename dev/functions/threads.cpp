#include "common.h"
#include "sdr_hw.h"
#include "modulator.h"
#include "ofdm_core.h"
#include "sync_time.h"
#include "sync_freq.h"
#include <iostream>

void rx_back(SharedData &sd, SDRConfig &config)
{
    vector<complex<double>> local_raw_buffer;

    vector<double> local_fft_mag(sd.fft.FFT_SIZE);
    sd.ofdm_sync.reference = generate_zc_preamble(sd);

    chrono::high_resolution_clock::time_point t_start, t_end;

    long long total_duration_us = 0;
    int frame_count = 0;

    while (sd.flags.g_running)
    {
        t_start = chrono::high_resolution_clock::now();
        string mod_type;
        if (sd.flags.modulation_index == 0) mod_type = "QAM::2";
        else if (sd.flags.modulation_index == 1) mod_type = "QAM::4";
        else if (sd.flags.modulation_index == 2) mod_type = "QAM::16";
        else if (sd.flags.modulation_index == 3) mod_type = "QAM::64";
        else mod_type = "QAM::2";

        if (!sd.pipe.read(local_raw_buffer)) {
            asm volatile("pause" ::: "memory");
            continue;
        }
        
        sd.buffer_without_dsp = local_raw_buffer;

        if (sd.flags.ofdm_time_est)
        {
            sd.ofdm.sig_begin = zc_sync(local_raw_buffer, sd);
        }

        if (sd.ofdm.sig_begin >= 0 && sd.flags.cut_begin && sd.ofdm.sig_begin < (int)local_raw_buffer.size() - (sd.ofdm.n_subcarriers + sd.ofdm.cp_len))
        {
            local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.sig_begin + sd.ofdm.n_subcarriers + sd.ofdm.cp_len);
        }

        sd.ofdm_sync.packet_len = 0;
        if (sd.flags.header_dec)
        {
            sd.ofdm_sync.packet_len = decode_header(local_raw_buffer, sd);
            if (sd.ofdm.n_subcarriers + sd.ofdm.cp_len < (int)local_raw_buffer.size())
                local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.n_subcarriers + sd.ofdm.cp_len);
        }

        if (sd.flags.cfo_est_enabled)
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
            sd.buffer = move(local_raw_buffer);
            sd.interleaved_rx_bits = demodulator(sd.buffer, mod_type);

            if (!sd.interleaved_rx_bits.empty() && sd.flags.ofdm_eq_enabled){
                sd.rx_bits = hamming_decoder(sd.interleaved_rx_bits, ref(sd));

                bool crc_ok = verifyCRC16(sd.rx_bits);

                sd.bler_total_blocks++;
                if (!crc_ok) {
                    sd.bler_error_blocks++;
                }

                if (sd.bler_total_blocks > 0) {
                    sd.bler_value = (double)sd.bler_error_blocks / sd.bler_total_blocks;
                }

                if (sd.bler_total_blocks > 10000){
                    sd.bler_total_blocks = 0;
                    sd.bler_error_blocks = 0;
                    sd.bler_value = 0;
                }
            } else {
                sd.bler_total_blocks = 0;
                sd.bler_error_blocks = 0;
                sd.bler_value = 0;
            }

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

        int16_t* data_ptr = static_cast<int16_t*>(config.rx_buffer);
        if (!data_ptr) continue;


        vector<complex<double>> tmp;
        tmp.reserve(sr);
        for (int i = 0; i < sr; ++i)
            tmp.emplace_back((double)data_ptr[2*i], (double)data_ptr[2*i+1]);
        sd.pipe.write(tmp);

        t_end = chrono::high_resolution_clock::now();

        auto duration = chrono::duration_cast<chrono::microseconds>(t_end - t_start).count();
        total_duration_us += duration;
        frame_count++;

        sd.avg_stream_time = (double)total_duration_us / frame_count;
        total_duration_us = 0;
        frame_count = 0;
    }
}