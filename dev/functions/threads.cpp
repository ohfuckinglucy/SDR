#include "header.h"
#include "modulator.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <atomic>

void rx_back(SharedData& sd, SDRConfig &config)
{
    if (!config.sdr || !config.rxStream) {
        cerr << "ERROR: SDR config!" << endl;
        sd.flags.g_running = false;
        return;
    }

    const size_t SCOPE_DISPLAY_SIZE = 1920 * N_BUFFERS;

    vector<complex<double>> scope_display_buffer(SCOPE_DISPLAY_SIZE);
    vector<complex<double>> local_raw_buffer;
    vector<complex<double>> local_symbols;
    size_t tx_sent_idx = 0;
    bool tx_active = false;
    vector<double> local_fft_mag(sd.fft.FFT_SIZE);

    vector<int16_t> tx_samples(2 * config.tx_mtu * N_BUFFERS, 0);

    vector<complex<double>> tx_frame;

    for (size_t samples_sent = 0; sd.flags.g_running; ++samples_sent){
        if (sd.flags.rx_gain_changed){
            SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, 0, sd.rx_gain);
            sd.flags.rx_gain_changed = false;
        }

        if (sd.flags.tx_gain_changed){
            SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, 0, sd.tx_gain);
            sd.flags.tx_gain_changed = false;
        }

        if (sd.flags.rx_freq_changed){
            SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, sd.freq, nullptr);
            sd.flags.rx_freq_changed = false;
        }

        if (sd.flags.rx_bw_changed){
            SoapySDRDevice_setBandwidth(config.sdr, SOAPY_SDR_RX, 0, sd.rx_bandwidth);
            sd.flags.rx_bw_changed = false;
        }

        string mod_type;
        if (sd.flags.modulation_index == 0) mod_type = "QAM::2";
        else if (sd.flags.modulation_index == 1) mod_type = "QAM::4";
        else mod_type = "QAM::16";

        if (sd.flags.tx_regenerate){
            vector<complex<double>> baseband_frame; 
            int bits_ps = bits_per_symbol(mod_type);

            size_t total_symbols = sd.tx_symbol_count;

            if (sd.flags.ofdm_enabled_tx) {
                int data_per_symbol =
                    sd.ofdm.n_subcarriers - sd.ofdm.pilot_idx.size();

                int ofdm_blocks =
                    ceil((double)total_symbols / data_per_symbol);

                total_symbols = ofdm_blocks * data_per_symbol;
            }

            sd.bits.resize(total_symbols * bits_ps);
            for (auto &b : sd.bits)
                b = rand() % 2;

            vector<complex<double>> symbols = modulator(sd.bits, sd.bits.size(), mod_type);

            tx_frame.clear();

            if (sd.flags.ofdm_enabled_tx){
                vector<complex<double>> freq_blocks = insert_pilots(symbols, sd);
                vector<complex<double>> data_signal = ofdm_modulator(freq_blocks, sd);
                vector<complex<double>> preamble = preamble_generate(sd);

                baseband_frame.reserve(preamble.size() + data_signal.size());
                baseband_frame.insert(baseband_frame.end(), preamble.begin(), preamble.end());
                baseband_frame.insert(baseband_frame.end(), data_signal.begin(), data_signal.end());
            } else {
                baseband_frame = move(symbols);
            }

            tx_frame.clear();
            if (!baseband_frame.empty()) {
                if (sd.flags.upsampling_enabled) {
                    tx_frame = UpSampler(baseband_frame, sd.FormFilter.tx_l);
                    if (sd.flags.tx_filter) filter(tx_frame.data(), tx_frame.size(), sd.FormFilter.tx_l);
                } else {
                    tx_frame = move(baseband_frame);
                }
            }

            tx_sent_idx = 0;
            tx_active = !tx_frame.empty();
            sd.flags.tx_regenerate = false;
        }

        if (sd.flags.loopback_flag && !tx_frame.empty()) {
            fill(tx_samples.begin(), tx_samples.end(), 0);

            size_t max_samples = min(tx_frame.size(), (size_t)(N_BUFFERS * config.tx_mtu));

            for (size_t i = 0; i < max_samples; ++i) {
                double scale = 12000.0;

                tx_samples[2*i] = static_cast<int16_t>(tx_frame[i].real() * scale);
                tx_samples[2*i+1] = static_cast<int16_t>(tx_frame[i].imag() * scale);
            }
        }

        void *rx_buffs[] = {config.rx_buffer};
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(
            config.sdr,
            config.rxStream,
            rx_buffs,
            config.rx_mtu,
            &flags,
            &timeNs,
            TIMEOUT);

        if (sd.flags.loopback_flag && !tx_samples.empty()) {
            long long tx_time = timeNs + TX_DELAY;
            flags = SOAPY_SDR_HAS_TIME;

            size_t tx_idx = samples_sent % N_BUFFERS;

            const void* tx_buffs[] = {
                tx_samples.data() +
                (tx_idx * config.tx_mtu)
            };

            SoapySDRDevice_writeStream(
                config.sdr,
                config.txStream,
                tx_buffs,
                config.tx_mtu,
                &flags,
                tx_time,
                TIMEOUT);
        }

        int16_t* data_ptr = static_cast<int16_t*>(rx_buffs[0]);

        local_raw_buffer.clear();
        local_raw_buffer.reserve(sr);
        local_symbols.clear();

        for (int i = 0; i < sr; ++i) {
            double I = static_cast<double>(data_ptr[2*i]);
            double Q = static_cast<double>(data_ptr[2*i + 1]);

            complex<double> x(I, Q);

            if (sd.flags.costas_loop_enabled){
                if (sd.flags.QAM16_costas_loop) x = costas_loop_16qam(sd, x);
                else x = costas_loop(sd, x);
            }

            local_raw_buffer.push_back(x);
        }

        update_scope_buffer(
            scope_display_buffer,
            local_raw_buffer,
            SCOPE_DISPLAY_SIZE);

        if (sd.flags.ofdm_time_est) {
            sd.ofdm.sig_begin = time_est(local_raw_buffer, sd);
        }

        if (sd.flags.cfo_est_enabled) local_raw_buffer = cfo_est(local_raw_buffer, sd);

        if (sd.ofdm.sig_begin >= 0 && sd.flags.cut_begin){
            local_raw_buffer.erase(
            local_raw_buffer.begin(),
            local_raw_buffer.begin() + sd.ofdm.sig_begin + 80);
        }

        if (sd.flags.ofdm_eq_enabled){
            local_raw_buffer = discard_cp(local_raw_buffer, sd);
            local_raw_buffer = ofdm_equalize(local_raw_buffer, sd);
        }

        if (sd.flags.filter_enabled) filter(local_raw_buffer.data(), local_raw_buffer.size(), sd.FormFilter.rx_l);

        if (sd.gardner.sym_sync_enabled) {
                sym_sync(sd, local_raw_buffer);
                sd.flags.used_gardner = true;
                for (size_t i = sd.gardner.ss_offset; i < local_raw_buffer.size(); i += sd.FormFilter.rx_l) {
                    local_symbols.push_back(local_raw_buffer[i]);
                }
            } else {
                local_symbols = (local_raw_buffer);
                sd.flags.used_gardner = false;
            }

        if (sd.flags.fft_flag) {
                size_t n = min(local_raw_buffer.size(), sd.fft.FFT_SIZE);
                for (size_t i = 0; i < n; i++) {
                    sd.fft.fft_in[i][0] = local_raw_buffer[local_raw_buffer.size() - n + i].real();
                    sd.fft.fft_in[i][1] = local_raw_buffer[local_raw_buffer.size() - n + i].imag();
                }
                for (size_t i = n; i < sd.fft.FFT_SIZE; i++) {
                    sd.fft.fft_in[i][0] = 0.0;
                    sd.fft.fft_in[i][1] = 0.0;
                }
                
                fftw_execute(sd.fft.fft_plan);

                for (size_t i = 0; i < sd.fft.FFT_SIZE; i++) {
                    double re = sd.fft.fft_out[i][0];
                    double im = sd.fft.fft_out[i][1];
                    local_fft_mag[i] = log10(re * re + im * im + 1e-10);
                }
                sd.flags.fft_ready = true;
            }

        {
            lock_guard<mutex> lock(sd.mtx);

            sd.raw_buffer = local_raw_buffer;
            sd.symbols = local_symbols;
            sd.scope_buffer = scope_display_buffer;

            if (sd.flags.fft_flag)
                sd.fft.fft_magnitude = local_fft_mag;
        }
    }
}

void tx_back(SharedData& sd, SDRConfig &config) {
    string last_mod_type = "";
    int bits_size = 0;

    size_t tx_sent_idx = 0;
    bool tx_active = false;

    vector<complex<double>> tx_frame;
    vector<int16_t> tx_samples(2 * config.tx_mtu * N_BUFFERS, 0);
    
    while (sd.flags.g_running) {
        static bool bits_initialized = false;
        if (sd.flags.tx_gain_changed)
        {
            SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, 0, sd.tx_gain);
            sd.flags.tx_gain_changed = false;
        }

        if (sd.flags.tx_freq_changed)
        {
            SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, sd.freq, nullptr);
            sd.flags.tx_freq_changed = false;
        }
        
        string mod_type;
        if (sd.flags.modulation_index == 0) mod_type = "QAM::2";
        else if (sd.flags.modulation_index == 2) mod_type = "QAM::16";
        else mod_type = "QAM::4";
        
        if (sd.flags.tx_regenerate){
            vector<complex<double>> baseband_frame; 
            int bits_ps = bits_per_symbol(mod_type);

            size_t total_symbols = sd.tx_symbol_count;

            if (sd.flags.ofdm_enabled_tx) {
                int data_per_symbol =
                    sd.ofdm.n_subcarriers - sd.ofdm.pilot_idx.size();

                int ofdm_blocks =
                    ceil((double)total_symbols / data_per_symbol);

                total_symbols = ofdm_blocks * data_per_symbol;
            }

            sd.bits.resize(total_symbols * bits_ps);
            for (auto &b : sd.bits)
                b = rand() % 2;

            vector<complex<double>> symbols = modulator(sd.bits, sd.bits.size(), mod_type);

            tx_frame.clear();

            if (sd.flags.ofdm_enabled_tx){
                vector<complex<double>> freq_blocks = insert_pilots(symbols, sd);
                vector<complex<double>> data_signal = ofdm_modulator(freq_blocks, sd);
                vector<complex<double>> preamble = preamble_generate(sd);

                baseband_frame.reserve(preamble.size() + data_signal.size());
                baseband_frame.insert(baseband_frame.end(), preamble.begin(), preamble.end());
                baseband_frame.insert(baseband_frame.end(), data_signal.begin(), data_signal.end());
            } else {
                baseband_frame = move(symbols);
            }

            tx_frame.clear();
            if (!baseband_frame.empty()) {
                if (sd.flags.upsampling_enabled) {
                    tx_frame = UpSampler(baseband_frame, sd.FormFilter.tx_l);
                    if (sd.flags.tx_filter) filter(tx_frame.data(), tx_frame.size(), sd.FormFilter.tx_l);
                } else {
                    tx_frame = move(baseband_frame);
                }
            }

            tx_sent_idx = 0;
            tx_active = !tx_frame.empty();
            sd.flags.tx_regenerate = false;
        }

        fill(tx_samples.begin(), tx_samples.end(), 0);

            size_t max_samples = min(tx_frame.size(), (size_t)(N_BUFFERS * config.tx_mtu));

            for (size_t i = 0; i < max_samples; ++i) {
                double scale = 12000.0;

                tx_samples[2*i] = static_cast<int16_t>(tx_frame[i].real() * scale);
                tx_samples[2*i+1] = static_cast<int16_t>(tx_frame[i].imag() * scale);
            }

        {
            sd.last_tx_samples.clear();
            int N_show = min(100, static_cast<int>(tx_samples.size()));
            sd.last_tx_samples.assign(tx_samples.begin(), tx_samples.begin() + N_show);
        }

        size_t total = tx_samples.size() / 2;
        size_t sent = 0;

        while (sent < total) {
            size_t to_send = min(static_cast<size_t>(config.tx_mtu), total - sent);
            const void* tx_buffs[] = { tx_samples.data()};
            void* rx_buffs[] = { config.rx_buffer };

            int flags = 0;
            long long timeNs = 0;

            int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
  

            long long tx_time = timeNs + TX_DELAY;
            flags = SOAPY_SDR_HAS_TIME;

            SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, config.rx_mtu, &flags, tx_time, TIMEOUT);

            sent += to_send;
        }
    }
}