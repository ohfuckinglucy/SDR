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

void rx_back(SharedData& sd, SDRConfig &config) {
    if (!config.sdr || !config.rxStream) {
        cerr << "ERROR: SDR config!" << endl;
        sd.flags.g_running = false;
        return;
    }

    vector<complex<double>> symbols;
    vector<complex<double>> symbols_UL;
    vector<int16_t> tx_samples(2 * config.tx_mtu, 0);

    // bits.resize(bit_size);
    
    int cnt = 0;
    for (size_t samples_sent = 0; sd.flags.g_running; ++samples_sent) {
        if (sd.flags.loopback_flag){
            string mod_type;
            
            if (sd.flags.modulation_index == 0){
                mod_type = "QAM::2";
            } else if (sd.flags.modulation_index == 2){
                mod_type = "QAM::16";
            } else {
                mod_type = "QAM::4";
            }
            
            int bits_size = bits_per_symbol(mod_type);
            size_t max_symbols = config.tx_mtu / sd.FormFilter.tx_l;

            vector<int16_t> bits(max_symbols * bits_size);
            const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
            for (int i = 0; i < 26; ++i) {
                bits[i] = barker13[i % 13];
            }
            for (int i = 26; i < bits.size(); ++i) {
                bits[i] = rand() % 2;
            }

            vector<complex<double>> signal;

            vector<complex<double>> symbols = modulator(bits, bits.size(), mod_type);

            if (sd.flags.ofdm_enabled){
                signal = ofdm_modulator(symbols, ref(sd));
            } else {
                signal = move(symbols);
            }

            vector<complex<double>> final_signal;

            if (sd.flags.upsampling_enabled){
                final_signal = UpSampler(signal, sd.FormFilter.tx_l);
                if (sd.flags.tx_filter) {
                    filter(final_signal.data(), final_signal.size(), sd.FormFilter.tx_l);
                }
            } else {
                final_signal = move(signal);
            }
            
            tx_samples.resize(2 * config.tx_mtu);
            for (size_t i = 0; i < config.tx_mtu; i++) {
                tx_samples[2*i]   = static_cast<int16_t>(real(final_signal[i]) * 16000);
                tx_samples[2*i+1] = static_cast<int16_t>(imag(final_signal[i]) * 16000);
            }
        }
        
        void *rx_buffs[] = {config.rx_buffer};
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        if (sd.flags.loopback_flag && !tx_samples.empty()) {
            const void* tx_buffs[] = {tx_samples.data()};
            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, config.tx_mtu, &flags, tx_time, TIMEOUT);
        }

        
        int16_t* data_ptr = static_cast<int16_t*>(rx_buffs[0]);
        
        {
            sd.raw_buffer.clear();

            for (int i = 0; i < sr; ++i) {
                double I = static_cast<double>(data_ptr[2*i]);
                double Q = static_cast<double>(data_ptr[2*i + 1]);

                complex<double> x(I, Q);

                if (sd.flags.filter_enabled){
                    x = mf_filter(sd, x);
                }
                if (sd.flags.costas_loop_enabled){
                    x = costas_loop(sd, x);
                }

                sd.raw_buffer.push_back(x);
            }

            sd.symbols.clear();
            if (sd.gardner.sym_sync_enabled) {
                sym_sync(sd, sd.raw_buffer);
            }
            for (size_t i = sd.gardner.ss_offset; i < sd.raw_buffer.size(); i += sd.FormFilter.rx_l) {
                sd.symbols.push_back(sd.raw_buffer[i]);
            }

            if(sd.flags.fft_flag){
                size_t n = min(sd.raw_buffer.size(), sd.fft.FFT_SIZE);
                for (size_t i = 0; i < n; i++) {
                    sd.fft.fft_in[i][0] = sd.raw_buffer[sd.raw_buffer.size() - n + i].real(); // I
                    sd.fft.fft_in[i][1] = sd.raw_buffer[sd.raw_buffer.size() - n + i].imag(); // Q
                }

                for (size_t i = n; i < sd.fft.FFT_SIZE; i++) {
                    sd.fft.fft_in[i][0] = 0.0;
                    sd.fft.fft_in[i][1] = 0.0;
                }

                fftw_execute(sd.fft.fft_plan);

                for (size_t i = 0; i < sd.fft.FFT_SIZE; i++) {
                    double re = sd.fft.fft_out[i][0];
                    double im = sd.fft.fft_out[i][1];
                    sd.fft.fft_magnitude[i] = log10(re * re + im * im + 1e-10);
                }

                sd.flags.fft_ready = true;
            }
        }
    }
}

void tx_back(SharedData& sd, SDRConfig &config) {
    while (sd.flags.g_running) {
        string mod_type;
        
        if (sd.flags.modulation_index == 0){
            mod_type = "QAM::2";
        } else if (sd.flags.modulation_index == 2){
            mod_type = "QAM::16";
        } else {
            mod_type = "QAM::4";
        }
        
        int bits_size = bits_per_symbol(mod_type);
        size_t max_symbols = config.tx_mtu / sd.FormFilter.tx_l;

        vector<int16_t> bits(max_symbols * bits_size);
        const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
        for (int i = 0; i < 26; ++i) {
            bits[i] = barker13[i % 13];
        }
        for (int i = 26; i < bits.size(); ++i) {
            bits[i] = rand() % 2;
        }

        vector<complex<double>> signal;

        vector<complex<double>> symbols = modulator(bits, bits.size(), mod_type);

        if (sd.flags.ofdm_enabled){
            signal = ofdm_modulator(symbols, ref(sd));
        } else {
            signal = move(symbols);
        }

        vector<complex<double>> final_signal;

        if (sd.flags.upsampling_enabled){
            final_signal = UpSampler(signal, sd.FormFilter.tx_l);
            if (sd.flags.tx_filter) {
                filter(final_signal.data(), final_signal.size(), sd.FormFilter.tx_l);
            }
        } else {
            final_signal = move(signal);
        }

        vector<int16_t> tx_samples(2 * final_signal.size());
        for (size_t i = 0; i < final_signal.size(); i++) {
            tx_samples[2*i]   = static_cast<int16_t>(real(final_signal[i]) * 16000);
            tx_samples[2*i+1] = static_cast<int16_t>(imag(final_signal[i]) * 16000);
        }

        {
            lock_guard<mutex> lock(sd.mtx);
            sd.last_tx_samples.clear();
            int N_show = min(100, static_cast<int>(tx_samples.size()));
            sd.last_tx_samples.assign(tx_samples.begin(), tx_samples.begin() + N_show);
        }

        size_t total = tx_samples.size() / 2;
        size_t sent = 0;

        while (sent < total) {
            size_t to_send = min(static_cast<size_t>(config.tx_mtu), total - sent);
            const void* tx_buffs[] = { tx_samples.data() + sent * 2 };
            void* rx_buffs[] = { config.rx_buffer };

            int flags = 0;
            long long timeNs = 0;

            int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
            if (sr <= 0) {
                timeNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()
                ).count();
            }

            long long tx_time = timeNs + TX_DELAY;
            flags = SOAPY_SDR_HAS_TIME;

            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, config.rx_mtu, &flags, tx_time, TIMEOUT);

            sent += to_send;
        }
    }
}