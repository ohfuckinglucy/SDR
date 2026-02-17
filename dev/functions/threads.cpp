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
    int bit_size = 1920*2;

    if (!config.sdr || !config.rxStream) {
        cerr << "ERROR: SDR config!" << endl;
        sd.flags.g_running = false;
        return;
    }

    vector<complex<double>> symbols;
    vector<complex<double>> symbols_UL;
    vector<int16_t> tx_samples(2 * config.tx_mtu, 0);

    sd.bits.resize(bit_size);
    const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
    for (int i = 0; i < 26; ++i) {
        sd.bits[i] = barker13[i % 13];
    }
    for (int i = 26; i < bit_size; ++i) {
        sd.bits[i] = rand() % 2;
    }

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

            vector<complex<double>> symbols = modulator(sd.bits.data(), bit_size, mod_type);

            vector<complex<double>> symbols_UL = UpSampler(symbols.data(), symbols.size(), sd.FormFilter.tx_l);

            if (sd.flags.tx_filter) {
                filter(symbols_UL.data(), symbols_UL.size(), sd.FormFilter.tx_l);
            }

            
            tx_samples.resize(2 * symbols_UL.size());
            for (size_t i = 0; i < symbols_UL.size(); i++) {
                tx_samples[2*i]   = static_cast<int16_t>(real(symbols_UL[i]) * 16000);
                tx_samples[2*i+1] = static_cast<int16_t>(imag(symbols_UL[i]) * 16000);
            }
        }
        
        void *rx_buffs[] = {config.rx_buffer};
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        if (sd.flags.loopback_flag && !tx_samples.empty()) {
            size_t total = tx_samples.size() / 2;
            if (samples_sent >= total) {
                samples_sent = 0;
            }
            size_t to_send = min(static_cast<size_t>(config.tx_mtu), total - samples_sent);
            const void* tx_buffs[] = { tx_samples.data() + samples_sent * 2 };
            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, to_send, &flags, tx_time, TIMEOUT);
            samples_sent += to_send;
        }

        int16_t* data_ptr = static_cast<int16_t*>(config.rx_buffer);

        {
            lock_guard<mutex> lock(sd.mtx);

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

            if (sd.raw_buffer.size() > SharedData::MAX_SAMPLES) {
                sd.raw_buffer.erase(sd.raw_buffer.begin(), sd.raw_buffer.end() - SharedData::MAX_SAMPLES);
            }
            if (sd.gardner.sym_sync_enabled) {
                sym_sync(sd, sd.raw_buffer);
                sd.symbols.clear();
                for (size_t i = sd.gardner.ss_offset; i < sd.raw_buffer.size(); i += sd.FormFilter.rx_l) {
                    sd.symbols.push_back(sd.raw_buffer[i]);
                }
                if (sd.symbols.size() > SharedData::MAX_SYMBOLS) {
                    sd.symbols.erase(sd.symbols.begin(), sd.symbols.end() - SharedData::MAX_SYMBOLS);
                }
            } else {
                sd.symbols = sd.raw_buffer;
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
                    sd.fft.fft_magnitude[i] = log10(re * re + im * im + 1e-10); // лог-масштаб
                }

                sd.flags.fft_ready = true;
            }
        }
    }
}

void tx_back(SharedData& sd, SDRConfig &config) {
    int bit_size = 30000;

    while (sd.flags.g_running) {
        sd.bits.resize(bit_size);
        const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
        for (int i = 0; i < 26; ++i) {
            sd.bits[i] = barker13[i % 13];
        }
        for (int i = 26; i < bit_size; ++i) {
            sd.bits[i] = rand() % 2;
        }

        string mod_type;

        if (sd.flags.modulation_index == 0){
            mod_type = "QAM::2";
        } else if (sd.flags.modulation_index == 2){
            mod_type = "QAM::16";
        } else {
            mod_type = "QAM::4";
        }

        vector<complex<double>> signal;

        vector<complex<double>> symbols = modulator(sd.bits.data(), bit_size, mod_type);

        if (sd.flags.ofdm_enabled){
            signal = ofdm_modulator(symbols, ref(sd));
        } else {
            signal = move(symbols);
        }

        vector<complex<double>> final_signal;

        if (sd.flags.upsampling_enabled){
            final_signal = UpSampler(signal.data(), signal.size(), sd.FormFilter.tx_l);
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

            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, to_send, &flags, tx_time, TIMEOUT);

            sent += to_send;
        }
    }
}