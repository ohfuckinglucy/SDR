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

    const size_t SCOPE_DISPLAY_SIZE = 20000; 
    vector<complex<double>> scope_display_buffer(SCOPE_DISPLAY_SIZE, {0.0, 0.0});

    vector<complex<double>> signal;
    vector<complex<double>> final_signal;
    vector<int16_t> tx_samples(2 * config.tx_mtu * N_BUFFERS, 0);
    vector<complex<double>> local_raw_buffer;
    vector<complex<double>> local_symbols;
    vector<double> local_fft_mag(sd.fft.FFT_SIZE);

    for (size_t samples_sent = 0; sd.flags.g_running; ++samples_sent) {
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
        
                if (sd.flags.loopback_flag){
            string mod_type;
            
            if (sd.flags.modulation_index == 0) mod_type = "QAM::2";
            else if (sd.flags.modulation_index == 2) mod_type = "QAM::16";
            else mod_type = "QAM::4";
            
            int bits_size = bits_per_symbol(mod_type);
            
            vector<complex<double>> current_frame;

            if (sd.flags.ofdm_enabled_tx){
                int Nfft = sd.ofdm.n_subcarriers;
                int CP = sd.ofdm.cp_len;
                int symbol_len = Nfft + CP;
                int pilots_count = sd.ofdm.pilot_idx.size();
                int data_per_sym = Nfft - pilots_count;

                size_t total_buffer_samples = N_BUFFERS * config.tx_mtu;
                size_t preamble_len = symbol_len; 
                
                if (total_buffer_samples > preamble_len) {
                    size_t available_for_data = total_buffer_samples - preamble_len;
                    int num_data_symbols = available_for_data / symbol_len;
                    
                    if (num_data_symbols < 1) num_data_symbols = 1;

                    size_t total_data_symbols_count = num_data_symbols * data_per_sym;
                    
                    sd.bits.resize(total_data_symbols_count * bits_size);
                    const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
                    for (int i = 0; i < 26 && i < (int)sd.bits.size(); ++i) sd.bits[i] = barker13[i % 13];
                    for (size_t i = 26; i < sd.bits.size(); ++i) sd.bits[i] = rand() % 2;

                    vector<complex<double>> symbols = modulator(sd.bits, sd.bits.size(), mod_type);
                    
                    if (!symbols.empty()) {
                        vector<complex<double>> preamble = preamble_generate(ref(sd));
                        vector<complex<double>> freq_blocks = insert_pilots(symbols, ref(sd));
                        vector<complex<double>> data_signal = ofdm_modulator(freq_blocks, ref(sd));

                        current_frame.reserve(preamble.size() + data_signal.size());
                        current_frame.insert(current_frame.end(), preamble.begin(), preamble.end());
                        current_frame.insert(current_frame.end(), data_signal.begin(), data_signal.end());
                    }
                }
            } else {
                size_t raw_max_symbols = config.tx_mtu / sd.FormFilter.tx_l;
                if (raw_max_symbols == 0) raw_max_symbols = 1;
                
                sd.bits.resize(raw_max_symbols * bits_size);
                const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
                for (int i = 0; i < 26 && i < (int)sd.bits.size(); ++i) sd.bits[i] = barker13[i % 13];
                for (size_t i = 26; i < sd.bits.size(); ++i) sd.bits[i] = rand() % 2;

                vector<complex<double>> symbols = modulator(sd.bits, sd.bits.size(), mod_type);
                
                if (!symbols.empty()) {
                    if (sd.flags.upsampling_enabled){
                        current_frame = UpSampler(symbols, sd.FormFilter.tx_l);
                        if (sd.flags.tx_filter) filter(current_frame.data(), current_frame.size(), sd.FormFilter.tx_l);
                    } else {
                        current_frame = move(symbols);
                    }
                }
            }

            if (!current_frame.empty()) {
                size_t samples_to_gen = min(current_frame.size(), (size_t)(N_BUFFERS * config.tx_mtu));
                
                fill(tx_samples.begin(), tx_samples.end(), 0); 
                
                for (size_t i = 0; i < samples_to_gen; i++) {
                    double scale = 12000.0; 
                    tx_samples[2 * i]   = static_cast<int16_t>(real(current_frame[i]) * scale);
                    tx_samples[2 * i + 1] = static_cast<int16_t>(imag(current_frame[i]) * scale);
                }
                
            } else {
                fill(tx_samples.begin(), tx_samples.end(), 0);
            }
        }
        
        void *rx_buffs[] = {config.rx_buffer};
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);

        if (sd.flags.loopback_flag && !tx_samples.empty()) {
            long long tx_time = timeNs + TX_DELAY;
            flags = SOAPY_SDR_HAS_TIME;

            size_t tx_idx = samples_sent % N_BUFFERS;
            const void* tx_buffs[] = { tx_samples.data() + (tx_idx * config.tx_mtu * 2) };
            SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, config.tx_mtu, &flags, tx_time, TIMEOUT);
        }
        
        int16_t* data_ptr = static_cast<int16_t*>(rx_buffs[0]);
        
        local_raw_buffer.clear();
        local_raw_buffer.reserve(sr);
        local_symbols.clear();

        if (sr > 0){
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

            if (!local_raw_buffer.empty()) {
                if (local_raw_buffer.size() >= SCOPE_DISPLAY_SIZE) {
                    scope_display_buffer = vector<complex<double>>(local_raw_buffer.end() - SCOPE_DISPLAY_SIZE, local_raw_buffer.end());
                } else {
                    size_t shift = local_raw_buffer.size();
                    if (shift >= SCOPE_DISPLAY_SIZE) {
                        scope_display_buffer = local_raw_buffer;
                        if (scope_display_buffer.size() > SCOPE_DISPLAY_SIZE) 
                            scope_display_buffer.resize(SCOPE_DISPLAY_SIZE);
                    } else {
                        for (size_t i = 0; i < SCOPE_DISPLAY_SIZE - shift; ++i) {
                            scope_display_buffer[i] = scope_display_buffer[i + shift];
                        }
                        for (size_t i = 0; i < shift; ++i) {
                            scope_display_buffer[SCOPE_DISPLAY_SIZE - shift + i] = local_raw_buffer[i];
                        }
                    }
                }
                sd.last_rx_count = SCOPE_DISPLAY_SIZE; 
            }

            if (sd.flags.ofdm_time_est){
                sd.ofdm.sig_begin = time_est(local_raw_buffer, ref(sd));
                local_raw_buffer = cfo_est(local_raw_buffer, sd);

                if (sd.ofdm.sig_begin >= 0){
                    local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.sig_begin);
                }

                local_raw_buffer = discard_cp(local_raw_buffer, ref(sd));
                local_raw_buffer = ofdm_equalize(local_raw_buffer, ref(sd));
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
                std::lock_guard<std::mutex> lock(sd.mtx);
                
                sd.last_rx_count = sr;
                sd.raw_buffer = local_raw_buffer;
                sd.symbols = local_symbols;

                sd.scope_buffer = scope_display_buffer; 
                sd.scope_filled = true; 

                if (sd.flags.fft_flag && sd.flags.fft_ready) {
                    sd.fft.fft_magnitude = local_fft_mag;
                }
            }
        }
    }
}

void tx_back(SharedData& sd, SDRConfig &config) {
    while (sd.flags.g_running) {
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
        
        if (sd.flags.modulation_index == 0){
            mod_type = "QAM::2";
        } else if (sd.flags.modulation_index == 2){
            mod_type = "QAM::16";
        } else {
            mod_type = "QAM::4";
        }
        
        int bits_size = bits_per_symbol(mod_type);
        size_t raw_max_symbols = config.tx_mtu / sd.FormFilter.tx_l;
        size_t data_per_symbol = 0;

        if (sd.flags.ofdm_enabled){
            int Nfft = sd.ofdm.n_subcarriers;
            int CP   = sd.ofdm.cp_len;
            int ofdm_symbol_len = Nfft + CP;

            int data_per_ofdm_symbol = Nfft - sd.ofdm.pilot_idx.size();

            size_t ofdm_symbols_max = config.tx_mtu / ofdm_symbol_len;

            if (ofdm_symbols_max == 0)
                ofdm_symbols_max = 1;

            size_t total_data_symbols = ofdm_symbols_max * data_per_ofdm_symbol;

            raw_max_symbols = total_data_symbols;
        }

        vector<int16_t> bits(raw_max_symbols * bits_size);
        const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
        for (int i = 0; i < 26; ++i) {
            bits[i] = barker13[i % 13];
        }
        for (size_t i = 26; i < bits.size(); ++i) {
            bits[i] = rand() % 2;
        }

        vector<complex<double>> signal;
        vector<complex<double>> final_signal;

        vector<complex<double>> symbols = modulator(bits, bits.size(), mod_type);

        if (sd.flags.ofdm_enabled){
            vector<complex<double>> preamble = preamble_generate(ref(sd));
            vector<complex<double>> freq_blocks = insert_pilots(symbols, ref(sd));
            signal = ofdm_modulator(freq_blocks, ref(sd));

            final_signal = move(preamble); 
            final_signal.insert(final_signal.end(), signal.begin(), signal.end());
        } else {
            signal = move(symbols);

            if (sd.flags.upsampling_enabled){
                final_signal = UpSampler(signal, sd.FormFilter.tx_l);
                if (sd.flags.tx_filter) {
                    filter(final_signal.data(), final_signal.size(), sd.FormFilter.tx_l);
                }
            } else {
                final_signal = move(signal);
            }
        }

        vector<int16_t> tx_samples(2 * final_signal.size());
        for (size_t i = 0; i < final_signal.size(); i++) {
            tx_samples[2*i]   = static_cast<int16_t>(real(final_signal[i]) * 16000);
            tx_samples[2*i+1] = static_cast<int16_t>(imag(final_signal[i]) * 16000);
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

            SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, config.rx_mtu, &flags, tx_time, TIMEOUT);

            sent += to_send;
        }
    }
}