#include "common.h"
#include "sdr_hw.h"
#include "modulator.h"
#include "ofdm_core.h"
#include "sync_time.h"
#include "sync_freq.h"
#include <iostream>

const vector<int16_t> barker = {
    0,0,0,0,0,1,1,0,0,1,0,1,0
};

const size_t barker_len = barker.size();

void rx_back(SharedData& sd, SDRConfig &config){
    if (!config.sdr || !config.rxStream) {
        cerr << "ERROR: SDR config!" << endl;
        sd.flags.g_running = false;
        return;
    }

    vector<complex<double>> local_raw_buffer;
    vector<complex<double>> local_symbols;
    size_t tx_sent_idx = 0;
    bool tx_active = false;
    vector<double> local_fft_mag(sd.fft.FFT_SIZE);
    size_t total_samples = 0;

    vector<int16_t> tx_samples(2 * config.tx_mtu * N_BUFFERS, 0);

    vector<complex<double>> tx_frame;

    for (size_t samples_sent = 0; sd.flags.g_running; ++samples_sent){
        reconfig_sdr(ref(sd), ref(config));

        string mod_type;
        if (sd.flags.modulation_index == 0) mod_type = "QAM::2";
        else if (sd.flags.modulation_index == 1) mod_type = "QAM::4";
        else mod_type = "QAM::16";

        if (sd.flags.tx_regenerate){
            vector<complex<double>> frame; 

            int bits_ps = bits_per_symbol(mod_type);

            size_t total_symbols = sd.tx_symbol_count;

            if (sd.flags.ofdm_enabled_tx) {
                int data_per_symbol = sd.ofdm.n_subcarriers - sd.ofdm.pilot_idx.size();

                int ofdm_blocks = ceil((double)total_symbols / data_per_symbol);

                total_symbols = ofdm_blocks * data_per_symbol;
            }

            sd.bits.resize(total_symbols * bits_ps);

            if (sd.bits.size() < 26) sd.bits.resize(100);

            for (size_t i = 0; i < barker_len; ++i)
                sd.bits[i] = barker[i];

            for (size_t i = 0; i < barker_len; ++i)
                sd.bits[barker_len + i] = barker[i];

            for (size_t i = 2 * barker_len; i < sd.bits.size(); ++i)
                sd.bits[i] = rand() % 2;

            vector<complex<double>> symbols = modulator(sd.bits, sd.bits.size(), mod_type);

            if (sd.flags.ofdm_enabled_tx){
                vector<complex<double>> preamble = generate_minn_preamble(sd);
                vector<complex<double>> freq_blocks = insert_pilots(symbols, sd);
                vector<complex<double>> data_signal = ofdm_modulator(freq_blocks, sd);
                vector<complex<double>> header = generate_header(data_signal.size(), sd);

                frame.reserve(preamble.size() + data_signal.size());
                frame.insert(frame.end(), preamble.begin(), preamble.end());
                frame.insert(frame.end(), header.begin(), header.end());
                frame.insert(frame.end(), data_signal.begin(), data_signal.end());
            } else {
                frame = move(symbols);
            }

            tx_frame.clear();
            if (!frame.empty()) {
                if (sd.flags.upsampling_enabled) {
                    tx_frame = UpSampler(frame, sd.form_filter.tx_l);
                    if (sd.flags.tx_filter) filter(tx_frame.data(), tx_frame.size(), sd.form_filter.tx_l);
                } else {
                    tx_frame = move(frame);
                }
            }

            tx_sent_idx = 0;
            tx_active = !tx_frame.empty();
            sd.flags.tx_regenerate = false;
        }

        size_t num_blocks;
        if (sd.flags.loopback_flag && !tx_frame.empty()) {
            double scale = 12000.0;
            if (sd.flags.ofdm_enabled_tx) scale = 120000.0;

            size_t frame_len = tx_frame.size();
            
            num_blocks = (frame_len + config.tx_mtu - 1) / config.tx_mtu;
            total_samples = num_blocks * config.tx_mtu;

            tx_samples.assign(2 * total_samples, 0);

            for (size_t i = 0; i < frame_len; ++i) {
                tx_samples[2*i] = static_cast<int16_t>(tx_frame[i].real() * scale);
                tx_samples[2*i + 1] = static_cast<int16_t>(tx_frame[i].imag() * scale);
            }
        }

        void *rx_buffs[] = {config.rx_buffer};
        int flags = 0;
        long long timeNs = 0;
        
        size_t buf_count = total_samples / config.tx_mtu;

        int sr = SoapySDRDevice_readStream(
            config.sdr,
            config.rxStream,
            rx_buffs,
            config.rx_mtu,
            &flags,
            &timeNs,
            TIMEOUT);

        if (sd.flags.loopback_flag && !tx_samples.empty() && total_samples > 0) {
            for (size_t blk = 0; blk < num_blocks; ++blk) {
                const void* tx_buffs[] = { tx_samples.data() + 2 * blk * config.tx_mtu };
                int flags = SOAPY_SDR_HAS_TIME;
                long long tx_time = timeNs + TX_DELAY;

                SoapySDRDevice_writeStream(
                    config.sdr,
                    config.txStream,
                    tx_buffs,
                    config.tx_mtu,
                    &flags,
                    tx_time,
                    TIMEOUT
                );
            }

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

        if (sd.flags.ofdm_time_est) {
            // sd.ofdm.sig_begin = shmidt_sync(local_raw_buffer, sd);
            sd.ofdm.sig_begin = minn_sync(local_raw_buffer, sd);

            if (sd.flags.loopback_flag){
                sd.flags.ofdm_time_est = false;
            }
        }
        
        if (sd.ofdm.sig_begin >= 0 && sd.flags.cut_begin){
            local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.sig_begin);
        }

        if (sd.flags.cfo_est_enabled){
            // local_raw_buffer = freq_sync(local_raw_buffer, sd);
            // local_raw_buffer = cfo_sync_shmid_cox(local_raw_buffer, sd);
        }

        if (sd.flags.cp_time_sync){
            vector<int> preamble_indices = ofdm_sym_sync(local_raw_buffer, sd);
            if (!preamble_indices.empty()) {
                sd.ofdm.sym_begin = preamble_indices[0];
            } else {
                sd.ofdm.sym_begin = -1;
            }
            if (sd.flags.loopback_flag){
                sd.flags.cp_time_sync = false;
            }
        }
        

        if (sd.ofdm.sig_begin >= 0 && sd.flags.cut_begin){
            local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.n_subcarriers);
        }

        sd.ofdm_sync.packet_len = 0;
        if(sd.flags.header_dec){
            sd.ofdm_sync.packet_len = decode_header(local_raw_buffer, sd);

            if (sd.ofdm.n_subcarriers + sd.ofdm.cp_len < local_raw_buffer.size()){
                local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.n_subcarriers + sd.ofdm.cp_len);
            }

            if (sd.ofdm_sync.packet_len > 0){
                if (sd.ofdm_sync.packet_len >= local_raw_buffer.size()){

                } else {
                    local_raw_buffer.erase(local_raw_buffer.begin() + sd.ofdm_sync.packet_len, local_raw_buffer.end());
                }
            }
        }
        
        if (sd.flags.cfo_est_enabled && sd.ofdm.sym_begin >= 0) local_raw_buffer = cfo_est(local_raw_buffer, sd);
        
        if (sd.flags.ofdm_fft_enabled){
            local_raw_buffer = discard_cp(local_raw_buffer, sd);
        }

        if (sd.flags.ofdm_eq_enabled){
            local_raw_buffer = ofdm_equalize(local_raw_buffer, sd);
        }

        if (sd.flags.filter_enabled) filter(local_raw_buffer.data(), local_raw_buffer.size(), sd.form_filter.rx_l);

        if (sd.gardner.sym_sync_enabled) {
            sym_sync(sd, local_raw_buffer);
            sd.flags.used_gardner = true;
            for (size_t i = sd.gardner.ss_offset; i < local_raw_buffer.size(); i += sd.form_filter.rx_l) {
                local_symbols.push_back(local_raw_buffer[i]);
            }
        } else {
            local_symbols = (local_raw_buffer);
            sd.flags.used_gardner = false;
        }

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

        {
            lock_guard<mutex> lock(sd.mtx);

            sd.raw_buffer = local_raw_buffer;
            sd.rx_bits = demodulator(local_raw_buffer, mod_type);
            sd.symbols = local_symbols;

            sd.fft.fft_magnitude = local_fft_mag;
        }
    }
}

void tx_back(SharedData& sd, SDRConfig &config){
    string last_mod_type = "";
    int bits_size = 0;

    size_t tx_sent_idx = 0;
    bool tx_active = false;

    vector<complex<double>> tx_frame;
    vector<int16_t> tx_samples(2 * config.tx_mtu * N_BUFFERS, 0);
    
    while (sd.flags.g_running) {
        static bool bits_initialized = false;
        
        reconfig_sdr(ref(sd), ref(config));
        
        string mod_type;
        if (sd.flags.modulation_index == 0) mod_type = "QAM::2";
        else if (sd.flags.modulation_index == 2) mod_type = "QAM::16";
        else mod_type = "QAM::4";
        
        if (sd.flags.tx_regenerate){
            vector<complex<double>> frame; 
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
                vector<complex<double>> preamble = generate_minn_preamble(sd);
                vector<complex<double>> freq_blocks = insert_pilots(symbols, sd);
                vector<complex<double>> data_signal = ofdm_modulator(freq_blocks, sd);
                vector<complex<double>> header = generate_header(data_signal.size(), sd);

                frame.reserve(preamble.size() + data_signal.size());
                frame.insert(frame.end(), preamble.begin(), preamble.end());
                frame.insert(frame.end(), header.begin(), header.end());
                frame.insert(frame.end(), data_signal.begin(), data_signal.end());
            } else {
                frame = move(symbols);
            }

            tx_frame.clear();
            if (!frame.empty()) {
                if (sd.flags.upsampling_enabled) {
                    tx_frame = UpSampler(frame, sd.form_filter.tx_l);
                    if (sd.flags.tx_filter) filter(tx_frame.data(), tx_frame.size(), sd.form_filter.tx_l);
                } else {
                    tx_frame = move(frame);
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
                if (sd.flags.ofdm_enabled_tx) scale = 120000.0;

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