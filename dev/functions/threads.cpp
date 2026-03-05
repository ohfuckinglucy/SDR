#include "common.h"
#include "sdr_hw.h"
#include "modulator.h"
#include "ofdm_core.h"
#include "sync_time.h"
#include "sync_freq.h"
#include <iostream>

void rx_back(SharedData& sd, SDRConfig &config){
    vector<complex<double>> local_symbols;
    vector<double> local_fft_mag(sd.fft.FFT_SIZE);

    while (sd.flags.g_running){
        string mod_type;
        if (sd.flags.modulation_index == 0) mod_type = "QAM::2";
        else if (sd.flags.modulation_index == 2) mod_type = "QAM::16";
        else mod_type = "QAM::4";

        size_t filled = sd.pipe.filled_count.load(memory_order_acquire);

        if (filled == 0) {
            asm volatile("pause" ::: "memory");
            continue;
        }

        size_t current_read = sd.pipe.read_idx.load(memory_order_relaxed);

        vector<complex<double>> local_raw_buffer = std::move(sd.pipe.buffers[current_read]);
        sd.pipe.buffers[current_read].clear();

        size_t next_read = (current_read + 1) % Dbuf::NUM_BUFFERS;
        sd.pipe.read_idx.store(next_read, memory_order_release);
        sd.pipe.filled_count.fetch_sub(1, memory_order_acq_rel);

        if (local_raw_buffer.empty()) continue;

        local_symbols.clear();

        if (sd.flags.costas_loop_enabled){
            for (auto& x : local_raw_buffer) {
                x = sd.flags.QAM16_costas_loop ? costas_loop_16qam(sd, x) : costas_loop(sd, x);
            }
        }

        if (sd.flags.ofdm_time_est) {
            sd.ofdm.sig_begin = minn_sync(local_raw_buffer, sd);
            if (sd.flags.loopback_flag) sd.flags.ofdm_time_est = false;
        }

        if (sd.ofdm.sig_begin >= 0 && sd.flags.cut_begin && sd.ofdm.sig_begin < (int)local_raw_buffer.size()){
            local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.sig_begin);
        }

        if (sd.flags.cp_time_sync){
            vector<int> preamble_indices = ofdm_sym_sync(local_raw_buffer, sd);
            sd.ofdm.sym_begin = preamble_indices.empty() ? -1 : preamble_indices[0];
            if (sd.flags.loopback_flag) sd.flags.cp_time_sync = false;
        }

        if (sd.ofdm.sig_begin >= 0 && sd.flags.cut_begin && sd.ofdm.sig_begin < (int)local_raw_buffer.size()){
            local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.n_subcarriers);
        }

        sd.ofdm_sync.packet_len = 0;
        if(sd.flags.header_dec){
            sd.ofdm_sync.packet_len = decode_header(local_raw_buffer, sd);
            if (sd.ofdm.n_subcarriers + sd.ofdm.cp_len < (int)local_raw_buffer.size()){
                local_raw_buffer.erase(local_raw_buffer.begin(), local_raw_buffer.begin() + sd.ofdm.n_subcarriers + sd.ofdm.cp_len);
            }
            if (sd.ofdm_sync.packet_len > 0 && sd.ofdm_sync.packet_len < (int)local_raw_buffer.size()) {
                local_raw_buffer.erase(local_raw_buffer.begin() + sd.ofdm_sync.packet_len, local_raw_buffer.end());
            }
        }

        if (sd.flags.cfo_est_enabled && sd.ofdm.sym_begin >= 0) 
            local_raw_buffer = cfo_est(local_raw_buffer, sd);
        
        if (sd.flags.ofdm_fft_enabled)
            local_raw_buffer = discard_cp(local_raw_buffer, sd);

        if (sd.flags.ofdm_eq_enabled)
            local_raw_buffer = ofdm_equalize(local_raw_buffer, sd);

        if (sd.flags.filter_enabled) 
            filter(local_raw_buffer.data(), local_raw_buffer.size(), sd.form_filter.rx_l);

        if (sd.gardner.sym_sync_enabled) {
            sym_sync(sd, local_raw_buffer);
            sd.flags.used_gardner = true;
            for (size_t i = sd.gardner.ss_offset; i < local_raw_buffer.size(); i += sd.form_filter.rx_l) {
                local_symbols.push_back(local_raw_buffer[i]);
            }
        } else {
            local_symbols = local_raw_buffer;
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
        fftw_execute(sd.fft.spectrum_plan);

        for (size_t i = 0; i < sd.fft.FFT_SIZE; i++) {
            double re = sd.fft.fft_out[i][0];
            double im = sd.fft.fft_out[i][1];
            local_fft_mag[i] = log10(re * re + im * im + 1e-10);
        }
        sd.flags.fft_ready = true;

        {
            lock_guard<mutex> lock(sd.mtx);
            sd.raw_buffer = std::move(local_raw_buffer);
            sd.rx_bits = demodulator(sd.raw_buffer, mod_type);
            sd.symbols = std::move(local_symbols);
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

void SDRStream(SharedData& sd, SDRConfig &config){
    if (!config.sdr || !config.rxStream) {
        cerr << "ERROR: SDR config!" << endl;
        sd.flags.g_running = false;
        return;
    }

    for(size_t i=0; i<Dbuf::NUM_BUFFERS; ++i)
        sd.pipe.buffers[i].reserve(sd.pipe.buffer_size);

    size_t blk = 0;

    while (sd.flags.g_running){
        reconfig_sdr(ref(sd), ref(config));
        
        signal_generate(ref(sd), ref(config));
        
        size_t frame_len = sd.tx_samples.size() / 2;
        size_t num_blocks = (frame_len + config.tx_mtu - 1) / config.tx_mtu;
        size_t total_samples = num_blocks * config.tx_mtu;
        
        if (blk < num_blocks) blk = 0;

        void *rx_buffs[] = {config.rx_buffer};
        int flags = 0;
        long long timeNs = 0;
        
        size_t buf_count = total_samples / config.tx_mtu;
        
        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
        if (sd.flags.loopback_flag){
            const void* tx_buffs[] = { sd.tx_samples.data() + 2 * blk * config.tx_mtu};
            int flags = SOAPY_SDR_HAS_TIME;
            long long tx_time = timeNs + TX_DELAY;
            
            SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, config.tx_mtu, &flags, tx_time, TIMEOUT);
            ++ blk;
        }

        size_t current_write = sd.pipe.write_idx.load(memory_order_relaxed);
        size_t current_filled = sd.pipe.filled_count.load(memory_order_acquire);

        if (current_filled >= Dbuf::NUM_BUFFERS) {
            size_t old_read = sd.pipe.read_idx.load(memory_order_relaxed);
            size_t new_read = (old_read + 1) % Dbuf::NUM_BUFFERS;
            sd.pipe.read_idx.store(new_read, memory_order_release);
            sd.pipe.filled_count.store(Dbuf::NUM_BUFFERS - 1, memory_order_release);
        }

        auto& buf = sd.pipe.buffers[current_write];
        buf.clear();
        int16_t* data_ptr = static_cast<int16_t*>(config.rx_buffer);
        for (int i = 0; i < sr; ++i) {
            buf.emplace_back((double)data_ptr[2*i], (double)data_ptr[2*i+1]);
        }

        size_t next_write = (current_write + 1) % Dbuf::NUM_BUFFERS;
        sd.pipe.write_idx.store(next_write, memory_order_release);

        size_t new_filled = sd.pipe.filled_count.fetch_add(1, memory_order_acq_rel) + 1;
        if (new_filled > Dbuf::NUM_BUFFERS) {
            sd.pipe.filled_count.store(Dbuf::NUM_BUFFERS, memory_order_release);
        }
    }
}