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

void signal_generate(SharedData& sd, SDRConfig &config){
    vector<complex<double>> local_raw_buffer;
    vector<complex<double>> local_symbols;
    size_t tx_sent_idx = 0;
    bool tx_active = false;
    vector<double> local_fft_mag(sd.fft.FFT_SIZE);
    size_t total_samples = 0;

    sd.tx_samples.resize(2 * config.tx_mtu * N_BUFFERS, 0);

    vector<complex<double>> tx_frame;

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
            vector<complex<double>> header = generate_header(data_signal.size() - sd.ofdm.padding, sd);

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

        sd.tx_samples.assign(2 * total_samples, 0);

        for (size_t i = 0; i < frame_len; ++i) {
            sd.tx_samples[2*i] = static_cast<int16_t>(tx_frame[i].real() * scale);
            sd.tx_samples[2*i + 1] = static_cast<int16_t>(tx_frame[i].imag() * scale);
        }
    }
}

void rebuild_ofdm_plans(SharedData& sd) {
    int N = sd.ofdm.n_subcarriers;
    
    usleep(50000);

    if (sd.fft.ofdm_fft_plan) {
        fftw_destroy_plan(sd.fft.ofdm_fft_plan);
        sd.fft.ofdm_fft_plan = nullptr;
    }
    if (sd.fft.ofdm_ifft_plan) {
        fftw_destroy_plan(sd.fft.ofdm_ifft_plan);
        sd.fft.ofdm_ifft_plan = nullptr;
    }
    
    if (sd.fft.ifft_in) {
        fftw_free(sd.fft.ifft_in);
        sd.fft.ifft_in = nullptr;
    }
    if (sd.fft.ifft_out) {
        fftw_free(sd.fft.ifft_out);
        sd.fft.ifft_out = nullptr;
    }
    if (sd.fft.ofdm_rx_in) {
        fftw_free(sd.fft.ofdm_rx_in);
        sd.fft.ofdm_rx_in = nullptr;
    }
    if (sd.fft.ofdm_rx_out) {
        fftw_free(sd.fft.ofdm_rx_out);
        sd.fft.ofdm_rx_out = nullptr;
    }

    if (N <= 0) N = 64;

    sd.fft.ifft_in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
    sd.fft.ifft_out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
    sd.fft.ofdm_rx_in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
    sd.fft.ofdm_rx_out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);

    if (!sd.fft.ifft_in || !sd.fft.ifft_out || !sd.fft.ofdm_rx_in || !sd.fft.ofdm_rx_out) {
        cerr << "FFT malloc failed!" << endl;
        exit(1);
    }

    sd.fft.ofdm_fft_plan = fftw_plan_dft_1d(N, sd.fft.ofdm_rx_in, sd.fft.ofdm_rx_out, FFTW_FORWARD, FFTW_ESTIMATE);
    sd.fft.ofdm_ifft_plan = fftw_plan_dft_1d(N, sd.fft.ifft_in, sd.fft.ifft_out, FFTW_BACKWARD, FFTW_ESTIMATE);

    if (!sd.fft.ofdm_fft_plan || !sd.fft.ofdm_ifft_plan) {
        cerr << "FFT plan creation failed!" << endl;
        exit(1);
    }

    sd.flags.ofdm_config_changed = false;
}