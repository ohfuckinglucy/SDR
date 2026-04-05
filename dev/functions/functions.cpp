#include <unistd.h>

#include "common.h"
#include "error_interleaving.hpp"
#include "logger.hpp"
#include "modulator.h"
#include "ofdm_core.h"
#include "sync_time.h"

void signal_generate(SharedData &sd, SDRConfig &config) {
    std::vector<std::complex<float>> local_raw_buffer;
    std::vector<std::complex<float>> local_symbols;
    std::vector<float> local_fft_mag(sd.fft.FFT_SIZE);
    size_t total_samples = 0;

    sd.tx_samples.resize(2 * config.tx_mtu * N_BUFFERS, 0);

    std::vector<std::complex<float>> tx_frame;

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

    if (sd.flags.tx_regenerate) {
        std::vector<int16_t> CRC;
        std::vector<std::complex<float>> frame;

        int bits_ps = bits_per_symbol(mod_type);

        size_t total_symbols = sd.tx_symbol_count;

        if (sd.flags.ofdm_enabled_tx) {
            int data_per_symbol = sd.ofdm.n_subcarriers - sd.ofdm.pilot_idx.size();

            int ofdm_blocks = ceil((float) total_symbols / data_per_symbol);

            total_symbols = ofdm_blocks * data_per_symbol;
        }

        sd.bits.resize(total_symbols * bits_ps);

        for (size_t i = 0; i < sd.bits.size(); ++i)
            sd.bits[i] = rand() % 2;

        CRC = calculateCRC16_fromBits(sd.bits);

        for (int16_t bit : CRC) {
            sd.bits.push_back(bit);
        }

        std::vector<int16_t> encoded_bits = hamming_encoder(sd.bits);

        size_t remainder = encoded_bits.size() % bits_ps;
        if (remainder != 0) {
            size_t padding = bits_ps - remainder;
            for (size_t i = 0; i < padding; ++i)
                encoded_bits.push_back(0);
        }

        std::vector<std::complex<float>> symbols = modulator(encoded_bits, encoded_bits.size(), mod_type);

        if (sd.flags.ofdm_enabled_tx) {
            std::vector<std::complex<float>> preamble = generate_zc_preamble(sd);
            std::vector<std::complex<float>> freq_blocks = insert_pilots(symbols, sd);
            std::vector<std::complex<float>> data_signal = ofdm_modulator(freq_blocks, sd);
            std::vector<std::complex<float>> header = generate_header(symbols.size(), sd);

            tx_frame.reserve(preamble.size() + data_signal.size());
            tx_frame.insert(tx_frame.end(), preamble.begin(), preamble.end());
            tx_frame.insert(tx_frame.end(), header.begin(), header.end());
            tx_frame.insert(tx_frame.end(), data_signal.begin(), data_signal.end());
        } else {
            tx_frame = std::move(symbols);
        }

        sd.flags.tx_regenerate = false;
    }

    size_t num_blocks;
    if (sd.flags.loopback_flag && !tx_frame.empty()) {
        float scale = 12000.0;
        if (sd.flags.ofdm_enabled_tx)
            scale = 120000.0;

        size_t frame_len = tx_frame.size();

        num_blocks = (frame_len + config.tx_mtu - 1) / config.tx_mtu;
        total_samples = num_blocks * config.tx_mtu;

        sd.tx_samples.assign(2 * total_samples, 0);

        for (size_t i = 0; i < frame_len; ++i) {
            sd.tx_samples[2 * i] = static_cast<int16_t>(tx_frame[i].real() * scale);
            sd.tx_samples[2 * i + 1] = static_cast<int16_t>(tx_frame[i].imag() * scale);
        }
    }
}

void rebuild_ofdm_plans(SharedData &sd) {
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

    if (N <= 0)
        N = 128;

    sd.fft.ifft_in = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N);
    sd.fft.ifft_out = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N);
    sd.fft.ofdm_rx_in = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N);
    sd.fft.ofdm_rx_out = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N);

    if (!sd.fft.ifft_in || !sd.fft.ifft_out || !sd.fft.ofdm_rx_in || !sd.fft.ofdm_rx_out) {
        logs::dsp.warn("FFT malloc failed! strerror {} errno {}", strerror(errno), errno);
        exit(1);
    }

    sd.fft.ofdm_fft_plan = fftw_plan_dft_1d(N, sd.fft.ofdm_rx_in, sd.fft.ofdm_rx_out, FFTW_FORWARD, FFTW_ESTIMATE);
    sd.fft.ofdm_ifft_plan = fftw_plan_dft_1d(N, sd.fft.ifft_in, sd.fft.ifft_out, FFTW_BACKWARD, FFTW_ESTIMATE);

    if (!sd.fft.ofdm_fft_plan || !sd.fft.ofdm_ifft_plan) {
        logs::dsp.warn("FFT plan creation failed! strerror {} errno {}", strerror(errno), errno);
        exit(1);
    }

    sd.flags.ofdm_config_changed = false;
}

float SNR_calculation(const std::vector<std::complex<float>> &signal, SharedData &sd) {
    const size_t NOISE_WIN = 32;
    const size_t SIGNAL_WIN = 256;

    if (signal.size() < NOISE_WIN + SIGNAL_WIN)
        return 0.0f;

    float sum_noise = 0.0f;
    for (size_t i = 0; i < NOISE_WIN; ++i)
        sum_noise += norm(signal[i]);
    float rms_noise = sqrtf(sum_noise / NOISE_WIN);
    if (rms_noise < 1e-4f)
        rms_noise = 1e-4f;

    size_t sig_start = signal.size() / 4;
    float sum_signal = 0.0f;
    for (size_t i = sig_start; i < sig_start + SIGNAL_WIN; ++i)
        sum_signal += norm(signal[i]);
    float rms_signal = sqrtf(sum_signal / SIGNAL_WIN);

    return 20.0f * log10f(rms_signal / rms_noise);
}

static std::complex<float> find_nearest_symbol(std::complex<float> received,
                                               const std::vector<std::complex<float>> &constellation) {
    float min_dist = 1e30f;
    std::complex<float> best = constellation[0];

    for (const auto &sym : constellation) {
        float dist = norm(received - sym);
        if (dist < min_dist) {
            min_dist = dist;
            best = sym;
        }
    }
    return best;
}

float calculate_EVM(const std::vector<std::complex<float>> &received,
                    const std::vector<std::complex<float>> &constellation) {
    if (received.empty() || constellation.empty())
        return 100.0f;

    float error_power = 0.0f;
    float signal_power = 0.0f;

    for (const auto &sym : received) {
        std::complex<float> ideal = find_nearest_symbol(sym, constellation);
        error_power += norm(sym - ideal);
        signal_power += norm(ideal);
    }

    if (signal_power < 1e-10f)
        return 100.0f;

    return 100.0f * sqrtf(error_power / signal_power);
}