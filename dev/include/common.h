#ifndef COMMON_H
#define COMMON_H

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <cstdlib>
#include <complex>
#include <map>
#include <string>
#include <iostream>
#include <ctime>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <fftw3.h>
#include <queue>
#include <array>
#include <condition_variable>

#include <GL/glew.h>
#include <SDL2/SDL.h>

#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_opengl3.h"

constexpr size_t N_BUFFERS = 4;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 8000000;

using namespace std;

struct SharedData;

template<typename T>
class DoubleBuffer {
public:
    void write(const T& data) {
        bufs_[write_idx_] = data;
        published_.store(write_idx_, std::memory_order_release);
        write_idx_ ^= 1;
    }

    bool read(T& out) {
        int idx = published_.load(std::memory_order_acquire);
        if (idx < 0) return false;
        out = bufs_[idx];
        return true;
    }

private:
    std::array<T, 2> bufs_;
    std::atomic<int> published_{-1};
    int write_idx_ = 0;
};

struct SDRConfig {
    SoapySDRDevice* sdr;
    SoapySDRStream* rxStream;
    SoapySDRStream* txStream;
    int rx_mtu;
    int tx_mtu;
    int sample_rate;
    int carrier_freq;
    int16_t* tx_buff;
    int16_t* rx_buffer;
};

struct Flags {
    atomic<bool> g_running{false};
    bool loopback_flag = false;
    bool fft_flag = false;
    bool fft_ready = false;
    bool ofdm_enabled = false;
    bool ofdm_enabled_tx = false;
    bool rx_gain_changed = false;
    bool tx_gain_changed = false;
    bool rx_freq_changed = false;
    bool tx_freq_changed = false;
    bool rx_bw_changed = false;
    bool tx_bw_changed = false;
    bool ofdm_time_est = false;
    bool channel_estimated = false;
    bool cfo_est_enabled = false;
    bool data_est_enabled = false;
    bool ofdm_eq_enabled = false;
    bool ofdm_fft_enabled = false;
    bool cut_begin = false;
    bool tx_regenerate = true;
    bool cp_time_sync = false;
    bool header_dec = false;
    bool ofdm_config_changed = true;
    int modulation_index = 0;
};

struct Fft_conf {
    vector<complex<float>> fft_buffer;
    vector<float> fft_magnitude;
    static constexpr size_t FFT_SIZE = 1024;

    fftw_plan ofdm_fft_plan = nullptr;
    fftw_plan ofdm_ifft_plan = nullptr;
    fftw_plan spectrum_plan = nullptr;
    
    fftw_complex* fft_in = nullptr;
    fftw_complex* fft_out = nullptr;
    
    fftw_complex* ifft_in = nullptr;
    fftw_complex* ifft_out = nullptr;

    fftw_complex* ofdm_rx_in = nullptr;
    fftw_complex* ofdm_rx_out = nullptr;
};

struct device_finder {
    string selected_uri;
    vector<SoapySDRKwargs> pluto_devices;
    int selected_device_index = 0;
};

struct ofdm_conf {
    int n_subcarriers = 128;
    int cp_len = 32;
    int sig_begin = 0;
    int num_pilots = 20;
    int guard_dc = 2;
    int guard_edge = 26;
    vector<int8_t> pilot_idx = {8, 16, 24, 40, 48, 56};

    int padding = 0;
};

struct SyncResult {
    int timing_offset;
    float cfo_estimate = 0;
    uint16_t packet_len = 0;

    vector<complex<float>> reference;
};

struct HammingStats {
    uint64_t blocks_processed = 0;
    uint64_t blocks_with_errors = 0;
    uint64_t bits_corrected = 0;
    uint64_t uncorrectable = 0;

    uint64_t last_reset_time = 0;
    
    std::array<uint32_t, 32> syndrome_hist = {0}; 
};

struct SharedData {
    mutex mtx;
    Flags flags;
    Fft_conf fft;
    device_finder dev_f;
    ofdm_conf ofdm;
    SyncResult ofdm_sync;
    HammingStats Ham_stats;

    DoubleBuffer<std::vector<std::complex<float>>> pipe;

    vector<int16_t> bits;
    vector<int16_t> rx_bits;
    vector<int16_t> interleaved_rx_bits;
    vector<int16_t> tx_samples;
    vector<int16_t> last_tx_samples;
    vector<complex<float>> buffer;
    vector<complex<float>> buffer_without_dsp;
    vector<complex<float>> symbols;

    float avg_time = 0;
    float avg_stream_time = 0;

    static constexpr size_t MAX_SAMPLES = 1920 * 2;
    static constexpr size_t MAX_SYMBOLS = 192 * 2;

    int tx_symbol_count = 256;
    float tx_gain = 30;
    float rx_gain = 20;
    int Threshold = 0;
    int freq = 767000000;
    float rx_bandwidth = 1.92e6;
    float tx_bandwidth = 1.92e6;

    vector<float> timing_offsets;

    size_t bler_total_blocks = 0;
    size_t bler_error_blocks = 0;
    float bler_value = 0.0;

};

void update_scope_buffer(vector<complex<float>>& scope_buffer, const vector<complex<float>>& new_samples, size_t SCOPE_DISPLAY_SIZE);
bool is_guard(int k, SharedData &sd);
void signal_generate(SharedData& sd, SDRConfig &config);
void rebuild_ofdm_plans(SharedData& sd);

void rx_back(SharedData& sd, SDRConfig &config);
void tx_back(SharedData& sd, SDRConfig &config);

void SDRStream(SharedData& sd, SDRConfig &config);

vector<int16_t> calculateCRC16_fromBits(const vector<int16_t> &bits);
bool verifyCRC16(vector<int16_t> &received_bits);

vector<int16_t> hamming_encoder(const vector<int16_t> &bits);
vector<int16_t> hamming_decoder(vector<int16_t> &bits, SharedData &sd);

#endif