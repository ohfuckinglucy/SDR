#pragma once

#include <SoapySDR/Device.h>
#include <array>
#include <atomic>
#include <complex>
#include <cstddef>
#include <fftw3.h>
#include <mutex>
#include <vector>

constexpr size_t N_BUFFERS = 4;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 8000000;

struct SharedData;

template <typename T>
class DoubleBuffer {
  public:
    void write(const T &data)
    {
        bufs_[write_idx_] = data;
        published_.store(write_idx_, std::memory_order_release);
        write_idx_ ^= 1;
    }

    bool read(T &out)
    {
        int idx = published_.load(std::memory_order_acquire);
        if (idx < 0)
            return false;
        out = bufs_[idx];
        return true;
    }
  private:
    std::array<T, 2> bufs_;
    std::atomic<int> published_{ -1 };
    int write_idx_ = 0;
};

struct SDRConfig
{
    SoapySDRDevice *sdr;
    SoapySDRStream *rxStream;
    SoapySDRStream *txStream;
    int rx_mtu;
    int tx_mtu;
    int sample_rate;
    int carrier_freq;
    int16_t *tx_buff;
    int16_t *rx_buffer;
};

struct Flags
{
    std::atomic<bool> g_running{ false };
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

struct Fft_conf
{
    std::vector<std::complex<float>> fft_buffer;
    std::vector<float> fft_magnitude;
    static constexpr size_t FFT_SIZE = 1920;

    fftw_plan ofdm_fft_plan = nullptr;
    fftw_plan ofdm_ifft_plan = nullptr;
    fftw_plan spectrum_plan = nullptr;

    fftw_complex *fft_in = nullptr;
    fftw_complex *fft_out = nullptr;

    fftw_complex *ifft_in = nullptr;
    fftw_complex *ifft_out = nullptr;

    fftw_complex *ofdm_rx_in = nullptr;
    fftw_complex *ofdm_rx_out = nullptr;
};

struct device_finder
{
    std::string selected_uri;
    std::vector<SoapySDRKwargs> pluto_devices;
    int selected_device_index = 0;
};

struct ofdm_conf
{
    int n_subcarriers = 128;
    int cp_len = 32;
    int sig_begin = 0;
    int num_pilots = 10;
    int guard_dc = 1;
    int guard_edge = 28;
    std::vector<int8_t> pilot_idx = { 8, 16, 24, 40, 48, 56 };

    int padding = 0;
};

struct SyncResult
{
    int timing_offset = -1;
    float cfo_estimate = 0;
    uint16_t packet_len = 0;

    std::vector<std::complex<float>> reference;
    std::vector<std::complex<float>> preamble_freq;
};

struct HammingStats
{
    uint64_t blocks_processed = 0;
    uint64_t blocks_with_errors = 0;
    uint64_t bits_corrected = 0;
    uint64_t uncorrectable = 0;

    uint64_t last_reset_time = 0;

    std::array<uint32_t, 32> syndrome_hist = { 0 };
};

struct SharedData
{
    std::mutex mtx;
    Flags flags;
    Fft_conf fft;
    device_finder dev_f;
    ofdm_conf ofdm;
    SyncResult ofdm_sync;
    HammingStats Ham_stats;

    DoubleBuffer<std::vector<std::complex<float>>> pipe;

    std::vector<int16_t> bits;
    std::vector<int16_t> rx_bits;
    std::vector<int16_t> interleaved_rx_bits;
    std::vector<int16_t> tx_samples;
    std::vector<int16_t> last_tx_samples;
    std::vector<std::complex<float>> buffer;
    std::vector<std::complex<float>> buffer_without_dsp;
    std::vector<std::complex<float>> symbols;

    std::vector<float> SNR_vec;
    std::vector<float> EVM_vec;
    int snr_vec_offset = 0;
    int snr_vec_size = 1920 * 10;
    size_t frames_processed = 0;

    float avg_time = 0;
    float avg_stream_time = 0;

    static constexpr size_t MAX_SAMPLES = 1920 * 2;
    static constexpr size_t MAX_SYMBOLS = 192 * 2;

    int tx_symbol_count = 256;
    float tx_gain = 30;
    float rx_gain = 20;
    int Threshold = 0;
    float freq = 2.2e9;
    float rx_bandwidth = 1.92e6;
    float trx_bandwidth = 10.0e6;

    float EVM = 0;
    float SNR_DB = 0;

    std::vector<float> timing_offsets;

    size_t bler_total_blocks = 0;
    size_t bler_error_blocks = 0;
    float bler_value = 0.0;
};

void update_scope_buffer(std::vector<std::complex<float>> &scope_buffer, const std::vector<std::complex<float>> &new_samples, size_t SCOPE_DISPLAY_SIZE);
bool is_guard(int k, SharedData &sd);
void signal_generate(SharedData &sd, SDRConfig &config);
void rebuild_ofdm_plans(SharedData &sd);

void rx_back(SharedData &sd);
void tx_back(SharedData &sd, SDRConfig &config);

void SDRStream(SharedData &sd, SDRConfig &config);

float SNR_calculation(const std::vector<std::complex<float>> &signal);
float calculate_EVM(const std::vector<std::complex<float>> &received, const std::vector<std::complex<float>> &constellation);

std::vector<int16_t> pic_read();
void pic_write(std::vector<int16_t> buffer);