#pragma once

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Types.hpp>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <fftw3.h>
#include <mutex>
#include <string>
#include <sys/types.h>
#include <vector>

namespace SDRField
{
    const uint16_t None = 0;
    const uint16_t RxGain = 1 << 0;
    const uint16_t TxGain = 1 << 1;
    const uint16_t RxFreq = 1 << 2;
    const uint16_t TxFreq = 1 << 3;
    const uint16_t RxBW = 1 << 4;
    const uint16_t TxBW = 1 << 5;
    const uint16_t RxSampleRate = 1 << 6;
    const uint16_t TxSampleRate = 1 << 7;
    const uint16_t OFDMConfig = 1 << 8;
} // namespace SDRField

namespace FrameFlag
{
    const uint8_t None = 0;
    const uint8_t IsFirst = 1 << 0;
    const uint8_t IsLast = 1 << 1;
    const uint8_t IsCrc = 1 << 2;
} // namespace FrameFlag

enum class SignalType
{
    Random,
    Text,
    File
};

enum class SignalModulation
{
    BPSK,
    QPSK,
    QAM16,
    QAM64
};

template <typename T>
class DoubleBuffer {
  public:
    void write(const T &data)
    {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            bufs_[write_idx_] = data;
            published_idx_ = write_idx_;
            has_data_ = true;
            write_idx_ ^= 1;
        }
        cv_.notify_one();
    }

    void read(T &out)
    {
        std::unique_lock<std::mutex> lock(mtx_);

        cv_.wait(lock, [this]
                 { return has_data_; });

        out = bufs_[published_idx_];
        has_data_ = false;
    }

    bool try_read(T &out)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_data_)
            return false;

        out = bufs_[published_idx_];
        has_data_ = false;
        return true;
    }
  private:
    std::array<T, 2> bufs_;
    std::mutex mtx_;
    std::condition_variable cv_;

    int write_idx_ = 0;
    int published_idx_ = -1;
    bool has_data_ = false;
};

struct SDRcfg
{
    uint16_t dirty_mask = 0;
    float rx_gain = 7;
    float tx_gain = 89;
    float rx_freq = 2.2e9;
    float tx_freq = 2.2e9;
    float rx_bw = 10e6;
    float tx_bw = 10e6;
    float rx_sample_rate = 1.92e6;
    float tx_sample_rate = 1.92e6;
};

struct FFTPlans
{
    int N_spec = 1920;
    fftwf_complex *in_spectre;
    fftwf_complex *out_spectre;
    fftwf_plan plan_spectre;

    fftwf_complex *in_ifft;
    fftwf_complex *out_ifft;
    fftwf_plan plan_ifft;

    fftwf_complex *in_fft;
    fftwf_complex *out_fft;
    fftwf_plan plan_fft;
};

struct OFDMcfg
{
    int N = 128;
    int CP = 16;
    int num_pilots = 4;
    std::vector<uint8_t> pilot_idx;
    std::vector<std::complex<float>> zc_reference;

    int16_t q = 5;
    int mystery_offset = -1;
    int best_idx = 0;

    float cfo_est = 0;
};

struct DSPFlags
{
    bool PSS = false;
    bool FFT = false;
    bool CFO = false;
    bool EQ = false;
};

struct Stats
{
    float EVM;
    std::vector<float> EVM_vec;
    float SNR;
    std::vector<float> SNR_vec;

    size_t vec_offset = 0;
    size_t vec_size = 1920;
    size_t frames_processed = 0;

    ssize_t total_block_proccesed = 0;
    ssize_t error_block = 0;
    float BLER = 0;
};

struct Header
{
    bool is_valid = false;
    size_t num_samples = 0;
    SignalModulation modulation;
    uint8_t flag;
    SignalType sig_type;
};

static constexpr size_t FILE_CHUNK_BYTES = 100;

struct SharedData
{
    SDRcfg SDR;
    FFTPlans fftplans;
    OFDMcfg ofdmcfg;
    DSPFlags dspflags;
    Stats stats;

    std::mutex mtx;
    std::atomic<bool> allRunning = true;
    bool tx_continuous = false;
    bool tx_once = false;
    bool sig_changed = false;
    std::string tx_text = "text";
    std::string decoded_text;

    bool is_first = false;
    bool is_last = false;

    DoubleBuffer<std::vector<std::complex<float>>> pipe;

    SignalType type_of_signal;
    SignalModulation type_of_modulation;

    Header hdr;

    std::vector<int16_t> tx_samples;
    std::vector<std::complex<float>> gui_buffer;
    std::vector<float> spectrum;
    std::vector<float> timing_offsets;
    std::vector<float> gui_timing_offsets;

    size_t num_samples = 100;

    float avg_stream_time = 0;
    float avg_dsp_time = 0;

    std::string tx_file_path;
    std::string tx_file_name;
    std::vector<uint8_t> tx_file_data;
    bool tx_file_loaded = false;
    size_t tx_file_chunk_idx = 0;
    size_t tx_file_total_chunks = 0;

    std::string rx_file_name;
    std::vector<uint8_t> rx_file_chunks_buf;
    bool file_received = false;
    std::string rx_file_save_path;
    size_t rx_file_expected_total = 0;
};

constexpr uint8_t MAGIC_NUMBER = 0x5A;

class SDR {
  private:
    SoapySDR::Device *device = nullptr;
    SoapySDR::Stream *rxStream = nullptr;
    SoapySDR::Stream *txStream = nullptr;

    double sample_rate = 1.92e6;
    double carrier_freq = 7.67e8;

    long long timeNs = 0;
    long long TimeOut = 400000;
    long long TxDelay = 8000000;
  public:
    std::vector<int16_t> rx_buffer;
    std::vector<int16_t> tx_buffer;
    size_t rx_mtu = 0;
    size_t tx_mtu = 0;

    SDR(const std::string &uri)
    {
        SoapySDR::Kwargs args;
        args["driver"] = "plutosdr";
        args["uri"] = uri;
        args["direct"] = "1";
        args["timestamp_every"] = "1920";
        args["loopback"] = "0";

        device = SoapySDR::Device::make(args);

        device->setSampleRate(SOAPY_SDR_RX, 0, sample_rate);
        device->setFrequency(SOAPY_SDR_RX, 0, carrier_freq);
        device->setSampleRate(SOAPY_SDR_TX, 0, sample_rate);
        device->setFrequency(SOAPY_SDR_TX, 0, carrier_freq);

        device->setGain(SOAPY_SDR_RX, 0, 7.0);
        device->setGain(SOAPY_SDR_TX, 0, 89.0);

        rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, { 0 });
        txStream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, { 0 });

        device->activateStream(rxStream);
        device->activateStream(txStream);

        rx_mtu = device->getStreamMTU(rxStream);
        tx_mtu = device->getStreamMTU(txStream);

        rx_buffer.resize(2 * rx_mtu);
        tx_buffer.resize(2 * tx_mtu);
    }

    ~SDR()
    {
        if (device)
        {
            device->deactivateStream(rxStream);
            device->deactivateStream(txStream);
            device->closeStream(rxStream);
            device->closeStream(txStream);
            SoapySDR::Device::unmake(device);
        }
    }

    int receive()
    {
        void *buffs[] = { rx_buffer.data() };
        int flags = 0;

        int ret = device->readStream(rxStream, buffs, rx_mtu, flags, timeNs, TimeOut);
        return ret;
    }

    int send()
    {
        const void *buffs[] = { tx_buffer.data() };
        int flags = SOAPY_SDR_HAS_TIME;

        long long tx_time = timeNs + TxDelay;

        return device->writeStream(txStream, buffs, tx_mtu, flags, tx_time, TimeOut);
    }

    static std::vector<SoapySDR::Kwargs> findDevices()
    {
        return SoapySDR::Device::enumerate({ { "driver", "plutosdr" } });
    }

    void updateConfig(SharedData &sd)
    {
        auto &cfg = sd.SDR;

        if (cfg.dirty_mask & SDRField::RxGain)
            device->setGain(SOAPY_SDR_RX, 0, cfg.rx_gain);

        if (cfg.dirty_mask & SDRField::TxGain)
            device->setGain(SOAPY_SDR_TX, 0, cfg.tx_gain);

        if (cfg.dirty_mask & SDRField::RxFreq)
            device->setFrequency(SOAPY_SDR_RX, 0, cfg.rx_freq);

        if (cfg.dirty_mask & SDRField::TxFreq)
            device->setFrequency(SOAPY_SDR_TX, 0, cfg.tx_freq);

        if (cfg.dirty_mask & SDRField::RxBW)
            device->setBandwidth(SOAPY_SDR_RX, 0, cfg.rx_bw);

        if (cfg.dirty_mask & SDRField::TxBW)
            device->setBandwidth(SOAPY_SDR_TX, 0, cfg.tx_bw);

        if (cfg.dirty_mask & SDRField::RxSampleRate)
        {
            device->deactivateStream(rxStream, 0, 0);
            device->setSampleRate(SOAPY_SDR_RX, 0, cfg.rx_sample_rate);
            device->activateStream(rxStream, 0, 0);
        }

        if (cfg.dirty_mask & SDRField::TxSampleRate)
        {
            device->deactivateStream(txStream, 0, 0);
            device->setSampleRate(SOAPY_SDR_TX, 0, cfg.tx_sample_rate);
            device->activateStream(txStream, 0, 0);
        }

        cfg.dirty_mask = 0;
    }
};

void SDRStream(SharedData &sd, SDR &sdr);
void DSPThread(SharedData &sd);
bool LoadFileForTX(SharedData &sd);
bool SaveReceivedFile(SharedData &sd);