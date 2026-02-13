#ifndef HEADER_H
#define HEADER_H

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
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
#include <fftw3.h>

using namespace std;

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

struct SharedData {
    vector<int16_t> bits;
    mutex mtx;

    vector<int16_t> tx_samples;
    vector<complex<double>> raw_buffer;
    vector<complex<double>> symbols;
    static constexpr size_t MAX_SAMPLES = 30000;
    static constexpr size_t MAX_SYMBOLS = 70000;

    bool sym_sync_enabled = false;
    float BnTs = 0.001;
    int ss_offset = 0;
    double ss_phase = 0.0;
    double ss_p1 = 0.0;
    double ss_p2 = 0.0;
    size_t ss_last_index = 0;
    double zeta = sqrt(2.0)/2.0;
    static constexpr int Nsp = 10;
    
    bool filter_enabled = false;
    int mf_L = 10;
    int tx_l = 10;
    vector<complex<double>> mf_delay;
    complex<double> mf_sum = 0.0;
    size_t mf_index = 0;
    bool mf_init = false;

    bool costas_loop_enabled = false;
    bool cl_init = false;
    double cl_theta_hat = 0;
    float cl_Kp = 0.02;
    float cl_Ki = 0.0001;
    double cl_integrator = 0;

    vector<complex<double>> fft_buffer;
    vector<double> fft_magnitude;
    bool fft_ready = false;
    static constexpr size_t FFT_SIZE = 1024;

    fftw_plan fft_plan = nullptr;
    fftw_complex* fft_in = nullptr;
    fftw_complex* fft_out = nullptr;

    double tx_gain = -10;
    double rx_gain = 20;

    int freq = 734750000;

    string rx_uri;
    string tx_uri;

    bool g_running = true;
    string selected_uri;
    bool tx_filter = false;
    bool loopback_flag = false;
    bool fft_flag = false;

    int modulation_index;

    vector<SoapySDRKwargs> pluto_devices;
    int selected_device_index = 0;
};

template<typename T>
void Show_Array(const char* title, T* array, int len);

SDRConfig SDRinit(char *usb, struct SharedData &sd);

vector<complex<double>> UpSampler(complex<double>* symbols, int len_s, int L);

void filter(complex<double>* symbols_ups, int len_symbols_ups, int L);
complex<double> mf_filter(SharedData& sd, complex<double> x);
void sym_sync(SharedData& sd, const vector<complex<double>>& buf);
complex<double> costas_loop(SharedData& sd, complex<double> r);
vector<complex<double>> filter_ret(const vector<complex<double>>& symbols_ups, int L);

void Backend(SharedData& sd, SDRConfig &config);
void tx_back(SharedData& sd, SDRConfig &config);

vector<SoapySDRKwargs> find_pluto_devices();

extern const complex<double> i0;
extern const complex<double> i1;
extern const complex<double> i2;
extern const complex<double> i3;
extern const complex<double> i4;

extern const map<string, complex<double>> qpsk_map;

constexpr size_t N_BUFFERS = 100000;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

#endif