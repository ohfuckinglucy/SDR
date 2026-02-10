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
#include <mutex>

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
    char* usb;
    char* type;

    vector<complex<double>> raw_buffer;
    vector<complex<double>> symbols;
    static constexpr size_t MAX_SAMPLES = 15000;
    static constexpr size_t MAX_SYMBOLS = 7000;

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
    std::vector<std::complex<double>> mf_delay;
    std::complex<double> mf_sum = 0.0;
    size_t mf_index = 0;
    bool mf_init = false;

    bool costas_loop_enabled = false;
    bool cl_init = false;
    double cl_theta_hat = 0;
    float cl_Kp = 0.02;
    float cl_Ki = 0.0001;
    double cl_integrator = 0;

    static constexpr size_t FFT_SIZE = 1024;
    std::vector<std::complex<double>> fft_buffer;
    std::vector<double> fft_magnitude;
    bool fft_ready = false;

    double tx_gain = -30;
    double rx_gain = 10;

    int freq = 734750000;

    string selected_uri = "usb:1.55.5";
};

template<typename T>
void Show_Array(const char* title, T* array, int len);

SDRConfig SDRinit(char *usb, struct SharedData &sd);

vector<complex<double>> UpSampler(complex<double>* symbols, int len_s, int L);

void filter(complex<double>* symbols_ups, int len_symbols_ups, int L);
complex<double> mf_filter(SharedData& sd, complex<double> x);
void sym_sync(SharedData& sd, const std::vector<std::complex<double>>& buf);
complex<double> costas_loop(SharedData& sd, complex<double> r);

extern const complex<double> i0;
extern const complex<double> i1;
extern const complex<double> i2;
extern const complex<double> i3;
extern const complex<double> i4;

extern const map<string, complex<double>> qpsk_map;

#endif