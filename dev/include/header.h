#ifndef HEADER_H
#define HEADER_H

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

#include <GL/glew.h>
#include <SDL2/SDL.h>

#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_opengl3.h"

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

struct TED_gardner{
    bool sym_sync_enabled = false;
    float BnTs = 0.0000001;
    float Kp = 4.0;
    int ss_offset = 0;
    double ss_phase = 0.0;
    double ss_p1 = 0.0;
    double ss_p2 = 0.0;
    size_t ss_last_index = 0;
    double zeta = sqrt(2.0)/2.0;
    static constexpr int Nsp = 10;

    vector<int> TED_offsets;
};

struct Flags{
    atomic<bool> g_running{false};

    bool upsampling_enabled = false;
    bool costas_loop_enabled = false;
    bool cl_init = false;
    bool filter_enabled = false;
    bool tx_filter = false;
    bool loopback_flag = false;
    bool fft_flag = false;
    bool mf_init = false;
    bool fft_ready = false;
    bool ofdm_enabled = false;
    bool used_gardner = false;
    bool rx_gain_changed = false;
    bool tx_gain_changed = false;
    bool rx_freq_changed = false;
    bool tx_freq_changed = false;

    int modulation_index;
};

struct FormFilter{
    int rx_l = 10;
    int tx_l = 10;

    vector<complex<double>> mf_delay;
    complex<double> mf_sum = 0.0;
    size_t mf_index = 0;
};

struct CostasLoop{
    double cl_theta_hat = 0;
    float cl_Kp = 0.02;
    float cl_Ki = 0.0001;
    double cl_integrator = 0;
};

struct Fft_conf{
    vector<complex<double>> fft_buffer;
    vector<double> fft_magnitude;
    static constexpr size_t FFT_SIZE = 1024;

    fftw_plan fft_plan = nullptr;
    fftw_complex* fft_in = nullptr;
    fftw_complex* fft_out = nullptr;
};

struct device_finder{
    string selected_uri;

    vector<SoapySDRKwargs> pluto_devices;
    int selected_device_index = 0;
};

struct ofdm_conf{
    int8_t n_subcarriers = 64; // Кол-во поднесущих
    int8_t cp_len = 16; // Длина префикса
    int8_t n_ofdm_symbols = 2; // Кол-во ofdm символов

    double Tb = 1e-6; // Битовая длительность 
    double Ts = n_subcarriers * Tb; // Длительность ofdm символа без префикса
    double Tg = cp_len * Tb; // Длительность префикса
    double T_sym = Ts + Tg; // Длительность ofdm символа

    vector<int8_t>pilot_idx = {8, 16, 24, 40, 48, 56};
};

struct SharedData {
    mutex mtx;
    
    struct TED_gardner gardner;
    struct Flags flags;
    struct FormFilter FormFilter;
    struct CostasLoop costas;
    struct Fft_conf fft;
    struct device_finder dev_f;
    struct ofdm_conf ofdm;
    
    vector<int16_t> bits;
    vector<int16_t> tx_samples;
    vector<int16_t> last_tx_samples;
    vector<complex<double>> raw_buffer;
    vector<complex<double>> symbols;
    
    vector<complex<double>> scope_buffer;
    size_t scope_head = 0;
    bool scope_filled = false;
    static constexpr size_t SCOPE_SIZE = 1920*2;
    
    static constexpr size_t MAX_SAMPLES = 1920*2;
    static constexpr size_t MAX_SYMBOLS = 192*2;

    double tx_gain = -10;
    double rx_gain = 20;

    int freq = 734750000;
};

template<typename T>
void Show_Array(const char* title, T* array, int len);

SDRConfig SDRinit(char *usb, struct SharedData &sd);
vector<SoapySDRKwargs> find_pluto_devices();

vector<complex<double>> UpSampler(const vector<complex<double>>& signal, int L);
void filter(complex<double>* symbols_ups, int len_symbols_ups, int L);
complex<double> mf_filter(SharedData& sd, complex<double> x);
void sym_sync(SharedData& sd, const vector<complex<double>>& buf);
complex<double> costas_loop(SharedData& sd, complex<double> r);

int bits_per_symbol(string type);

vector<complex<double>> ofdm_modulator(const vector<complex<double>>& symbols, struct SharedData& sd);

void rx_back(SharedData& sd, SDRConfig &config);
void tx_back(SharedData& sd, SDRConfig &config);

extern const complex<double> i0;
extern const complex<double> i1;
extern const complex<double> i2;
extern const complex<double> i3;
extern const complex<double> i4;

extern const map<string, complex<double>> qpsk_map;

constexpr size_t N_BUFFERS = 1;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

#endif