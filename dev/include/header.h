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

template<typename T>
void Show_Array(const char* title, T* array, int len);

SDRConfig SDRinit(char *usb);

vector<complex<double>> UpSampler(complex<double>* symbols, int len_s, int L);

void filter(complex<double>* symbols_ups, int len_symbols_ups, int L);

extern const complex<double> i0;
extern const complex<double> i1;
extern const complex<double> i2;
extern const complex<double> i3;
extern const complex<double> i4;

extern const map<string, complex<double>> qpsk_map;

#endif