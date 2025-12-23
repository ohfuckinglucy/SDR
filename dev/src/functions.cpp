#include "../include/header.h"
#include <iostream>
#include <ctime>
#include <cstring>

const complex<double> i0(0, 0);
const complex<double> i1(1, 1);
const complex<double> i2(-1, 1);
const complex<double> i3(-1, -1);
const complex<double> i4(1, -1);

const map<string, complex<double>> qpsk_map = {
    {"00", i1},
    {"01", i2},
    {"11", i3},
    {"10", i4}
};

template<typename T>
void Show_Array(const char* title, T* array, int len) {
    printf("%s: ", title);
    for (int i = 0; i < len; i++) {
        std::cout << array[i];
    }
    printf("\n");
}

template void Show_Array<int16_t>(const char*, int16_t*, int);
template void Show_Array<complex<double>>(const char*, complex<double>*, int);

SDRConfig SDRinit(char *usb) {
    SDRConfig config = {};
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", usb);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");

    config.sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    config.sample_rate = 1e6;
    config.carrier_freq = 770e6;

    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_RX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, config.carrier_freq, nullptr);
    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_TX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, config.carrier_freq, nullptr);

    int channels = 0;
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, channels, 0.0);
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, channels, -30.0);

    size_t rx_channels[] = {0};
    size_t tx_channels[] = {0};
    size_t channel_count = 1;

    config.rxStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, rx_channels, channel_count, nullptr);
    config.txStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, tx_channels, channel_count, nullptr);

    SoapySDRDevice_activateStream(config.sdr, config.rxStream, 0, 0, 0);
    SoapySDRDevice_activateStream(config.sdr, config.txStream, 0, 0, 0);

    config.rx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.rxStream);
    config.tx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.txStream);

    config.tx_buff = (int16_t*)calloc(2 * config.tx_mtu, sizeof(int16_t));
    config.rx_buffer = (int16_t*)calloc(2 * config.rx_mtu, sizeof(int16_t));

    return config;
}

void Mapper_QPSK(int16_t* bits, int len_b, complex<double>* symbols, int len_s) {
    for (int i = 0; i < len_b; i += 2) {
        string pair_bits = std::to_string(bits[i]) + std::to_string(bits[i + 1]);
        symbols[i / 2] = qpsk_map.at(pair_bits);
    }
}

void Mapper_BPSK(int16_t* bits, int len_b, complex<double>* symbols, int len_s) {
    for (int i = 0; i < len_b; ++i) {
        double real_part = (bits[i] == 1) ? 1 : -1;
        symbols[i] = complex<double>(real_part, 0.0);
    }
}

void UpSampler(complex<double>* symbols, int len_s, complex<double>* symbols_ups, int L) {
    for (int i = 0; i < len_s * L; i++) {
        symbols_ups[i] = i0;
    }
    for (int i = 0; i < len_s; i++) {
        symbols_ups[i * L] = symbols[i];
    }
}

void filter(complex<double>* symbols_ups, int len_symbols_ups, complex<double>* impulse, int L) {
    complex<double>* sum = (complex<double>*)malloc(len_symbols_ups * sizeof(complex<double>));
    for (int i = 0; i < len_symbols_ups; i++) {
        sum[i] = complex<double>(0, 0);
        for (int j = 0; j < L && (i - j) >= 0; j++) {
            sum[i] += impulse[j] * symbols_ups[i - j];
        }
    }
    for (int i = 0; i < len_symbols_ups; i++) {
        symbols_ups[i] = sum[i];
    }
    free(sum);
}