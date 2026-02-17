#include "header.h"
#include "modulator.h"
#include <iostream>
#include <ctime>
#include <cstring>
#include <cmath>

#define _USE_MATH_DEFINES

template<typename T>
void Show_Array(const char* title, T* array, int len) {
    printf("%s: ", title);
    for (int i = 0; i < len; i++) {
        cout << array[i];
    }
    printf("\n");
}

template void Show_Array<int16_t>(const char*, int16_t*, int);
template void Show_Array<complex<double>>(const char*, complex<double>*, int);

SDRConfig SDRinit(char *usb, struct SharedData &sd) {
    SDRConfig config = {};
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", usb);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");

    cout << "Opening device: " << usb << endl;

    config.sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    config.sample_rate = 1e6;
    config.carrier_freq = 734750000;

    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_RX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, config.carrier_freq, nullptr);
    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_TX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, config.carrier_freq, nullptr);

    int channels = 0;
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, channels, 20);
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, channels, 40);

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

vector<SoapySDRKwargs> find_pluto_devices() {
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    
    size_t length;
    SoapySDRKwargs *devices = SoapySDRDevice_enumerate(&args, &length);
    
    std::vector<SoapySDRKwargs> result(devices, devices + length);
    
    SoapySDRKwargs_clear(&args);
    
    return result;
}