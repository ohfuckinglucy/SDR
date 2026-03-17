#include "sdr_hw.h"
#include <iostream>
#include <cstring>

SDRConfig SDRinit(char *usb, struct SharedData &sd) {
    SDRConfig config = {};
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", usb);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "1");

    cout << "Opening device: " << usb << endl;

    config.sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    config.sample_rate = 1.92e6;
    config.carrier_freq = 7.670000e+08;

    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_RX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, config.carrier_freq, nullptr);
    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_TX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, config.carrier_freq, nullptr);

    int channels = 0;
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, channels, 7);
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, channels, 89);

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

void reconfig_sdr(SharedData &sd, SDRConfig &config){
    if (sd.flags.rx_gain_changed){
        SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, 0, sd.rx_gain);
        sd.flags.rx_gain_changed = false;
    }

    if (sd.flags.tx_gain_changed){
        SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, 0, sd.tx_gain);
        sd.flags.tx_gain_changed = false;
    }

    if (sd.flags.rx_freq_changed){
        SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, sd.freq, nullptr);
        sd.flags.rx_freq_changed = false;
    }

    if (sd.flags.tx_freq_changed){
        SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, sd.freq, nullptr);
        sd.flags.tx_freq_changed = false;
    }

    if (sd.flags.rx_bw_changed){
        SoapySDRDevice_deactivateStream(config.sdr, config.rxStream, 0, 0);
        SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_RX, 0, sd.rx_bandwidth);
        sd.flags.rx_bw_changed = false;
        SoapySDRDevice_activateStream(config.sdr, config.rxStream, 0, 0, 0);
    }
    
    if (sd.flags.tx_bw_changed){
        SoapySDRDevice_deactivateStream(config.sdr, config.txStream, 0, 0);
        SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_TX, 0, sd.tx_bandwidth);
        sd.flags.tx_bw_changed = false;
        SoapySDRDevice_activateStream(config.sdr, config.txStream,0, 0, 0);
    }
}