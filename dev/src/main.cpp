#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h>

struct SDRConfig{
    SoapySDRDevice *sdr;
    SoapySDRStream *rxStream;
    SoapySDRStream *txStream;
    size_t rx_mtu;
    size_t tx_mtu;
    int sample_rate;
    int carrier_freq;
    int16_t *tx_buff;
    int16_t *rx_buffer;
};

struct SDRConfig SDRinit(){
    struct SDRConfig config = {};
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    if (1) {
        SoapySDRKwargs_set(&args, "uri", "usb:");
    } else {
        SoapySDRKwargs_set(&args, "uri", "ip:192.168.2.1");
    }
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");
    config.sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    config.sample_rate = 1e6;
    config.carrier_freq = 800e6;

    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_RX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, config.carrier_freq, NULL);
    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_TX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, config.carrier_freq, NULL);

    size_t channels = 0;
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, channels, 10.0);
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, channels, -90.0);

    size_t rx_channels[] = {0};
    size_t tx_channels[] = {0};
    size_t channel_count = 1;

    config.rxStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, rx_channels, channel_count, NULL);
    config.txStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, tx_channels, channel_count, NULL);

    SoapySDRDevice_activateStream(config.sdr, config.rxStream, 0, 0, 0);
    SoapySDRDevice_activateStream(config.sdr, config.txStream, 0, 0, 0);

    config.rx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.rxStream);
    config.tx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.txStream);

    config.tx_buff = malloc(2 * config.tx_mtu * sizeof(int16_t));
    config.rx_buffer = malloc(2 * config.rx_mtu * sizeof(int16_t));

    return config;
}

int main(){
    struct SDRConfig config = SDRinit();

    //заполнение tx_buff значениями сэмплов первые 16 бит - I, вторые 16 бит - Q.
    for (size_t n = 0; n < config.tx_mtu; n++) {
        double t = (double)n / (double)config.sample_rate; 
        double phase = 2.0 * M_PI * (double)config.carrier_freq * t;
        int I = 100.0 * cos(1); 
        double Q = 100.0 * sin(1);

        // Запись в буфер: I и Q — int16_t
        config.tx_buff[2 * n]     = (int16_t)(I);
        config.tx_buff[2 * n + 1] = (int16_t)(Q);
    }

    for(size_t i = 0; i < 2; i++)
    {
        config.tx_buff[0 + i] = 0xffff;
        config.tx_buff[10 + i] = 0xffff;
    }

    size_t iteration_count = 99;

    long long last_time = 0;
    const long long timeoutUs = 10000000; 

    for (size_t buffers_read = 0; buffers_read < iteration_count; buffers_read++)
    {
        void *rx_buffs[] = {config.rx_buffer};
        int flags;
        long long timeNs;
        
        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, timeoutUs);

        printf("Buffer: %lu - Samples: %i, Flags: %i, Time: %lli, TimeDiff: %lli\n", buffers_read, sr, flags,
            timeNs, timeNs - last_time);
        last_time = timeNs;

        long long tx_time = timeNs + (4 * 1000 * 1000);

        for(size_t i = 0; i < 6; i++)
        {
            uint8_t tx_time_byte = (tx_time >> (i * 4)) & 0xff;
            config.tx_buff[6 + i] = tx_time_byte << 4;
        }

        void *tx_buffs[] = {config.tx_buff};
        flags = SOAPY_SDR_HAS_TIME;
        int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, (const void * const*)tx_buffs, config.tx_mtu, &flags, tx_time, timeoutUs);
        if ((size_t)st != config.tx_mtu)
        {
            printf("TX Failed: %i\n", st);
        }
    }

    SoapySDRDevice_deactivateStream(config.sdr, config.rxStream, 0, 0);
    SoapySDRDevice_deactivateStream(config.sdr, config.txStream, 0, 0);

    SoapySDRDevice_closeStream(config.sdr, config.rxStream);
    SoapySDRDevice_closeStream(config.sdr, config.txStream);

    SoapySDRDevice_unmake(config.sdr);

    free(config.tx_buff);
    free(config.rx_buffer);

    return 0;
}