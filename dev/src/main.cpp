#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h>
#include <iostream>
#include <map>
#include <string.h>

using namespace std;

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

    config.tx_buff = (int16_t*)malloc(2 * config.tx_mtu * sizeof(int16_t));
    config.rx_buffer = (int16_t*)malloc(2 * config.rx_mtu * sizeof(int16_t));

    return config;
}

// Объявление комплесных чисел
const complex<double> i0 (0, 0);
const complex<double> i1 (1, 1);
const complex<double> i2 (-1, 1);
const complex<double> i3 (-1, -1);
const complex<double> i4 (1, -1);

// Пары: Бит: Символ
static map<string, complex<double>> qpsk_map = {
    {"00", i1},
    {"01", i2},
    {"11", i3},
    {"10", i4}
};

template<typename T>

// Функция для вывода массива
void Show_Array(const char* title, T *array, int len){
    printf("%s: ", title);
    for (size_t i = 0; i < len; i++){
        cout << array[i];
    }
    printf("\n");
}

// Функция для преобразования битов в символы
void Mapper(int16_t *bits, int len_b, complex<double> *symbols, int len_s){
    string pair_bits;
    for (size_t i = 0; i < len_b; i += 2){
        pair_bits = to_string(bits[i]) + to_string(bits[i+1]);
        symbols[i/2] = qpsk_map[pair_bits];
    }
}

void UpSampler(complex<double> *symbols, int len_s, complex<double> *symbols_ups, int L){
    for (size_t i = 0; i < len_s*L; i++){
        symbols_ups[i] = i0;
    }
    for (size_t i = 0; i < len_s; i ++){
        symbols_ups[i*L] = symbols[i];
    }
}

void filter(complex<double> *symbols_ups, int len_symbols_ups, complex<double> *impulse, int L) {
    complex<double> *sum = (complex<double>*)malloc(len_symbols_ups * sizeof(complex<double>));
    for (size_t i = 0; i < len_symbols_ups; i++) {
        sum[i] = 0;
        for (size_t j = 0; j < L && (int)(i-j) >= 0; j++) {
            sum[i] += impulse[j] * symbols_ups[i-j];
        }
    }
    for (size_t i = 0; i < len_symbols_ups; i++) {
        symbols_ups[i] = sum[i];
    }
    free(sum);
}

int main(){
    struct SDRConfig config = SDRinit();

    FILE *file = fopen("symbols.bin", "wb");
    if (file == NULL){
        perror("fopen: ");
    }

    srand(time(0));

    int len_bits = 100;
    int16_t *bits = (int16_t*)malloc(len_bits * sizeof(int16_t)); // Массив бит
    for (size_t i = 0; i < len_bits; i ++){
        bits[i] = rand() % 2;
        cout << bits[i];
    }

    int len_symbols = len_bits/2;
    complex<double> *symbols = (complex<double>*)malloc(len_symbols * sizeof(complex<double>)); // Массив Символов

    int L = 10;
    int len_symbols_ups = len_symbols*L;
    complex<double> *symbols_ups = (complex<double>*)malloc(len_symbols_ups * sizeof(complex<double>)); // Массив Символов после апсемплинга

    complex<double> impulse[L]; // Импульсная хар-ка
    for (size_t i = 0; i < L; i++){
        impulse[i] = 1;
    }

    Mapper(bits, len_bits, symbols, len_symbols);
    UpSampler(symbols, len_symbols, symbols_ups, L);
    filter(symbols_ups, len_symbols_ups, impulse, L);
    fwrite(symbols_ups, sizeof(complex<double>), len_symbols_ups, file);

    int16_t *tx_samples = (int16_t*)malloc(2 * len_symbols_ups * sizeof(int16_t));

    for (size_t i = 0; i < len_symbols_ups; i++) {
        tx_samples[2*i] = static_cast<int16_t>(real(symbols_ups[i]));  // I
        tx_samples[2*i + 1] = static_cast<int16_t>(imag(symbols_ups[i])); // Q
    }
    
    // Копируем I/Q пары в начало tx_buff
    memcpy(config.tx_buff, tx_samples, 2 * len_symbols_ups * sizeof(int16_t));

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
    free(bits);
    free(symbols);
    free(symbols_ups);
    fclose(file);

    return 0;
}