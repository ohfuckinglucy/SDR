#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h>
#include <iostream>
#include <map>
#include <ctime>
#include <vector>
#include <string.h>

using namespace std;
template<typename T>
void Show_Array(const char* title, T *array, int len);

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
    config.carrier_freq = 870e6;

    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_RX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, config.carrier_freq, NULL);
    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_TX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, config.carrier_freq, NULL);

    size_t channels = 0;
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, channels, 40.0);
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, channels, -7.0);

    size_t rx_channels[] = {0};
    size_t tx_channels[] = {0};
    size_t channel_count = 1;

    config.rxStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, rx_channels, channel_count, NULL);
    config.txStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, tx_channels, channel_count, NULL);

    SoapySDRDevice_activateStream(config.sdr, config.rxStream, 0, 0, 0);
    SoapySDRDevice_activateStream(config.sdr, config.txStream, 0, 0, 0);

    config.rx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.rxStream);
    config.tx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.txStream);

    config.tx_buff = (int16_t*)calloc(2 * config.tx_mtu, sizeof(int16_t));
    config.rx_buffer = (int16_t*)calloc(2 * config.rx_mtu, sizeof(int16_t));

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

    FILE *tx = fopen("tx.pcm", "wb");
    if (tx == NULL){
        perror("fopen: ");
    }

    FILE *rx = fopen("rx.pcm", "wb");
    if (rx == NULL){
        perror("fopen: ");
    }

    int n = 20;

    srand(time(0));

    int16_t *bits = (int16_t*)malloc(n * sizeof(int16_t));

    for (auto i = 0; i < n; i ++){
        bits[i] = (rand() % 2);
    }

    int len_symbols = n/2;
    complex<double> *symbols = (complex<double>*)malloc(len_symbols * sizeof(complex<double>)); // Массив Символов

    int L = 10;
    int len_symbols_ups = len_symbols*L;
    complex<double> *symbols_ups = (complex<double>*)malloc(len_symbols_ups * sizeof(complex<double>)); // Массив Символов после апсемплинга

    complex<double> impulse[L]; // Импульсная хар-ка
    for (size_t i = 0; i < L; i++){
        impulse[i] = 1;
    }

    Mapper(bits, n, symbols, len_symbols);
    UpSampler(symbols, len_symbols, symbols_ups, L);
    filter(symbols_ups, len_symbols_ups, impulse, L);
    Show_Array("bits", bits, n);
    Show_Array("symbols", symbols, len_symbols);

    int16_t *tx_samples = (int16_t*)malloc(2*config.tx_mtu*sizeof(int16_t));

    for (size_t i = 0; i < config.tx_mtu; i++) {
        tx_samples[2*i] = (int16_t)(real(symbols_ups[i])) * 1000 << 4;  // I
        tx_samples[2*i + 1] = (int16_t)(imag(symbols_ups[i])) * 1000 << 4; // Q
    }

    fwrite(tx_samples, sizeof(int16_t), len_symbols_ups, tx);
    fclose(tx);

    int flags;
    long long timeNs;
    long long last_time = 0;
    long timeoutUs = 400000;
    flags = SOAPY_SDR_HAS_TIME;

    void *rx_buffs[] = {config.rx_buffer};
    void *tx_buffs[] = {tx_samples};

    for (size_t i = 0; i < 4; i++)
    {
        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, timeoutUs);

        long long tx_time = timeNs + (4 * 1000 * 1000); // Schedule TX 4ms ahead
        if (i)
        {
            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, (const void * const*)tx_buffs, config.tx_mtu, &flags, tx_time, timeoutUs);

            if (st < 0)
                printf("TX Failed on buffer %zu: %i\n", i, st);
            printf("Buffer: %lu - Samples: %i, Flags: %i, Time: %lli, TimeDiff: %lli\n", i, sr, flags, timeNs, (timeNs - last_time) * (last_time > 0));
        }
        fwrite(rx_buffs[0], sizeof(int16_t), 2 * config.rx_mtu, rx);
        last_time = tx_time;
    }

    SoapySDRDevice_deactivateStream(config.sdr, config.rxStream, 0, 0);
    SoapySDRDevice_deactivateStream(config.sdr, config.txStream, 0, 0);

    SoapySDRDevice_closeStream(config.sdr, config.rxStream);
    SoapySDRDevice_closeStream(config.sdr, config.txStream);

    SoapySDRDevice_unmake(config.sdr);

    free(config.tx_buff);
    free(config.rx_buffer);
    free(symbols);
    free(symbols_ups);
    fclose(rx);

    return 0;
}