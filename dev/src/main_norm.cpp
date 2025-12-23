#include "../include/header.h"
#include <iostream>
#include <ctime>

using namespace std;
template<typename T>
void Show_Array(const char* title, T *array, int len);


int main(int argc, char *argv[]){
    struct SDRConfig config = SDRinit(argv[1]);

    FILE *rx = fopen("rx.pcm", "wb");
    if (rx == NULL){
        perror("fopen: ");
    }

    int n = 200000;

    srand(time(0));

    int16_t barker_seq[13] = {1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1};
    int barker_len = 13;

    int16_t *bits = (int16_t*)malloc((barker_len*2 + n) * sizeof(int16_t));
    int bits_len = barker_len * 2 + n;
    
    for (auto i = 0; i < 2*barker_len; i ++){
        bits[i] = barker_seq[i%barker_len];
    }

    for (auto i = barker_len*2; i < bits_len; i ++){
        bits[i] = (rand() % 2);
    }

    int len_symbols = bits_len;
    complex<double> *symbols = (complex<double>*)malloc(len_symbols * sizeof(complex<double>)); // Массив Символов

    int L = 10;
    int len_symbols_ups = len_symbols*L;
    complex<double> *symbols_ups = (complex<double>*)malloc(len_symbols_ups * sizeof(complex<double>)); // Массив Символов после апсемплинга

    complex<double> impulse[L]; // Импульсная хар-ка
    for (size_t i = 0; i < L; i++){
        impulse[i] = 1;
    }

    // Mapper_QPSK(bits, bits_len, symbols, len_symbols);
    Mapper_BPSK(bits, bits_len, symbols, len_symbols);

    UpSampler(symbols, len_symbols, symbols_ups, L);
    filter(symbols_ups, len_symbols_ups, impulse, L);
    // Show_Array("bits", bits, n);
    // Show_Array("symbols", symbols, len_symbols);

    int16_t *tx_samples = (int16_t*)calloc(2*len_symbols_ups, sizeof(int16_t));

    for (size_t i = 0; i < len_symbols_ups; i++) {
        tx_samples[2*i] = (int16_t)((real(symbols_ups[i])) * 2000);  // I
        tx_samples[2*i + 1] = (int16_t)((imag(symbols_ups[i])) * 2000); // Q
    }

    int flags;
    long long timeNs;
    long long last_time = 0;
    long timeoutUs = 4000000;
    flags = SOAPY_SDR_HAS_TIME;

    void *rx_buffs[] = {config.rx_buffer};
    
    int count = (2*len_symbols_ups) / (1920*2);

    int i_max = max(1, count);

    cout << "imax " << i_max << endl;

    for (int i = 0; i < i_max; i++)
    {
        void *tx_buffs[] = {tx_samples + i * 1920*2};

        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, timeoutUs);

        long long tx_time = timeNs + (4 * 1000 * 1000); // Schedule TX 4ms ahead
        int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, (const void * const*)tx_buffs, config.tx_mtu, &flags, tx_time, timeoutUs);

        if (st < 0)
            printf("TX Failed on buffer %zu: %i\n", i, st);
        printf("Buffer: %lu - Samples: %i, Flags: %i, Time: %lli, TimeDiff: %lli\n", i, sr, flags, timeNs, (timeNs - last_time) * (last_time > 0));
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