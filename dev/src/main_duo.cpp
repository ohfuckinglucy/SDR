#include "../include/header.h"
#include <iostream>
#include <ctime>
#include <unistd.h>

int main(int argc, char* argv[]){
    (void*)argc;
    FILE *tx = fopen("tx.pcm", "wb");
    if (tx == NULL){
        perror("fopen: ");
    }

    FILE *rx = fopen("rx.pcm", "wb");
    if (rx == NULL){
        perror("fopen: ");
    }

    int n = 50000;

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
    // Show_Array("bits", bits, n);
    // Show_Array("symbols", symbols, len_symbols);

    int16_t *tx_samples = (int16_t*)calloc(2 * len_symbols_ups, sizeof(int16_t));

    for (size_t i = 0; i < len_symbols_ups; i++) {
        tx_samples[2*i]     = (int16_t)(real(symbols_ups[i]) * 2000) << 4;
        tx_samples[2*i + 1] = (int16_t)(imag(symbols_ups[i]) * 2000) << 4;
    }

    fwrite(tx_samples, sizeof(int16_t), 2*len_symbols_ups, tx);
    fclose(tx);

    int pid = fork();

    if (pid != 0)
    {
        struct SDRConfig config2 = SDRinit(argv[1]);

        long long timeNs = 0;
        long long last_time = 0;
        long timeoutUs = 400000;

        while (1)
        {
            int flags = SOAPY_SDR_HAS_TIME;
            void *rx_buffs[] = { config2.rx_buffer };

            int sr = SoapySDRDevice_readStream(
                config2.sdr,
                config2.rxStream,
                rx_buffs,
                config2.rx_mtu,
                &flags,
                &timeNs,
                timeoutUs
            );

            printf("RX: sr=%d, time=%lld, diff=%lld\n",
                sr, timeNs, (last_time ? (timeNs - last_time) : 0));

            fwrite(rx_buffs[0], sizeof(int16_t), 2 * sr, rx);

            last_time = timeNs;
        }
    }
    else
    {
        struct SDRConfig config1 = SDRinit(argv[2]);

        long long timeNs = 0;
        long timeoutUs = 400000;

        void *tx_buffs[] = { tx_samples };

        sleep(1);

        timeNs = SoapySDRDevice_getHardwareTime(config1.sdr, NULL);

        while (1)
        {
            long long tx_time = timeNs + 4000000LL;

            int flags = SOAPY_SDR_HAS_TIME;

            int st = SoapySDRDevice_writeStream(
                config1.sdr,
                config1.txStream,
                (const void * const*)tx_buffs,
                1920,
                &flags,
                tx_time,
                timeoutUs
            );

            if (st < 0)
                printf("TX error: %d\n", st);

            timeNs = tx_time;
        }
        SoapySDRDevice_deactivateStream(config1.sdr, config1.txStream, 0, 0);
    }
    return 0;
}