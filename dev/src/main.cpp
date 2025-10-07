#include <SoapySDR/Device.h>   // Инициализация устройства
#include <SoapySDR/Formats.h>  // Типы данных, используемых для записи сэмплов
#include <stdio.h>             //printf
#include <stdlib.h>            //free
#include <stdint.h>
#include <complex.h>
#include <math.h>

int main(){
    FILE *rxFile = fopen("rx_samples.pcm", "wb");
    if (!rxFile) {
        fprintf(stderr, "Failed to open file for writing\n");
        return 1;
    }


    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");        // Говорим какой Тип устройства 
    if (1) {
        SoapySDRKwargs_set(&args, "uri", "usb:");           // Способ обмена сэмплами (USB)
    } else {
        SoapySDRKwargs_set(&args, "uri", "ip:192.168.2.1"); // Или по IP-адресу
    }
    SoapySDRKwargs_set(&args, "direct", "1");               // 
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");   // Размер буфера + временные метки
    SoapySDRKwargs_set(&args, "loopback", "0");             // Используем антенны или нет
    SoapySDRDevice *sdr = SoapySDRDevice_make(&args);       // Инициализация
    SoapySDRKwargs_clear(&args);

    int sample_rate = 1e6;
    int carrier_freq = 800e6;
    // Параметры RX части
    SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, sample_rate);
    SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, carrier_freq , NULL);

    // Параметры TX части
    SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, sample_rate);
    SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, carrier_freq , NULL);

    // Инициализация количества каналов RX\\\\TX (в AdalmPluto он один, нулевой)
    size_t channels = 0;
    // Настройки усилителей на RX\\\\TX
    SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, channels, 10.0); // Чувствительность приемника
    SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, channels, -90.0);// Усиление передатчика

    size_t rx_channels[] = {0};
    size_t tx_channels[] = {0};
    size_t channel_count = 1; 

    // Формирование потоков для передачи и приема сэмплов
    SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, rx_channels, channel_count, NULL);
    SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, tx_channels, channel_count, NULL);

    SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0); //start streaming
    SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0); //start streaming

    // Получение MTU (Maximum Transmission Unit), в нашем случае - размер буферов. 
    size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);

    // Выделяем память под буферы RX и TX
    int16_t tx_buff[2*tx_mtu];
    int16_t rx_buffer[2*rx_mtu];

    //заполнение tx_buff значениями сэмплов первые 16 бит - I, вторые 16 бит - Q.
    for (size_t n = 0; n < tx_mtu; n++) {
        double t = (double)n / (double)sample_rate; 
        double phase = 2.0 * M_PI * (double)carrier_freq * t;
        int I = 100.0 * cos(1); 
        double Q = 100.0 * sin(1);

        // Запись в буфер: I и Q — int16_t
        tx_buff[2 * n]     = (int16_t)(I);
        tx_buff[2 * n + 1] = (Q);
    }

    //prepare fixed bytes in transmit buffer
    //we transmit a pattern of FFFF FFFF [TS_0]00 [TS_1]00 [TS_2]00 [TS_3]00 [TS_4]00 [TS_5]00 [TS_6]00 [TS_7]00 FFFF FFFF
    //that is a flag (FFFF FFFF) followed by the 64 bit timestamp, split into 8 bytes and packed into the lsb of each of the DAC words.
    //DAC samples are left aligned 12-bits, so each byte is left shifted into place
    for(size_t i = 0; i < 2; i++)
    {
        tx_buff[0 + i] = 0xffff;
        // 8 x timestamp words
        tx_buff[10 + i] = 0xffff;
    }

    // Количество итерация чтения из буфера
    size_t iteration_count = 99;

    long long last_time = 0;
    const long long timeoutUs = 10000000; // в мкр 

    // Начинается работа с получением и отправкой сэмплов
    for (size_t buffers_read = 0; buffers_read < iteration_count; buffers_read++)
    {
        void *rx_buffs[] = {rx_buffer};
        int flags;        // flags set by receive operation
        long long timeNs; //timestamp for receive buffer
        
        // считали буффер RX, записали его в rx_buffer
        int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu, &flags, &timeNs, timeoutUs);

        // Смотрим на количество считаных сэмплов, времени прихода и разницы во времени с чтением прошлого буфера
        printf("Buffer: %lu - Samples: %i, Flags: %i, Time: %lli, TimeDiff: %lli\n", buffers_read, sr, flags,
            timeNs, timeNs - last_time);
        last_time = timeNs;

        // Переменная для времени отправки сэмплов относительно текущего приема
        long long tx_time = timeNs + (4 * 1000 * 1000); // на 4 \[мс\] в будущее

        // Добавляем время, когда нужно передать блок tx_buff, через tx_time -наносекунд
        for(size_t i = 0; i < 6; i++)
        {
            uint8_t tx_time_byte = (tx_time >> (i * 4)) & 0xff;
            tx_buff[6 + i] = tx_time_byte << 4;
        }

        // Здесь отправляем наш tx_buff массив
        void *tx_buffs[] = {tx_buff};
        if( (buffers_read % 8 == 0) ){
            // printf("buffers_read: %d\\n", buffers_read);
            flags = SOAPY_SDR_HAS_TIME;
            int st = SoapySDRDevice_writeStream(sdr, txStream, (const void * const*)tx_buffs, tx_mtu, &flags, tx_time, timeoutUs);
            if ((size_t)st != tx_mtu)
            {
                printf("TX Failed: %i\\n", st);
            }
        }

        fwrite(rx_buffer, 2*rx_mtu*sizeof(int16_t), 1, rxFile);
    }

    //stop streaming
    SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
    SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);

    //shutdown the stream
    SoapySDRDevice_closeStream(sdr, rxStream);
    SoapySDRDevice_closeStream(sdr, txStream);

    //cleanup device handle
    SoapySDRDevice_unmake(sdr);

    fclose(rxFile);

    return 0;

}