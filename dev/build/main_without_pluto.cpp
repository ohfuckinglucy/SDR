#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h>
#include <iostream>
#include <map>
#include <vector>
#include <string.h>

using namespace std;


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

void filter(complex<double> *symbols_ups, int len_symbols_ups, int L, int16_t type) {
    if (type == 1){
        complex<double> impulse[L]; // Импульсная хар-ка
        for (size_t i = 0; i < L; i++){
            impulse[i] = 1;
        }

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
    } else {
        complex<double> impulse[L]; // Импульсная хар-ка
        double a = 0.9;
        for (size_t i = 0; i < L; i++){
            if (i == 0){
                impulse[i] = 1;
                continue;
            }
            impulse[i] = (sin((M_PI*i)/L)/((M_PI*i)/L)) * ((cos((M_PI*a*i)/L))/(1 - pow(((2*a*i)/L), 2)));
            cout << "Импульс " << impulse[i];
        }

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
}

int main(){
    FILE *tx = fopen("tx.pcm", "wb");
    if (tx == NULL){
        perror("fopen: ");
    }

    int n = 20;

    int16_t *bits = (int16_t*)malloc(n * sizeof(int16_t));

    for (auto i = 0; i < n; i ++){
        bits[i] = (rand() % 2);
    }

    int len_symbols = n/2;
    complex<double> *symbols = (complex<double>*)malloc(len_symbols * sizeof(complex<double>)); // Массив Символов

    int L = 10;
    int len_symbols_ups = len_symbols*L;
    complex<double> *symbols_ups = (complex<double>*)malloc(len_symbols_ups * sizeof(complex<double>)); // Массив Символов после апсемплинга

    Mapper(bits, n, symbols, len_symbols);
    UpSampler(symbols, len_symbols, symbols_ups, L);
    filter(symbols_ups, len_symbols_ups, L, 1);
    Show_Array("Символы", symbols_ups, len_symbols_ups);

    int16_t *tx_samples = (int16_t*)malloc(2*len_symbols_ups*sizeof(int16_t));

    for (size_t i = 0; i < len_symbols_ups; i++) {
        tx_samples[2*i] = (int16_t)(real(symbols_ups[i]));  // I
        tx_samples[2*i+1] = (int16_t)(imag(symbols_ups[i])); // Q
    }

    fwrite(tx_samples, sizeof(int16_t), len_symbols_ups, tx);
    fclose(tx);

    free(symbols);
    free(symbols_ups);

    return 0;
}