#ifndef MODULATION_H
#define MODULATION_H
#include "common.h"

vector<complex<float>> modulator(vector<int16_t> bits, int len_bits, string type);
vector<int16_t> demodulator(const vector<complex<float>>& symbols, string type);

vector<complex<float>> generate_header(size_t size, SharedData &sd);
uint16_t decode_header(const vector<complex<float>> signal, SharedData &sd);

int bits_per_symbol(string type);

#endif