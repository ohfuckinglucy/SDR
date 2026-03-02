#ifndef MODULATION_H
#define MODULATION_H
#include "common.h"

vector<complex<double>> UpSampler(const vector<complex<double>>& symbols, int L);
void filter(complex<double>* symbols_ups, int len_symbols_ups, int L);
vector<complex<double>> modulator(vector<int16_t> bits, int len_bits, string type);
int bits_per_symbol(string type);

#endif