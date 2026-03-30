#ifndef OFDM_CORE_H
#define OFDM_CORE_H
#include "common.h"

bool is_guard(int k, SharedData &sd);

vector<complex<float>> insert_pilots(const vector<complex<float>>& symbols, SharedData& sd);
vector<complex<float>> ofdm_modulator(const vector<complex<float>>& freq_symbols, SharedData& sd);
vector<complex<float>> discard_cp(vector<complex<float>> signal, SharedData &sd);
void update_pilots(SharedData& sd);
vector<complex<float>> ofdm_equalize(const vector<complex<float>>& signal, SharedData &sd);

#endif