#ifndef OFDM_CORE_H
#define OFDM_CORE_H
#include "common.h"

bool is_guard(int k, SharedData &sd);

vector<complex<double>> insert_pilots(const vector<complex<double>>& symbols, SharedData& sd);
vector<complex<double>> ofdm_modulator(const vector<complex<double>>& freq_symbols, SharedData& sd);
vector<complex<double>> discard_cp(vector<complex<double>> signal, SharedData &sd);
void update_pilots(SharedData& sd);
vector<complex<double>> ofdm_equalize(const vector<complex<double>>& signal, SharedData &sd);

#endif