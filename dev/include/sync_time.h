#ifndef SYNC_TIME_H
#define SYNC_TIME_H
#include "common.h"

void sym_sync(SharedData& sd, const vector<complex<double>>& buf);

vector<complex<double>> generate_shmidt_preamble(SharedData& sd);
int shmidt_sync(const vector<complex<double>>& signal, SharedData& sd);
vector<int> ofdm_sym_sync(const vector<complex<double>>& signal, SharedData& sd);

#endif