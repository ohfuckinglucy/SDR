#ifndef SYNC_TIME_H
#define SYNC_TIME_H
#include "common.h"

vector<complex<double>> generate_zc_preamble(SharedData& sd);

int zc_sync(const vector<complex<double>>& signal, SharedData& sd);

#endif