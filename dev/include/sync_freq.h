#ifndef SYNC_FREQ_H
#define SYNC_FREQ_H
#include "common.h"

vector<complex<double>> cfo_est(const vector<complex<double>> &signal, SharedData &sd);

#endif