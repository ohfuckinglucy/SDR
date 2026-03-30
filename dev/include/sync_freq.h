#ifndef SYNC_FREQ_H
#define SYNC_FREQ_H
#include "common.h"

vector<complex<float>> cfo_est(const vector<complex<float>> &signal, SharedData &sd);

#endif