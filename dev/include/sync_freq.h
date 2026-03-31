#ifndef SYNC_FREQ_H
#define SYNC_FREQ_H
#include "common.h"

std::vector<std::complex<float>> cfo_est(const std::vector<std::complex<float>> &signal, SharedData &sd);

#endif