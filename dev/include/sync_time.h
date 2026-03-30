#ifndef SYNC_TIME_H
#define SYNC_TIME_H
#include "common.h"

vector<complex<float>> generate_zc_preamble(SharedData& sd);

int zadoff_sync(const std::vector<std::complex<float>> &signal, SharedData &sd);

#endif