#pragma once

#include "common.h"

std::vector<std::complex<float>> generate_zc_preamble(SharedData &sd);

int zadoff_sync(const std::vector<std::complex<float>> &signal, SharedData &sd);
std::vector<std::complex<float>> remove_pss(const SharedData &sd, const std::vector<std::complex<float>> &signal);