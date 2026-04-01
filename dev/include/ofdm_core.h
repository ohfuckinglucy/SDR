#pragma once

#include "common.h"

bool is_guard(int k, SharedData &sd);

std::vector<std::complex<float>> insert_pilots(const std::vector<std::complex<float>> &symbols, SharedData &sd);
std::vector<std::complex<float>> ofdm_modulator(const std::vector<std::complex<float>> &freq_symbols, SharedData &sd);
std::vector<std::complex<float>> discard_cp(std::vector<std::complex<float>> signal, SharedData &sd);
void update_pilots(SharedData &sd);
std::vector<std::complex<float>> ofdm_equalize(const std::vector<std::complex<float>> &signal, SharedData &sd);