#pragma once
#include "common.hpp"

std::vector<std::complex<float>> modulator(const std::vector<int16_t> &input, SignalModulation type);
std::vector<int16_t> demodulator(const std::vector<std::complex<float>> &symbols, SignalModulation mod_type);
