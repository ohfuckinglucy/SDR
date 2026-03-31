#pragma once
#include "common.h"

std::vector<std::complex<float>> modulator(std::vector<int16_t> bits, int len_bits, std::string type);
std::vector<int16_t> demodulator(const std::vector<std::complex<float>>& symbols, std::string type);

std::vector<std::complex<float>> generate_header(size_t size, SharedData &sd);
uint16_t decode_header(const std::vector<std::complex<float>> signal, SharedData &sd);

int bits_per_symbol(std::string type);