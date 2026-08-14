#pragma once
#include "common.hpp"

constexpr int HEADER_BITS = 38; // 8 magic + 16 num_samples + 4 modulation + 8 flags + 2 sig_type

bool is_guard(int k, int N);
int usable_subcarriers(int N, size_t num_pilots);
void update_pilots(SharedData &sd);
void ensure_fft_plans(SharedData &sd);

std::vector<std::complex<float>> generate_zc_preamble(SharedData &sd);
std::vector<std::complex<float>> insert_pilots(const std::vector<std::complex<float>> &symbols, SharedData &sd);
std::vector<std::complex<float>> ofdm_modulator(const std::vector<std::complex<float>> &freq_symbols, SharedData &sd);

std::vector<std::complex<float>> generate_zc_preamble(SharedData &sd);
int zadoff_sync(const std::vector<std::complex<float>> &signal, SharedData &sd);
std::vector<std::complex<float>> FFT_ofdm(const std::vector<std::complex<float>> &signal, SharedData &sd);
std::vector<std::complex<float>> cfo_est(const std::vector<std::complex<float>> &signal, SharedData &sd);
std::vector<std::complex<float>> ofdm_equalize(const std::vector<std::complex<float>> &signal, SharedData &sd);