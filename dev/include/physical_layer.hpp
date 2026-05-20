#pragma once
#include "common.hpp"

// === Physical Layer ===
// TX: frequency-domain symbols → time-domain I/Q with ZC preamble, scaled to int16
std::vector<int16_t> phy_tx(const std::vector<std::complex<float>> &symbols, SharedData &sd);

// RX: time-domain I/Q → sync + CFO corrected time-domain signal
std::vector<std::complex<float>> phy_sync_cfo(const std::vector<std::complex<float>> &iq, SharedData &sd);

// RX: time-domain OFDM symbols → frequency-domain (CP removal + FFT)
std::vector<std::complex<float>> phy_ofdm_demod(const std::vector<std::complex<float>> &signal, SharedData &sd);

// RX: frequency-domain with pilots → equalized symbols
std::vector<std::complex<float>> phy_equalize(const std::vector<std::complex<float>> &freq_signal, SharedData &sd);

// RX: extract one OFDM symbol from time-domain, FFT + equalize (for header)
std::vector<std::complex<float>> phy_demod_symbol(const std::vector<std::complex<float>> &time_signal,
                                                   size_t symbol_idx, SharedData &sd);

// Spectrum calculation helper
std::vector<float> calc_spectrum(SharedData &sd, const std::vector<std::complex<float>> &buffer);
