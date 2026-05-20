#pragma once
#include "common.hpp"

const char *GetModulationName(SignalModulation mod);
uint16_t bits_per_sym(SignalModulation mod_type);
std::vector<std::complex<float>> get_reference_constellation(SignalModulation type);
float EVM_calculate(const std::vector<std::complex<float>> &received, const std::vector<std::complex<float>> &reference);
const char *GetSignalTypeName(SignalType type);
Header parse_header(const std::vector<std::complex<float>> &symbols, SharedData &sd);
std::vector<float> Spectrum_calulations(SharedData &sd, std::vector<std::complex<float>> raw_buffer);
std::vector<int16_t> GenerateSignal(SharedData &sd);