#pragma once
#include <cstdint>
#include <vector>

std::vector<int16_t> calculateCRC16(const std::vector<int16_t> &input);
bool verifyCRC16(std::vector<int16_t> &input);