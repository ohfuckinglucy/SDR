#include "common.h"

std::vector<int16_t> calculateCRC16_fromBits(const std::vector<int16_t> &bits);
bool verifyCRC16(std::vector<int16_t> &received_bits);
std::vector<int16_t> hamming_encoder(const std::vector<int16_t> &bits);
std::vector<int16_t> hamming_decoder(std::vector<int16_t> &bits, SharedData &sd);