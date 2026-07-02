#include "fec.hpp"

std::vector<int16_t> calculateCRC16(const std::vector<int16_t> &input)
{
    auto bits = input;

    uint16_t poly = 0x8005;
    uint16_t reg = 0;
    std::vector<int16_t> CRC;
    CRC.reserve(16);

    bits.resize(bits.size() + 16, 0);

    for (size_t i = 0; i < bits.size(); ++i)
    {
        bool eq = false;

        if (reg & 0x8000)
            eq = true;

        reg <<= 1;
        reg |= bits[i];

        if (eq)
            reg ^= poly;
    }

    for (int16_t i = 15; i >= 0; --i)
        CRC.push_back((reg >> i) & 1);

    return CRC;
}

bool verifyCRC16(std::vector<int16_t> &input)
{
    if (input.size() < 16)
        return false;

    std::vector<int16_t> received_crc;
    received_crc.insert(received_crc.begin(), input.end() - 16, input.end());
    input.erase(input.end() - 16, input.end());

    auto calculated_crc = calculateCRC16(input);

    if (received_crc == calculated_crc)
        return true;

    return false;
}
