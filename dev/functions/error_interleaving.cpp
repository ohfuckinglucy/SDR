#include "common.h"

std::vector<int16_t> calculateCRC16(const std::vector<uint8_t> &data)
{
    uint16_t crc = 0xFFFF;
    std::vector<int16_t> crc_bits;
    crc_bits.reserve(16);
    uint16_t polynomial = 0x1021;

    for (uint8_t byte : data)
    {
        crc ^= (uint16_t)byte << 8;

        for (int i = 0; i < 8; ++i)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ polynomial;
            else
                crc <<= 1;
        }
    }

    for (int i = 15; i >= 0; --i)
    {
        int16_t bit = (crc >> i) & 1;
        crc_bits.push_back(bit);
    }

    return crc_bits;
}

std::vector<int16_t> calculateCRC16_fromBits(const std::vector<int16_t> &bits)
{
    std::vector<uint8_t> bytes;
    bytes.reserve((bits.size() + 7) / 8);

    for (size_t i = 0; i < bits.size(); i += 8)
    {
        uint8_t byte = 0;
        for (int j = 0; j < 8 && (i + j) < bits.size(); ++j)
        {
            if (bits[i + j] != 0)
            {
                byte |= (1 << (7 - j));
            }
        }
        bytes.push_back(byte);
    }

    return calculateCRC16(bytes);
}

bool verifyCRC16(std::vector<int16_t> &received_bits)
{
    int crc_bits_count = 16;
    if (received_bits.size() < (size_t)crc_bits_count)
    {
        return false;
    }

    std::vector<int16_t> received_crc;
    received_crc.reserve(crc_bits_count);
    for (size_t i = received_bits.size() - crc_bits_count; i < received_bits.size(); ++i)
    {
        received_crc.push_back(received_bits[i]);
    }

    received_bits.resize(received_bits.size() - crc_bits_count);

    std::vector<int16_t> calculated_crc = calculateCRC16_fromBits(received_bits);

    if (received_crc.size() != calculated_crc.size())
    {
        return false;
    }

    for (size_t i = 0; i < received_crc.size(); ++i)
    {
        if (received_crc[i] != calculated_crc[i])
        {
            return false;
        }
    }

    return true;
}

std::vector<int16_t> hamming_encoder(const std::vector<int16_t> &bits)
{
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < bits.size(); i += 8) {
        uint8_t byte = 0;
        for (int j = 0; j < 8 && (i + j) < bits.size(); ++j)
            if (bits[i + j]) byte |= (1 << (7 - j));
        bytes.push_back(byte);
    }
    while (bytes.size() % 3 != 0)
        bytes.push_back(0);

    std::vector<int16_t> encoded_bits;
    const size_t block_count = bytes.size() / 3;
    encoded_bits.reserve(block_count * 30);

    for (size_t i = 0; i + 2 < bytes.size(); i += 3) {
        uint32_t data24 = (uint32_t(bytes[i]) << 16) | (uint32_t(bytes[i+1]) << 8) | bytes[i+2];
        uint32_t block = 0;
        uint8_t checksum = 0;
        int data_bit_pos = 23;

        for (int j = 1; j <= 30; ++j) {
            if ((j & (j - 1)) == 0) continue;
            if (data_bit_pos >= 0) {
                if ((data24 >> data_bit_pos) & 1) {
                    block    |= (1U << (j - 1));
                    checksum ^= j;
                }
                data_bit_pos--;
            }
        }
        for (int k = 0; k < 5; ++k)
            if ((checksum >> k) & 1)
                block |= (1U << ((1 << k) - 1));

        for (int b = 29; b >= 0; --b)
            encoded_bits.push_back((block >> b) & 1);
    }

    // Интерливинг: транспонирование матрицы block_count × 30 → 30 × block_count
    constexpr size_t B = 30;
    const size_t N = block_count;
    std::vector<int16_t> interleaved(N * B, 0);

    for (size_t g = 0; g < N * B; ++g) {
        size_t row = g / B;   // номер блока
        size_t col = g % B;   // бит внутри блока
        size_t dg  = col * N + row;
        interleaved[dg] = encoded_bits[g];
    }

    return interleaved;
}

std::vector<int16_t> hamming_decoder(std::vector<int16_t> &bits, SharedData &sd)
{
    constexpr size_t B = 30;
    if (bits.size() % B != 0) return {};
    const size_t N = bits.size() / B;

    // Деинтерливинг: транспонирование обратно 30 × N → N × 30
    std::vector<int16_t> deinterleaved(N * B, 0);

    for (size_t g = 0; g < N * B; ++g) {
        size_t row = g / B;
        size_t col = g % B;
        size_t sg  = col * N + row;
        deinterleaved[g] = bits[sg];
    }

    std::vector<uint8_t> decoded_bytes;
    decoded_bytes.reserve(N * 3);

    for (size_t i = 0; i < N; ++i) {
        uint32_t block = 0;
        for (int b = 0; b < 30; ++b)
            if (deinterleaved[i * 30 + b])
                block |= (1U << (29 - b));

        uint8_t syndrome = 0;
        for (int j = 1; j <= 30; ++j)
            if ((block >> (j - 1)) & 1)
                syndrome ^= j;

        sd.Ham_stats.blocks_processed++;
        if (syndrome != 0) {
            sd.Ham_stats.blocks_with_errors++;
            if (syndrome <= 30) {
                block ^= (1U << (syndrome - 1));
                sd.Ham_stats.bits_corrected++;
            } else {
                sd.Ham_stats.uncorrectable++;
            }
        }

        uint32_t data24 = 0;
        int data_bit_pos = 23;
        for (int j = 1; j <= 30; ++j) {
            if ((j & (j - 1)) == 0) continue;
            if ((block >> (j - 1)) & 1)
                data24 |= (1U << data_bit_pos);
            data_bit_pos--;
        }

        decoded_bytes.push_back((data24 >> 16) & 0xFF);
        decoded_bytes.push_back((data24 >>  8) & 0xFF);
        decoded_bytes.push_back( data24        & 0xFF);
    }

    std::vector<int16_t> out;
    out.reserve(decoded_bytes.size() * 8);
    for (uint8_t byte : decoded_bytes)
        for (int b = 7; b >= 0; --b)
            out.push_back((byte >> b) & 1);

    return out;
}