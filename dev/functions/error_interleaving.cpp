#include "common.h"

vector<int16_t> calculateCRC16(const vector<uint8_t> &data)
{
    uint16_t crc = 0xFFFF;
    vector<int16_t> crc_bits;
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

vector<int16_t> calculateCRC16_fromBits(const vector<int16_t> &bits)
{
    vector<uint8_t> bytes;
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

bool verifyCRC16(vector<int16_t> &received_bits) {
    int crc_bits_count = 16;
    if (received_bits.size() < crc_bits_count) {
        return false;
    }

    vector<int16_t> received_crc;
    received_crc.reserve(crc_bits_count);
    for (size_t i = received_bits.size() - crc_bits_count; i < received_bits.size(); ++i) {
        received_crc.push_back(received_bits[i]);
    }

    received_bits.resize(received_bits.size() - crc_bits_count);

    vector<int16_t> calculated_crc = calculateCRC16_fromBits(received_bits);

    if (received_crc.size() != calculated_crc.size()) {
        return false;
    }

    for (size_t i = 0; i < received_crc.size(); ++i) {
        if (received_crc[i] != calculated_crc[i]) {
            return false;
        }
    }

    return true;
}

vector<uint32_t> interleaving(vector<uint32_t> &hamming_encoded){
    if (hamming_encoded.size() < 1){
        cerr << "[ERROR] Недостаточный размер!" << endl;
        return {};
    }

    int N = hamming_encoded.size();
    int M = 32;

    vector<uint32_t> interleaving_block(N, 0);

    for (int col = 0; col < M; ++col){
        for (int row = 0; row < N; ++row){
            int bit = (hamming_encoded[row] >> (31 - col)) & 1U;

            int i = (col * N + row) / M;
            int bit_pos = 31 - ((col * N + row) % M);

            if (i < N) {
                interleaving_block[i] |= (bit << bit_pos);
            }
        }
    }

    return interleaving_block;
}

vector<uint32_t> deinterleaving(vector<uint32_t> interleaving_block){
    int N = interleaving_block.size();
    int M = 32;

    vector<uint32_t> deinterleaving_block(N, 0);

    for (int col = 0; col < M; ++col) {
        for (int row = 0; row < N; ++row) {
            int in_i = (col * N + row) / M;
            int in_bit = 31 - ((col * N + row) % M);

            int bit = (interleaving_block[in_i] >> in_bit) & 1U;

            deinterleaving_block[row] |= (bit << (31 - col));
        }
    }

    return deinterleaving_block;
}

vector<int16_t> hamming_encoder_from_Bits(vector<int16_t> &bits){
    vector<uint8_t> bytes;
    bytes.reserve((bits.size() + 7) / 8);
    
    for (size_t i = 0; i < bits.size(); i += 8)
    {
        uint8_t byte = 0;
        for (int j = 0; j < 8 && (i + j) < bits.size(); ++j)
        {
            if (bits[i + j] != 0)
            {
                byte |= (1U << (7 - j));
            }
        }
        bytes.push_back(byte);
    }

    vector<uint32_t> out = hamming_encoder(bytes);
    out = interleaving(out);

    vector<int16_t> out_bits;
    out_bits.reserve(out.size() * 32);

    for (auto &byte : out){
        for (int i = 31; i >= 0; --i){
            out_bits.push_back((byte >> i) & 1U);
        }
    }


    return out_bits;
}

vector<int16_t> hamming_decoder_from_Bits(vector<int16_t> &bits){
    vector<uint32_t> bytes;
    bytes.reserve((bits.size() + 31) / 32);
    
    for (size_t i = 0; i < bits.size(); i += 32)
    {
        uint32_t byte = 0;
        for (int j = 0; j < 32 && (i + j) < bits.size(); ++j)
        {
            if (bits[i + j] != 0)
            {
                byte |= (1 << (32 - j));
            }
        }
        bytes.push_back(byte);
    }
    bytes = deinterleaving(bytes);
    vector<uint8_t> out = hamming_decoder(bytes);

    vector<int16_t> out_bits;
    out_bits.reserve(out.size() * 8);

    for (auto &byte : out){
        for (int i = 7; i >= 0; --i){
            out_bits.push_back((byte >> i) & 1U);
        }
    }

    return out_bits;
}

vector<uint32_t> hamming_encoder(vector<uint8_t> &bytes){
    if (bytes.size() < 1){
        cerr << "[ERROR] Недостаточно байт!" << endl;
        return {};
    }

    while (bytes.size() % 3 != 0) {
        bytes.push_back(0);
    }

    vector<uint32_t> encoded_bytes;
    encoded_bytes.reserve(bytes.size() / 3);
    
    for (size_t i = 0; i + 2 < bytes.size(); i += 3){
        uint32_t data24 = (static_cast<uint32_t>(bytes[i]) << 16) | (static_cast<uint32_t>(bytes[i+1])) << 8 | bytes[i+2];

        uint32_t block = 0;
        uint8_t checksum = 0;
        int data_bit_pos = 23;

        for (int j = 1; j <= 30; ++j){
            if ((j & (j - 1)) == 0){
                continue;
            }
            if (data_bit_pos >= 0){
                if ((data24 >> data_bit_pos) & 1){
                    block |= (1 << (j - 1));
                    checksum ^= j;
                }
                data_bit_pos--;
            }
        }

        for (int k = 0; k < 5; ++k) {
            if ((checksum >> k) & 1) {
                block |= (1 << ((1 << k) - 1)); 
            }
        }

        encoded_bytes.push_back(block);
    }

    return encoded_bytes;
}

vector<uint8_t> hamming_decoder(vector<uint32_t> &encoded_bytes){
    if (encoded_bytes.size() < 1){
        cerr << "[ERROR] Недостаточно байт!" << endl;
        return {};
    }

    vector<uint8_t> decoded_bytes;
    decoded_bytes.reserve(encoded_bytes.size() * 3);

    for (size_t i = 0; i < encoded_bytes.size(); ++i){
        uint8_t syndrome = 0;
        uint32_t block = 0;

        int data_bit_pos = 23;

        for (int j = 1; j <= 30; ++j){
            if ((encoded_bytes[i] >> (j - 1)) & 1){
                syndrome ^= j;
            }
        }

        if (syndrome == 0){
            cerr << "[DEBUG] OK" << endl;
        } else{
            encoded_bytes[i] ^= (1U << (syndrome - 1));
        }

        for (int j = 1; j <= 30; ++j){
            if ((j & (j - 1)) == 0) continue;

            if (data_bit_pos < 0) break;

            if ((encoded_bytes[i] >> (j - 1)) & 1){
                block |= (1U << data_bit_pos);
            }
            data_bit_pos--;
        }


        decoded_bytes.push_back((block >> 16) & 0xFF);
        decoded_bytes.push_back((block >> 8) & 0xFF);
        decoded_bytes.push_back(block & 0xFF); 
    }

    return decoded_bytes;
}