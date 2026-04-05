#include "modulator.h"

#include "logger.hpp"
#include "ofdm_core.h"
#include "sync_freq.h"

std::vector<std::complex<float>> modulator(std::vector<int16_t> bits, int len_bits, std::string type) {
    float I, Q;

    if (type == "QAM::2") {
        std::vector<std::complex<float>> symbols(len_bits);
        for (int i = 0; i < len_bits; ++i) {
            I = 1 - 2 * bits[i];
            Q = 1 - 2 * bits[i];

            symbols[i] = std::complex<float>(I, Q) / sqrtf(2);
        }
        return symbols;
    } else if (type == "QAM::4") {
        if (len_bits % 2 != 0) {
            perror("L_Bits % 2 != 0");
            exit(1);
        }
        std::vector<std::complex<float>> symbols(len_bits / 2);
        for (int i = 0; i < len_bits / 2; ++i) {
            I = 1 - 2 * bits[2 * i];
            Q = 1 - 2 * bits[2 * i + 1];

            symbols[i] = std::complex<float>(I, Q) / sqrtf(2);
        }
        return symbols;
    } else if (type == "QAM::16") {
        if (len_bits % 4 != 0) {
            perror("L_Bits % 4 != 0");
            exit(1);
        }
        std::vector<std::complex<float>> symbols(len_bits / 4);
        for (int i = 0; i < len_bits / 4; ++i) {
            I = (1.0 - 2.0 * bits[4 * i]) * (2.0 - (1.0 - 2.0 * bits[4 * i + 2]));
            Q = (1.0 - 2.0 * bits[4 * i + 1]) * (2.0 - (1.0 - 2.0 * bits[4 * i + 3]));

            symbols[i] = std::complex<float>(I, Q) / sqrtf(10);
        }
        return symbols;
    } else if (type == "QAM::64") {
        if (len_bits % 6 != 0) {
            logs::dsp.warn("[MODULATOR] bits not % = 0");
            exit(1);
        }
        std::vector<std::complex<float>> symbols(len_bits / 6);
        for (int i = 0; i < len_bits / 6; ++i) {
            I = (1.0 - 2.0 * bits[6 * i]) * (4 - (1.0 - 2.0 * bits[6 * i + 2]) * (2 - (1.0 - 2.0 * bits[6 * i + 4])));
            Q = (1.0 - 2.0 * bits[6 * i + 1]) *
                (4 - (1.0 - 2.0 * bits[6 * i + 3]) * (2 - (1.0 - 2.0 * bits[6 * i + 5])));

            symbols[i] = std::complex<float>(I, Q) / sqrtf(42.0);
        }
        return symbols;
    } else {
        perror("unluck");
        exit(1);
    }
}

std::vector<int16_t> demodulator(const std::vector<std::complex<float>> &symbols, std::string type) {
    std::vector<int16_t> bits;

    if (type == "QAM::2") {
        bits.resize(symbols.size());
        for (size_t i = 0; i < symbols.size(); ++i) {
            float val = symbols[i].real();
            bits[i] = (val > 0) ? 0 : 1;
        }
    } else if (type == "QAM::4") {
        if (symbols.empty())
            return bits;

        bits.resize(symbols.size() * 2);
        for (size_t i = 0; i < symbols.size(); ++i) {
            float I = symbols[i].real();
            float Q = symbols[i].imag();

            bits[2 * i] = (I > 0) ? 0 : 1;
            bits[2 * i + 1] = (Q > 0) ? 0 : 1;
        }
    } else if (type == "QAM::16") {
        if (symbols.empty())
            return bits;

        bits.resize(symbols.size() * 4);
        float threshold = 2.0f / sqrtf(10.0f);

        for (size_t i = 0; i < symbols.size(); ++i) {
            float I = symbols[i].real();
            float Q = symbols[i].imag();

            bits[4 * i] = (I > 0) ? 0 : 1;
            bits[4 * i + 2] = (fabsf(I) > threshold) ? 1 : 0;
            bits[4 * i + 1] = (Q > 0) ? 0 : 1;
            bits[4 * i + 3] = (fabsf(Q) > threshold) ? 1 : 0;
        }
    } else if (type == "QAM::64") {
        if (symbols.empty())
            return bits;

        bits.resize(symbols.size() * 6);

        const float norm = sqrtf(42.0f);

        for (size_t i = 0; i < symbols.size(); ++i) {
            float I = symbols[i].real() * norm;
            float Q = symbols[i].imag() * norm;

            bits[6 * i + 0] = (I > 0.0f) ? 0 : 1;
            bits[6 * i + 1] = (Q > 0.0f) ? 0 : 1;

            float absI = fabsf(I);
            float absQ = fabsf(Q);

            bits[6 * i + 2] = (absI > 4.0f) ? 1 : 0;
            bits[6 * i + 3] = (absQ > 4.0f) ? 1 : 0;

            float sign_b2_I = (bits[6 * i + 2] == 0) ? 1.0f : -1.0f;
            float sign_b2_Q = (bits[6 * i + 3] == 0) ? 1.0f : -1.0f;

            float inner_I = (4.0f - absI) / sign_b2_I;
            float inner_Q = (4.0f - absQ) / sign_b2_Q;

            bits[6 * i + 4] = (inner_I < 2.0f) ? 0 : 1;
            bits[6 * i + 5] = (inner_Q < 2.0f) ? 0 : 1;
        }
    } else {
        return {};
    }

    return bits;
}

std::vector<std::complex<float>> generate_header(size_t size, SharedData &sd) {
    uint16_t num = size;
    std::vector<int16_t> binary;
    for (int i = 15; i >= 0; --i) {
        auto bit = (num >> i) & 1;
        binary.push_back(static_cast<int16_t>(bit));
    }

    std::vector<std::complex<float>> preamble_symbols = modulator(binary, binary.size(), "QAM::2");
    std::vector<std::complex<float>> freq_blocks = insert_pilots(preamble_symbols, sd);
    std::vector<std::complex<float>> ofdm_header = ofdm_modulator(freq_blocks, sd);

    return ofdm_header;
}

uint16_t decode_header(const std::vector<std::complex<float>> signal, SharedData &sd) {
    std::vector<std::complex<float>> header_frame;

    size_t N = sd.ofdm.n_subcarriers;
    size_t CP = sd.ofdm.cp_len;

    header_frame.insert(header_frame.begin(), signal.begin(), signal.begin() + N + CP);
    header_frame = cfo_est(header_frame, sd);
    header_frame = discard_cp(header_frame, sd);
    header_frame = ofdm_equalize(header_frame, sd);

    std::vector<int16_t> bits = demodulator(header_frame, "QAM::2");

    uint16_t packet_len = 0;
    for (size_t i = 0; i < 16 && i < bits.size(); ++i) {
        packet_len = (packet_len << 1) | (bits[i] & 1);
    }

    return packet_len;
}

int bits_per_symbol(std::string type) {
    if (type == "QAM::2") {
        return 1;
    } else if (type == "QAM::4") {
        return 2;
    } else if (type == "QAM::16") {
        return 4;
    } else if (type == "QAM::64") {
        return 6;
    } else {
        return 1;
    }
}