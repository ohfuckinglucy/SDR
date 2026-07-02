#include "common.hpp"
#include "modulation.hpp"

std::vector<std::complex<float>> modulator(const std::vector<int16_t> &input, SignalModulation type)
{
    float I, Q;
    std::vector<std::complex<float>> symbols;
    std::vector<int16_t> bits = input;

    switch (type)
    {
    case SignalModulation::BPSK: {
        symbols.resize(bits.size());

        for (size_t i = 0; i < bits.size(); ++i)
        {
            I = 1 - 2 * bits[i];
            Q = 1 - 2 * bits[i];

            symbols[i] = std::complex<float>(I, Q) / sqrtf(2);
        }

        break;
    }

    case SignalModulation::QPSK: {
        while (bits.size() % 2 != 0)
            bits.push_back(0);

        symbols.resize(bits.size() / 2);

        for (size_t i = 0; i < bits.size() / 2; ++i)
        {
            I = 1 - 2 * bits[2 * i];
            Q = 1 - 2 * bits[2 * i + 1];

            symbols[i] = std::complex<float>(I, Q) / sqrtf(2);
        }

        break;
    }

    case SignalModulation::QAM16: {
        while (bits.size() % 4 != 0)
            bits.push_back(0);

        symbols.resize(bits.size() / 4);

        for (size_t i = 0; i < bits.size() / 4; ++i)
        {
            I = (1.0 - 2.0 * bits[4 * i]) * (2.0 - (1.0 - 2.0 * bits[4 * i + 2]));
            Q = (1.0 - 2.0 * bits[4 * i + 1]) * (2.0 - (1.0 - 2.0 * bits[4 * i + 3]));

            symbols[i] = std::complex<float>(I, Q) / sqrtf(10);
        }

        break;
    }

    case SignalModulation::QAM64: {
        while (bits.size() % 6 != 0)
            bits.push_back(0);

        symbols.resize(bits.size() / 6);

        for (size_t i = 0; i < bits.size() / 6; ++i)
        {
            I = (1.0 - 2.0 * bits[6 * i]) * (4 - (1.0 - 2.0 * bits[6 * i + 2]) * (2 - (1.0 - 2.0 * bits[6 * i + 4])));
            Q = (1.0 - 2.0 * bits[6 * i + 1]) * (4 - (1.0 - 2.0 * bits[6 * i + 3]) * (2 - (1.0 - 2.0 * bits[6 * i + 5])));

            symbols[i] = std::complex<float>(I, Q) / sqrtf(42.0);
        }

        break;
    }

    default:
        break;
    }

    return symbols;
}

std::vector<int16_t> demodulator(const std::vector<std::complex<float>> &symbols, SignalModulation mod_type)
{
    std::vector<int16_t> bits;
    if (symbols.empty())
        return bits;

    switch (mod_type)
    {
    case SignalModulation::BPSK: {
        bits.resize(symbols.size());
        for (size_t i = 0; i < symbols.size(); ++i)
        {
            float val = symbols[i].real();
            bits[i] = (val > 0) ? 0 : 1;
        }
        break;
    }

    case SignalModulation::QPSK: {
        bits.resize(symbols.size() * 2);
        for (size_t i = 0; i < symbols.size(); ++i)
        {
            float I = symbols[i].real();
            float Q = symbols[i].imag();

            bits[2 * i] = (I > 0) ? 0 : 1;
            bits[2 * i + 1] = (Q > 0) ? 0 : 1;
        }
        break;
    }

    case SignalModulation::QAM16: {
        bits.resize(symbols.size() * 4);
        float threshold = 2.0f / sqrtf(10.0f);

        for (size_t i = 0; i < symbols.size(); ++i)
        {
            float I = symbols[i].real();
            float Q = symbols[i].imag();

            bits[4 * i] = (I > 0) ? 0 : 1;
            bits[4 * i + 2] = (fabsf(I) > threshold) ? 1 : 0;
            bits[4 * i + 1] = (Q > 0) ? 0 : 1;
            bits[4 * i + 3] = (fabsf(Q) > threshold) ? 1 : 0;
        }
        break;
    }

    case SignalModulation::QAM64: {
        bits.resize(symbols.size() * 6);

        const float norm = sqrtf(42.0f);

        for (size_t i = 0; i < symbols.size(); ++i)
        {
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
        break;
    }
    }

    return bits;
}
