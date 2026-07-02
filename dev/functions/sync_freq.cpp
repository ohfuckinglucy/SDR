#include "ofdm_core.hpp"

std::vector<std::complex<float>> cfo_est(const std::vector<std::complex<float>> &signal, SharedData &sd)
{
    int N = sd.ofdmcfg.N;
    int CP = sd.ofdmcfg.CP;
    int sym_len = N + CP;
    float fs = sd.SDR.rx_sample_rate;

    int n_symbols = signal.size() / sym_len;
    if (n_symbols < 1)
        return {};

    std::complex<float> corr_sum = 0;
    for (int sym = 0; sym < n_symbols; sym++)
    {
        int sym_start = sym * sym_len;
        std::complex<float> corr = 0;
        for (int k = 0; k < CP; k++)
            corr += conj(signal[sym_start + k]) * signal[sym_start + k + N];
        corr_sum += corr;
    }

    float epsilon = std::arg(corr_sum) / (2 * M_PI);
    float delta_f = epsilon * fs / N;
    sd.ofdmcfg.cfo_est = delta_f;

    std::vector<std::complex<float>> corrected(signal.size());
    for (size_t n = 0; n < signal.size(); n++)
    {
        float phase = -2 * M_PI * delta_f * n / fs;
        corrected[n] = signal[n] * std::complex<float>(std::cos(phase), std::sin(phase));
    }

    return corrected;
}
