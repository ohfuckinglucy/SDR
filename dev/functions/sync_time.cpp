#include "sync_time.h"
#include "ofdm_core.h"
#include <cmath>
#include <algorithm>

std::vector<std::complex<float>> generate_zc_preamble(SharedData &sd)
{
    int N = sd.ofdm.n_subcarriers;
    int CP = sd.ofdm.cp_len;
    int n_zc = 127;
    int16_t q = 5;
    const std::complex<float> j(0, 1);

    std::vector<std::complex<float>> zc;
    zc.reserve(n_zc);
    for (int i = 0; i < n_zc; ++i)
    {
        float phase = -M_PI * q * i * (i + 1) / n_zc;
        zc.push_back(exp(j * phase));
    }

    std::vector<std::complex<float>> freq(N, {0, 0});
    int half_zc = n_zc / 2;

    for (int i = 0; i < n_zc; ++i)
    {
        if (i == half_zc)
            continue;

        int k = (i - half_zc);
        int idx = (k < 0) ? (N + k) : k;

        if (is_guard(idx, sd))
            continue;
        freq[idx] = zc[i];
    }

    std::vector<std::complex<float>> time_domain = ofdm_modulator(freq, sd);

    return time_domain;
}

int zadoff_sync(const std::vector<std::complex<float>> &signal, SharedData &sd)
{
    const auto &zc = sd.ofdm_sync.reference;
    size_t signal_len = signal.size();
    size_t zc_len = zc.size();

    if (signal_len < zc_len)
        return -1;

    if (sd.timing_offsets.size() != signal_len - zc_len + 1)
        sd.timing_offsets.resize(signal_len - zc_len + 1);

    float max_norm = -1.f;
    int best_idx = 0;

    const float *sig_ptr = reinterpret_cast<const float *>(signal.data());
    const float *zc_ptr = reinterpret_cast<const float *>(zc.data());

    for (size_t n = 0; n <= signal_len - zc_len; ++n)
    {
        float sum_re = 0.f;
        float sum_im = 0.f;

#pragma omp simd reduction(+ : sum_re, sum_im)
        for (size_t k = 0; k < zc_len; ++k)
        {
            float sig_re = sig_ptr[2 * (n + k)];
            float sig_im = sig_ptr[2 * (n + k) + 1];

            float zc_re = zc_ptr[2 * k];
            float zc_im = zc_ptr[2 * k + 1];

            sum_re += sig_re * zc_re + sig_im * zc_im;
            sum_im += sig_im * zc_re - sig_re * zc_im;
        }

        float cur_norm = sum_re * sum_re + sum_im * sum_im;
        sd.timing_offsets[n] = cur_norm;

        if (cur_norm > max_norm)
        {
            max_norm = cur_norm;
            best_idx = (int)n;
        }
    }

    return best_idx;
}