#include "ofdm_core.hpp"

#include <cstddef>
#include <immintrin.h>
#include <vector>

std::vector<std::complex<float>> generate_zc_preamble(SharedData &sd)
{
    int N;
    int16_t q;
    {
        std::lock_guard<std::mutex> lock(sd.mtx);
        N = sd.fftplans.plan_N;
        q = sd.ofdmcfg.q;
    }

    const int n_zc = 127;
    const std::complex<float> j(0, 1);

    std::vector<std::complex<float>> freq(N, { 0.0f, 0.0f });
    int start_idx = -n_zc / 2;

    for (int i = 0; i < n_zc; ++i)
    {
        int k = start_idx + i;
        int idx = (k < 0) ? (N + k) : k;

        if (idx < 0 || idx >= N)
            continue;

        if (is_guard(idx, N))
            continue;

        float phase = -M_PI * q * i * (i + 1) / n_zc;
        freq[idx] = std::exp(j * phase);
    }

    std::vector<std::complex<float>> time_domain = ofdm_modulator(freq, sd);

    {
        std::lock_guard<std::mutex> lock(sd.mtx);
        sd.ofdmcfg.zc_reference = time_domain;
    }

    return time_domain;
}

int zadoff_sync(const std::vector<std::complex<float>> &signal, SharedData &sd)
{
    std::vector<std::complex<float>> zc;
    {
        std::lock_guard<std::mutex> lock(sd.mtx);
        zc = sd.ofdmcfg.zc_reference;
    }

    size_t signal_len = signal.size();
    size_t zc_len = zc.size();

    if (signal_len < zc_len)
        return -1;

    {
        std::lock_guard<std::mutex> lock(sd.mtx);
        if (sd.timing_offsets.size() != signal_len - zc_len + 1)
            sd.timing_offsets.resize(signal_len - zc_len + 1);
    }

    float max_norm = -1.f;
    int best_idx = 0;

    const float *sig_ptr = reinterpret_cast<const float *>(signal.data());
    const float *zc_ptr = reinterpret_cast<const float *>(zc.data());

    {
        std::lock_guard<std::mutex> lock(sd.mtx);
        for (size_t n = 0; n <= signal_len - zc_len; ++n)
        {
            float sum_re = 0.f;
            float sum_im = 0.f;

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
    }

    return best_idx;
}
