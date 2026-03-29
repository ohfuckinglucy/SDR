#include "sync_time.h"
#include "ofdm_core.h"
#include <cmath>
#include <algorithm>
#include <immintrin.h>

vector<complex<double>> generate_zc_preamble(SharedData &sd)
{
    int N = sd.ofdm.n_subcarriers;
    int CP = sd.ofdm.cp_len;
    int n_zc = 127;
    int16_t q = 5;
    const complex<double> j(0, 1);

    vector<complex<double>> zc;
    zc.reserve(n_zc);
    for (int i = 0; i < n_zc; ++i)
    {
        double phase = -M_PI * q * i * (i + 1) / n_zc;
        zc.push_back(exp(j * phase));
    }

    vector<complex<double>> freq(N, {0, 0});
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

    vector<complex<double>> time_domain = ofdm_modulator(freq, sd);

    return time_domain;
}

int zc_sync(const vector<complex<double>> &signal, SharedData &sd)
{
    const auto &ref = sd.ofdm_sync.reference;
    int N = ref.size();
    if ((int)signal.size() < N)
        return -1;

    double max_metric = 0.0;
    int best_pos = 0;

    const double *ref_ptr = reinterpret_cast<const double*>(ref.data());

    sd.timing_offsets.clear();

    for (size_t n = 0; n <= signal.size() - N; ++n)
    {
        const double *sig_ptr = reinterpret_cast<const double*>(signal.data() + n);

        __m256d corr_re_vec = _mm256_setzero_pd();
        __m256d corr_im_vec = _mm256_setzero_pd();
        __m256d energy_vec  = _mm256_setzero_pd();

        int k = 0;
        for (; k <= N - 2; k += 2)
        {
            __m256d rx  = _mm256_loadu_pd(sig_ptr + 2*k);
            __m256d rf  = _mm256_loadu_pd(ref_ptr + 2*k);

            __m256d rx_re = _mm256_unpacklo_pd(rx, rx);
            __m256d rx_im = _mm256_unpackhi_pd(rx, rx);
            __m256d rf_re = _mm256_unpacklo_pd(rf, rf);
            __m256d rf_im = _mm256_unpackhi_pd(rf, rf);

            corr_re_vec = _mm256_add_pd(corr_re_vec,
                _mm256_add_pd(_mm256_mul_pd(rx_re, rf_re),
                              _mm256_mul_pd(rx_im, rf_im)));

            corr_im_vec = _mm256_add_pd(corr_im_vec,
                _mm256_sub_pd(_mm256_mul_pd(rx_im, rf_re),
                              _mm256_mul_pd(rx_re, rf_im)));

            energy_vec = _mm256_add_pd(energy_vec,
                _mm256_add_pd(_mm256_mul_pd(rx_re, rx_re),
                              _mm256_mul_pd(rx_im, rx_im)));
        }

        double tmp[4];
        _mm256_storeu_pd(tmp, corr_re_vec);
        double corr_re = tmp[0] + tmp[1] + tmp[2] + tmp[3];
        _mm256_storeu_pd(tmp, corr_im_vec);
        double corr_im = tmp[0] + tmp[1] + tmp[2] + tmp[3];
        _mm256_storeu_pd(tmp, energy_vec);
        double energy  = tmp[0] + tmp[1] + tmp[2] + tmp[3];

        for (; k < N; ++k)
        {
            double rx_re = signal[n+k].real(), rx_im = signal[n+k].imag();
            double rf_re = ref[k].real(),      rf_im = ref[k].imag();
            corr_re += rx_re*rf_re + rx_im*rf_im;
            corr_im += rx_im*rf_re - rx_re*rf_im;
            energy  += rx_re*rx_re + rx_im*rx_im;
        }

        double metric = (corr_re*corr_re + corr_im*corr_im) / (energy + 1e-12);

        sd.timing_offsets.push_back(metric);

        if (metric > max_metric)
        {
            max_metric = metric;
            best_pos = (int)n;
        }
    }

    return best_pos;
}