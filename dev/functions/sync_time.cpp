#include "sync_time.h"
#include "ofdm_core.h"
#include <cmath>
#include <algorithm>

vector<complex<double>> generate_zc_preamble(SharedData &sd)
{
    int N = sd.ofdm.n_subcarriers;
    int CP = sd.ofdm.cp_len;
    int n_zc = 63;
    int16_t q = 26;
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
    if (signal.size() < N)
        return -1;

    float max_metric = 0.0;
    int best_pos = 0;

    for (size_t n = 0; n <= signal.size() - N; ++n)
    {
        double corr_re = 0, corr_im = 0;
        double energy = 0;

        for (int k = 0; k < N; ++k)
        {
            double rx_re = signal[n + k].real();
            double rx_im = signal[n + k].imag();
            double ref_re = ref[k].real();
            double ref_im = ref[k].imag();

            corr_re += (rx_re * ref_re + rx_im * ref_im);
            corr_im += (rx_im * ref_re - rx_re * ref_im);
            energy += (rx_re * rx_re + rx_im * rx_im);
        }

        double metric = (corr_re * corr_re + corr_im * corr_im) / (energy + 1e-12);

        sd.ofdm_sym_sync_corr[sd.ofdm_sym_sync_head] = (float)metric;
        sd.ofdm_sym_sync_head = (sd.ofdm_sym_sync_head + 1) % sd.SCOPE_SIZE;

        if (metric > max_metric)
        {
            max_metric = (float)metric;
            best_pos = (int)n;
        }
    }

    return best_pos;
}