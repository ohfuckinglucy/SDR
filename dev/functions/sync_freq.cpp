#include "sync_freq.h"
#include "sync_time.h"
#include <cmath>
#include <algorithm>

vector<complex<double>> cfo_est(const vector<complex<double>> &signal, SharedData &sd)
{
    int N = sd.ofdm.n_subcarriers;
    int CP = sd.ofdm.cp_len;
    int sym_len = N + CP;
    double fs = sd.rx_bandwidth;

    vector<complex<double>> corrected = signal;

    int n_symbols = (signal.size()) / sym_len;

    for (int sym = 0; sym < n_symbols; sym++)
    {
        int sym_start = sym * sym_len;

        complex<double> corr = 0;
        for (int k = 0; k < CP; k++)
        {
            corr += conj(corrected[sym_start + k]) * corrected[sym_start + k + N];
        }

        double epsilon = arg(corr) / (2 * M_PI);
        double delta_f = epsilon * fs / N;

        for (int k = 0; k < sym_len; k++)
        {
            int n = sym_start + k;
            double phase = -2 * M_PI * delta_f * k / fs;
            corrected[n] *= complex<double>(cos(phase), sin(phase));
        }

        sd.ofdm_sync.cfo_estimate = delta_f;
    }

    return corrected;
}