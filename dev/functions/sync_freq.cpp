#include "sync_freq.h"
#include "sync_time.h"
#include <cmath>
#include <algorithm>

vector<complex<double>> cfo_est(const vector<complex<double>> &signal, SharedData &sd){
    int N = sd.ofdm.n_subcarriers;
    int CP = sd.ofdm.cp_len;
    double fs = sd.rx_bandwidth;

    complex<double> corr = 0;

    for (int n = sd.ofdm.sym_begin; n < sd.ofdm.sym_begin + CP; n++) {
        corr += conj(signal[n]) * signal[n + N];
    }

    double epsilon = arg(corr) / (2 * M_PI);

    double delta_f =  epsilon * fs / N;

    sd.ofdm_sync.cfo_estimate = delta_f;

    vector<complex<double>> corrected = signal;
    for (size_t n = sd.ofdm.sym_begin; n < signal.size(); n++) {
        double phase = -2 * M_PI * delta_f * n / fs;
        corrected[n] *= complex<double>(cos(phase), sin(phase));
    }

    return corrected;
}