#include "sync_freq.h"
#include "sync_time.h"
#include <cmath>
#include <algorithm>

inline double quantize(double val) {
    double q = round(val);

    if (fmod(q, 2.0) == 0.0) {
        q = (q > 0) ? q - 1.0 : q + 1.0;
    }

    return clamp(q, -3.0, 3.0);
}

complex<double> costas_loop(SharedData& sd, complex<double> r){
    if (abs(r.real()) < sd.Threshold) return {};
    auto arg = polar(1.0, -sd.costas.cl_theta_hat);

    complex<double> r_corrected = r * arg;

    double I = r_corrected.real();
    double Q = r_corrected.imag();

    double sign_I = (I > 0) ? 1.0 : (I < 0 ? -1.0 : 0.0);
    double sign_Q = (Q > 0) ? 1.0 : (Q < 0 ? -1.0 : 0.0);

    double error = sign_I * Q - sign_Q * I;

    sd.costas.cl_integrator += error;

    const double integrator_limit = 10.0;
    sd.costas.cl_integrator = max(
        -integrator_limit,
        min(integrator_limit, sd.costas.cl_integrator)
    );

    sd.costas.cl_theta_hat += sd.costas.cl_Kp * error + sd.costas.cl_Ki * sd.costas.cl_integrator;

    sd.costas.cl_theta_hat = fmod(sd.costas.cl_theta_hat + M_PI, 2*M_PI) - M_PI;

    return r_corrected;
}

complex<double> costas_loop_16qam(SharedData& sd, complex<double> r) {
    complex<double> rotator = polar(1.0, -sd.costas.cl_theta_hat);
    complex<double> r_rotated = r * rotator;

    double I = r_rotated.real();
    double Q = r_rotated.imag();
    
    double current_peak = max(abs(I), abs(Q));
    
    if (sd.costas.signal_level == 0.0) {
        sd.costas.signal_level = current_peak;
    } else {
        double alpha = 0.01; 
        sd.costas.signal_level = (1.0 - alpha) * sd.costas.signal_level + alpha * current_peak;
    }

    if (sd.costas.signal_level < 1e-6) {
        return r_rotated;
    }

    double scale = 3.0 / sd.costas.signal_level;
    
    double I_norm = I * scale;
    double Q_norm = Q * scale;

    double I_decision = quantize(I_norm);
    double Q_decision = quantize(Q_norm);

    double error = (I_norm * Q_decision) - (Q_norm * I_decision);

    const double max_err = 1.0; 
    error = clamp(error, -max_err, max_err);

    sd.costas.cl_integrator += error;
    
    double Kp = (double)sd.costas.cl_Kp;
    double Ki = (double)sd.costas.cl_Ki;
    
    sd.costas.cl_theta_hat += Kp * error + Ki * sd.costas.cl_integrator;

    while (sd.costas.cl_theta_hat > M_PI) sd.costas.cl_theta_hat -= 2.0 * M_PI;
    while (sd.costas.cl_theta_hat < -M_PI) sd.costas.cl_theta_hat += 2.0 * M_PI;

    return r_rotated; 
}

vector<complex<double>> cfo_sync_shmid_cox(const vector<complex<double>> &signal, SharedData &sd){
    int N = sd.ofdm.n_subcarriers;
    int L = N / 2;

    complex<double> corr = 0;

    for (int k = 0; k < L; ++k) {
        corr += signal[k] * conj(signal[k + L]);
    }

    double nu_hat = arg(corr) / M_PI;

    sd.ofdm_sync.cfo_estimate = nu_hat;

    vector<complex<double>> corrected_signal = signal;

    for (size_t k = 0; k < corrected_signal.size(); ++k) {
        double phase = -2.0 * M_PI * nu_hat * k / N;
        corrected_signal[k] *= polar(1.0, phase);
    }

    return corrected_signal;
}

vector<complex<double>> cfo_est(const vector<complex<double>> &signal, SharedData &sd){
    int N = sd.ofdm.n_subcarriers;
    int CP = sd.ofdm.cp_len;
    double fs = sd.rx_bandwidth;

    complex<double> corr = 0;

    for (int n = sd.ofdm.sym_begin; n < sd.ofdm.sym_begin + CP; n++) {
        corr += conj(signal[n]) * signal[n + N];
    }

    double epsilon = arg(corr) / (2 * M_PI);

    double delta_f = epsilon * fs / N;

    sd.ofdm_sync.cfo_estimate = delta_f;

    vector<complex<double>> corrected = signal;
    for (size_t n = sd.ofdm.sym_begin; n < signal.size(); n++) {
        double phase = -2 * M_PI * delta_f * n / fs;
        corrected[n] *= complex<double>(cos(phase), sin(phase));
    }

    return corrected;
}

vector<complex<double>> freq_sync(const vector<complex<double>> &signal, SharedData &sd){
    complex<double> C_0 = 0;
    complex<double> C_1 = 0;
    
    int N = sd.ofdm.n_subcarriers;
    int L = N / 2;
    
    vector<complex<double>> pss = generate_shmidt_preamble(sd);
    
    for (size_t n = 0; n < L; ++ n){
        C_0 += signal[n] * conj(pss[n]);
        C_1 += signal[n + L] * conj(pss[n]);
    }

    complex<double> product = C_1 * conj(C_0);
    
    double delta_teta = atan2(product.imag(), product.real());
    
    double delta_f = (delta_teta * sd.rx_bandwidth) / (N * M_PI);

    sd.ofdm_sync.cfo_estimate = delta_f;
    
    vector<complex<double>> corrected_signal = signal;

    for (int n = 0; n < signal.size(); n++) {
        double correction_phase = -2 * M_PI * delta_f * n / sd.rx_bandwidth;
        corrected_signal[n] *= complex<double>(cos(correction_phase), sin(correction_phase));
    }

    return corrected_signal;
}