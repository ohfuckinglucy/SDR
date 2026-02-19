#include "header.h"
#include <algorithm>

vector<complex<double>> UpSampler(const vector<complex<double>>& symbols, int L){
    vector<complex<double>> symbols_ups(symbols.size() * L);
    for (size_t i = 0; i < symbols.size()*L; i++){
        symbols_ups[i] = i0;
    }
    for (size_t i = 0; i < symbols.size(); i ++){
        symbols_ups[i*L] = symbols[i];
    }

    return symbols_ups;
}

void filter(complex<double>* symbols_ups, int len_symbols_ups, int L) {
    if (L <= 1 || len_symbols_ups <= 0) return;

    vector<complex<double>> impulse(L, 1.0);
    vector<complex<double>> sum(len_symbols_ups, 0.0);

    for (int i = 0; i < len_symbols_ups; i++) {
        for (int j = 0; j < L && (i - j) >= 0; j++) {
            sum[i] += impulse[j] * symbols_ups[i - j];
        }
    }

    for (int i = 0; i < len_symbols_ups; i++) {
        symbols_ups[i] = sum[i];
    }
}

void sym_sync(SharedData& sd, const vector<complex<double>>& buf)
{
    int L = sd.FormFilter.rx_l;

    if (buf.size() < 3 * L)
        return;

    double teta = (sd.gardner.BnTs / L) /
                  (sd.gardner.zeta + 1.0/(4.0*sd.gardner.zeta));

    double K1 = (-4 * sd.gardner.zeta * teta) /
                ((1 + 2*sd.gardner.zeta*teta + teta*teta) * sd.gardner.Kp);

    double K2 = (-4 * teta * teta) /
                ((1 + 2*sd.gardner.zeta*teta + teta*teta) * sd.gardner.Kp);

    for (int ns = 0; ns < buf.size()/L - 1; ++ns)
    {
        int n = sd.gardner.ss_offset;

        int idx_e = n + L*ns;
        int idx_m = n + L/2 + L*ns;
        int idx_l = n + L + L*ns;

        if (idx_l >= buf.size())
            break;

        auto early = buf[idx_e];
        auto mid   = buf[idx_m];
        auto late  = buf[idx_l];

        double error =
            mid.real() * (late.real() - early.real()) +
            mid.imag() * (late.imag() - early.imag());

        sd.gardner.ss_p1 += error * K2;
        sd.gardner.ss_p2 += sd.gardner.ss_p1 + error * K1;

        while (sd.gardner.ss_p2 >= 1.0) sd.gardner.ss_p2 -= 1.0;
        while (sd.gardner.ss_p2 < 0.0)  sd.gardner.ss_p2 += 1.0;

        sd.gardner.ss_offset = int(sd.gardner.ss_p2 * L);
    }
}


complex<double> costas_loop(SharedData& sd, complex<double> r){
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

inline double quantize(double val) {
    double q = round(val);

    if (fmod(q, 2.0) == 0.0) {
        q = (q > 0) ? q - 1.0 : q + 1.0;
    }

    return clamp(q, -3.0, 3.0);
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