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

    if (buf.size() < size_t(3 * L))
        return;

    double teta = (sd.gardner.BnTs / L) /
                  (sd.gardner.zeta + 1.0/(4.0*sd.gardner.zeta));

    double K1 = (-4 * sd.gardner.zeta * teta) /
                ((1 + 2*sd.gardner.zeta*teta + teta*teta) * sd.gardner.Kp);

    double K2 = (-4 * teta * teta) /
                ((1 + 2*sd.gardner.zeta*teta + teta*teta) * sd.gardner.Kp);

    for (int ns = 0; size_t(ns) < buf.size()/L - 1; ++ns)
    {
        int n = sd.gardner.ss_offset;

        int idx_e = n + L*ns;
        int idx_m = n + L/2 + L*ns;
        int idx_l = n + L + L*ns;

        if (size_t(idx_l) >= buf.size())
            break;

        auto early = buf[idx_e];
        auto mid   = buf[idx_m];
        auto late  = buf[idx_l];

        if (abs(mid) < 200) continue;

        double error =
            mid.real() * (late.real() - early.real()) +
            mid.imag() * (late.imag() - early.imag());

        sd.gardner.ss_p1 += error * K2;
        sd.gardner.ss_p2 += sd.gardner.ss_p1 + error * K1;

        while (sd.gardner.ss_p2 >= 1.0) sd.gardner.ss_p2 -= 1.0;
        while (sd.gardner.ss_p2 < 0.0)  sd.gardner.ss_p2 += 1.0;

        sd.gardner.ss_offset = int(sd.gardner.ss_p2 * L);

        sd.timing_offsets[sd.timing_head] = sd.gardner.ss_offset;
        sd.timing_head = (sd.timing_head + 1) % sd.SCOPE_SIZE;
    }
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

int ofdm_time_sync(const vector<complex<double>>& signal, SharedData& sd){
    int N = sd.ofdm.n_subcarriers;
    int CP = sd.ofdm.cp_len;

    if (signal.size() < N + CP)
        return -1;

    double max_metric = 0.0;
    int best_pos = 0;
    complex<double> best_corr = 0;

    for (size_t d = 0; d + N + CP < signal.size(); ++d){
        complex<double> corr = 0.0;
        double energy = 0.0;

        for (int n = 0; n < CP; ++n){
            corr += signal[d + n] * conj(signal[d + n + N]);
            energy += norm(signal[d + n + N]);
        }

        double metric = norm(corr) / (energy + 1e-12);

        sd.ofdm_sym_sync_corr[sd.ofdm_sym_sync_head] = metric;
        sd.ofdm_sym_sync_head = (sd.ofdm_sym_sync_head + 1) % sd.SCOPE_SIZE;

        if (metric > max_metric){
            max_metric = metric;
            best_pos = d;
            best_corr = corr;
        }
    }

    if (CP > 0 && abs(best_corr) > 1e-12){
        complex<double> avg_corr = best_corr / static_cast<double>(CP);
        double phase = arg(avg_corr);
        sd.ofdm_sync.cfo_estimate = phase / (2.0 * M_PI);
    }

    return best_pos;
}

vector<complex<double>> discard_cp(vector<complex<double>> signal, SharedData &sd){
    int begin = 0;

    vector<complex<double>> sym_blocks;
    vector<complex<double>> result;
    vector<complex<double>> ofdm_signal;

    int Pl_len = sd.ofdm.n_subcarriers;
    int CP_len = sd.ofdm.cp_len;
    int N = Pl_len + CP_len;

    if (begin < 0 || begin + N > (int)signal.size())
        return ofdm_signal;

    fftw_complex *in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * Pl_len);
    fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * Pl_len);

    fftw_plan p = fftw_plan_dft_1d(Pl_len, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    for (size_t i = begin; i + N <= signal.size(); i += N){
        sym_blocks.clear();
        result.clear();

        sym_blocks.insert(sym_blocks.begin(), signal.begin() + i, signal.begin() + i + N);
        sym_blocks.erase(sym_blocks.begin(), sym_blocks.begin() + CP_len);

        for (int j = 0; j < Pl_len; ++j) {
            in[j][0] = sym_blocks[j].real();
            in[j][1] = sym_blocks[j].imag();
        }

        fftw_execute(p);

        result.resize(Pl_len);

        for (int j = 0; j < Pl_len; ++j) {
            result[j] = { out[j][0] / Pl_len, out[j][1] / Pl_len };
        }

        ofdm_signal.insert(ofdm_signal.end(), result.begin(), result.end());
    }

    fftw_destroy_plan(p);
    fftw_free(in);
    fftw_free(out);

    return ofdm_signal;
}

vector<complex<double>> cfo_est(vector<complex<double>> signal, SharedData &sd){
    int N = sd.ofdm.n_subcarriers;
    double cfo = sd.ofdm_sync.cfo_estimate;

    for (int k = 0; size_t(k) < signal.size(); ++k) {
        signal[k] *= exp(complex<double>(0, -2 * M_PI * cfo * k / N));
    }

    return signal;
}

vector<complex<double>> ofdm_equalize(vector<complex<double>> signal, SharedData &sd){
    vector<complex<double>> result;

    int N = sd.ofdm.n_subcarriers;
    complex<double> known_pilot = {1.0, 0.0};

    auto pilots = sd.ofdm.pilot_idx;

    for (size_t i = 0; i + N <= signal.size(); i += N){
        vector<complex<double>> sym(signal.begin() + i, signal.begin() + i + N);
        vector<complex<double>> H(N, {0,0});
        vector<complex<double>> equalized(N);

        for (size_t p = 0; p < pilots.size(); ++p){
            int k = pilots[p];
            H[k] = sym[k] / known_pilot;
        }

        for (size_t p = 0; p < pilots.size() - 1; ++p){
            int k1 = pilots[p];
            int k2 = pilots[p+1];

            complex<double> H1 = H[k1];
            complex<double> H2 = H[k2];

            for (int k = k1 + 1; k < k2; ++k){
                double alpha = double(k - k1) / double(k2 - k1);
                H[k] = H1 + alpha * (H2 - H1);
            }
        }

        for (int k = 0; k < pilots.front(); ++k)
            H[k] = H[pilots.front()];

        for (int k = pilots.back()+1; k < N; ++k)
            H[k] = H[pilots.back()];

        for (int k = 0; k < N; ++k){
            if (abs(H[k]) > 1e-12)
                equalized[k] = sym[k] / H[k];
            else
                equalized[k] = sym[k];
        }

        double phase = 0;
        for (auto p : pilots)
            phase += arg(sym[p]);

        phase /= pilots.size();

        for (int k = 0; k < N; ++k)
            equalized[k] *= exp(complex<double>(0, -phase));

        vector<complex<double>> data_only;

        for (int k = 0; k < N; ++k){
            if (find(pilots.begin(), pilots.end(), k) == pilots.end())
                data_only.push_back(equalized[k]);
        }

        result.insert(result.end(), data_only.begin(), data_only.end());
    }

    return result;
}