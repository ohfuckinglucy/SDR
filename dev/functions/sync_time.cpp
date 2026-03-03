#include "sync_time.h"
#include "ofdm_core.h"
#include <cmath>
#include <algorithm>

void sym_sync(SharedData& sd, const vector<complex<double>>& buf)
{
    int L = sd.form_filter.rx_l;

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

        if (abs(mid) < sd.Threshold) continue;

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

vector<complex<double>> generate_shmidt_preamble(SharedData& sd){
    int N = sd.ofdm.n_subcarriers;
    vector<complex<double>> freq(N, {0,0});

    for (int k = 1; k < N/2; ++k){
        if (k % 2 != 0) continue;
        if (is_guard(k, sd)) continue;
        if (is_guard(N - k, sd)) continue;

        freq[k] = {1,0};
        freq[N-k] = {1,0};
    }

    vector<complex<double>> preamble = ofdm_modulator(freq, sd);
    preamble.erase(preamble.begin(), preamble.begin() + sd.ofdm.cp_len);
    
    return preamble;
}

vector<complex<double>> generate_minn_preamble(SharedData& sd) {
    int N = sd.ofdm.n_subcarriers;
    if (N % 4 != 0) return {}; 

    vector<complex<double>> freq(N, {0.0, 0.0});
    
    for (int k = 0; k < N; k += 4) {
        if (is_guard(k, sd)) continue;
        
        int idx_group = (k / 4) % 4;
        complex<double> val;
        if (idx_group < 2) {
            val = {1.0, 0.0}; 
        } else {
            val = {-1.0, 0.0}; 
        }

        freq[k] = val;
        
        if (k != 0 && k != N/2) {
            freq[N - k] = conj(val);
        }
    }

    vector<complex<double>> preamble = ofdm_modulator(freq, sd);
    
    if (preamble.size() > (size_t)sd.ofdm.cp_len) {
        preamble.erase(preamble.begin(), preamble.begin() + sd.ofdm.cp_len);
    }
    return preamble;
}

int shmidt_sync(const vector<complex<double>>& signal, SharedData& sd){
    int N = sd.ofdm.n_subcarriers;
    int CP = sd.ofdm.cp_len;
    int L = N / 2;

    if (signal.size() < N + CP)
        return -1;

    float max_metric = 0.0;
    int best_pos = 0;

    for (size_t n = 0; n < signal.size() - 2*L; ++n){
        complex<float> corr = 0.0;
        float energy = 0.0;
        float metric = 0.0f;

        for (int k = 0; k < L; ++k){
            corr += signal[n + k] * signal[n + k + L];
            energy += norm(signal[n + k + L]);
        }

        metric = norm(corr) / energy * energy;

        sd.ofdm_sym_sync_corr[sd.ofdm_sym_sync_head] = metric;
        sd.ofdm_sym_sync_head = (sd.ofdm_sym_sync_head + 1) % sd.SCOPE_SIZE;

        if (metric > max_metric){
            max_metric = metric;
            best_pos = n;
        }
    }

    return best_pos;
}

int minn_sync(const vector<complex<double>>& signal, SharedData& sd) {
    int N = sd.ofdm.n_subcarriers;
    int L = N / 4;

    if (signal.size() < (size_t)N) return -1;

    float max_metric = 0.0;
    int best_pos = 0;

    complex<double> P;

    for (size_t n = 0; n <= signal.size() - N; ++n) {
        P = 0.0;

        double energy = 0.0;

        for (int k = 0; k < L; ++k) {
            auto s1 = signal[n + k];
            auto s2 = signal[n + k + L];
            auto s3 = signal[n + k + 2*L];
            auto s4 = signal[n + k + 3*L];
            
            complex<double> first_half = s1 + s2;
            complex<double> second_half = s3 + s4;
            
            P += first_half * conj(second_half);
            
            energy += norm(s1) + norm(s2) + norm(s3) + norm(s4);
        }

        double metric= norm(P); 
        
        metric = norm(P) / (energy * energy / 16.0);

        sd.ofdm_sym_sync_corr[sd.ofdm_sym_sync_head] = metric;
        sd.ofdm_sym_sync_head = (sd.ofdm_sym_sync_head + 1) % sd.SCOPE_SIZE;

        if (metric > max_metric) {
            max_metric = metric;
            best_pos = n;
        }
    }

    if (best_pos > 1920) best_pos = 0;

    return best_pos;
}

vector<int> ofdm_sym_sync(const vector<complex<double>>& signal, SharedData& sd){
    int N = sd.ofdm.n_subcarriers;
    int CP = sd.ofdm.cp_len;
    if(signal.size() < N+CP) return {};

    vector<double> metrics(signal.size(), 0.0);
    double max_metric = 0.0;

    for(size_t d=0; d+N+CP < signal.size(); ++d){
        complex<double> corr = 0.0;
        double energy = 0.0;
        for(int n=0; n<CP; ++n){
            corr += signal[d+n] * conj(signal[d+n+N]);
            energy += norm(signal[d+n] + signal[d+n+N]);
        }
        double metric = norm(corr) / (energy + 1e-12);
        metrics[d] = metric;
        if(metric > max_metric) max_metric = metric;
    }

    for(size_t i=0;i<signal.size();++i){
        sd.ofdm_sym_sync_corr[sd.ofdm_sym_sync_head] = metrics[i];
        sd.ofdm_sym_sync_head = (sd.ofdm_sym_sync_head+1) % sd.SCOPE_SIZE;
    }

    double threshold = 0.5*max_metric;
    vector<int> raw_peaks;

    int window = max(2, CP/2);
    for(size_t i=window; i+window<metrics.size(); ++i){
        bool peak = metrics[i] > threshold;
        for(int w=-window; w<=window && peak; ++w) if(w!=0) peak &= metrics[i] > metrics[i+w];
        if(peak) raw_peaks.push_back(i);
    }

    vector<int> cp_indices;
    int min_dist = N; 
    for(int idx : raw_peaks){
        if(cp_indices.empty() || idx - cp_indices.back() >= min_dist)
            cp_indices.push_back(idx);
    }

    if(!cp_indices.empty() && CP>0){
        complex<double> avg_corr = 0.0;
        int first_idx = cp_indices[0];
        for(int n=0;n<CP;++n) avg_corr += signal[first_idx+n]*conj(signal[first_idx+n+N]);
        avg_corr /= CP;
        sd.ofdm_sync.cfo_estimate = arg(avg_corr)/(2.0*M_PI);
    }

    return cp_indices;
}