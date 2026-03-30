#include "ofdm_core.h"
#include "modulator.h"
#include <algorithm>
#include <fftw3.h>

bool is_guard(int k, SharedData &sd){
    int N = sd.ofdm.n_subcarriers;

    int nyq = N/2;

    if (k < sd.ofdm.guard_dc || k >= N - sd.ofdm.guard_dc)
        return true;

    if (abs(k - nyq) <= sd.ofdm.guard_edge)
        return true;

    return false;
}

vector<complex<float>> insert_pilots(const vector<complex<float>>& symbols, SharedData& sd){
    sd.ofdm.padding = 0;
    const int N = sd.ofdm.n_subcarriers;
    const int dc = 0;
    const int nyq = N / 2;
    const auto& pilots = sd.ofdm.pilot_idx;

    int usable = 0;
    for (int k = 0; k < N; ++k)
        if (!is_guard(k, sd))
            usable++;
    usable -= pilots.size();
    if (usable <= 0) return {};

    size_t num_ofdm = (symbols.size() + usable - 1) / usable;
    vector<complex<float>> out;
    out.reserve(num_ofdm * N);

    size_t data_ptr = 0;

    for (size_t s = 0; s < num_ofdm; ++s) {
        vector<complex<float>> block(N, {0.0, 0.0});

        for (int k = 0; k < N; ++k) {
            if (is_guard(k, sd)) continue;

            if (find(pilots.begin(), pilots.end(), k) != pilots.end()) {
                block[k] = {1.0, 0.0};
            } else {
                if (data_ptr < symbols.size())
                    block[k] = symbols[data_ptr++];
                else
                    block[k] = {0.0, 0.0};
                    sd.ofdm.padding ++;
            }
        }

        out.insert(out.end(), block.begin(), block.end());
    }

    return out;
}

vector<complex<float>> ofdm_modulator(const vector<complex<float>>& freq_symbols, SharedData& sd) {
    const int N = sd.ofdm.n_subcarriers;
    const int CP = sd.ofdm.cp_len;
    if (freq_symbols.size() % N != 0) return {};

    vector<complex<float>> ofdm_signal;
    ofdm_signal.reserve(freq_symbols.size() + (freq_symbols.size()/N)*CP);

    size_t ptr = 0;
    while (ptr < freq_symbols.size()) {
        for (int i = 0; i < N; ++i) {
            sd.fft.ifft_in[i][0] = freq_symbols[ptr].real();
            sd.fft.ifft_in[i][1] = freq_symbols[ptr].imag();
            ptr++;
        }

        fftw_execute(sd.fft.ofdm_ifft_plan);

        vector<complex<float>> time_sym(N);
        for (int i = 0; i < N; ++i) time_sym[i] = {sd.fft.ifft_out[i][0] / N, sd.fft.ifft_out[i][1] / N};

        time_sym.insert(time_sym.begin(), time_sym.end() - CP, time_sym.end());
        ofdm_signal.insert(ofdm_signal.end(), time_sym.begin(), time_sym.end());
    }
    
    return ofdm_signal;
}

vector<complex<float>> discard_cp(vector<complex<float>> signal, SharedData &sd){
    int begin = 0;

    vector<complex<float>> sym_blocks;
    vector<complex<float>> result;
    vector<complex<float>> ofdm_signal;

    int Pl_len = sd.ofdm.n_subcarriers;
    int CP_len = sd.ofdm.cp_len;
    int N = Pl_len + CP_len;

    if (begin < 0 || begin + N > (int)signal.size())
        return ofdm_signal;

    ofdm_signal.reserve(signal.size() / N * Pl_len);

    for (size_t i = begin; i + N <= signal.size(); i += N){
        sym_blocks.clear();
        result.clear();

        sym_blocks.insert(sym_blocks.begin(), signal.begin() + i + CP_len, signal.begin() + i + N);

        for (int j = 0; j < Pl_len; ++j) {
            sd.fft.ofdm_rx_in[j][0] = sym_blocks[j].real();
            sd.fft.ofdm_rx_in[j][1] = sym_blocks[j].imag();
        }

        fftw_execute(sd.fft.ofdm_fft_plan);

        for (int j = 0; j < Pl_len; ++j) {
            float re = sd.fft.ofdm_rx_out[j][0] / Pl_len;
            float im = sd.fft.ofdm_rx_out[j][1] / Pl_len;
            ofdm_signal.emplace_back(re, im);
        }
    }

    return ofdm_signal;
}

void update_pilots(SharedData& sd) {
    sd.ofdm.pilot_idx.clear();

    int N = sd.ofdm.n_subcarriers;
    int num = sd.ofdm.num_pilots;
    if (num <= 0 || N <= 0) return;

    vector<int> available;

    for (int k = 0; k < N; ++k) {
        if (!is_guard(k, sd))
            available.push_back(k);
    }

    if (available.size() <= num) return;

    int step = available.size() / (num + 1);

    for (int i = 0; i < num; ++i) {
        int pos = available[(i + 1) * step];
        sd.ofdm.pilot_idx.push_back(pos);
    }
}

vector<complex<float>> ofdm_equalize(const vector<complex<float>>& signal, SharedData &sd){
    vector<complex<float>> result;

    int N = sd.ofdm.n_subcarriers;
    const auto& pilots = sd.ofdm.pilot_idx;
    complex<float> known_pilot = {1.0, 0.0};

    if (pilots.empty())
        return result;
    if (signal.empty()){
        return result;
    }

    vector<bool> is_pilot(N, false);
    for (auto p : pilots)
        if (p >= 0 && p < N)
            is_pilot[p] = true;

    for (size_t i = 0; i + N <= signal.size(); i += N){
        vector<complex<float>> sym(signal.begin() + i, signal.begin() + i + N);

        vector<complex<float>> H(N, {0.0, 0.0});
        vector<complex<float>> equalized(N);

        for (auto k : pilots)
            H[k] = sym[k] / known_pilot;

        for (size_t p = 0; p + 1 < pilots.size(); ++p){
            int k1 = pilots[p];
            int k2 = pilots[p + 1];

            complex<float> H1 = H[k1];
            complex<float> H2 = H[k2];

            for (int k = k1 + 1; k < k2; ++k){
                if (is_guard(k, sd)) continue;

                float alpha = float(k - k1) / float(k2 - k1);
                H[k] = H1 + alpha * (H2 - H1);
            }
        }

        for (int k = 0; k < pilots.front(); ++k)
            if (!is_guard(k, sd))
                H[k] = H[pilots.front()];

        for (int k = pilots.back() + 1; k < N; ++k)
            if (!is_guard(k, sd))
                H[k] = H[pilots.back()];

        for (int k = 0; k < N; ++k){
            if (is_guard(k, sd))
                continue;

            if (abs(H[k]) > 1e-12)
                equalized[k] = sym[k] / H[k];
            else
                equalized[k] = sym[k];
        }

        float phase = 0.0;
        int pilot_count = 0;

        for (auto p : pilots){
            phase += arg(equalized[p]);
            pilot_count++;
        }

        if (pilot_count > 0)
            phase /= pilot_count;

        for (int k = 0; k < N; ++k){
            if (!is_guard(k, sd))
                equalized[k] *= exp(complex<float>(0, -phase));
        }

        for (int k = 0; k < N; ++k){
            if (is_guard(k, sd)) continue;
            if (is_pilot[k]) continue;

            result.push_back(equalized[k]);
        }
    }

    return result;
}