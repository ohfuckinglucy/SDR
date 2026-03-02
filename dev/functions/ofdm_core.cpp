#include "ofdm_core.h"
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

vector<complex<double>> insert_pilots(const vector<complex<double>>& symbols, SharedData& sd){
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
    vector<complex<double>> out;
    out.reserve(num_ofdm * N);

    size_t data_ptr = 0;

    for (size_t s = 0; s < num_ofdm; ++s) {
        vector<complex<double>> block(N, {0.0, 0.0});

        for (int k = 0; k < N; ++k) {
            if (is_guard(k, sd)) continue;

            if (find(pilots.begin(), pilots.end(), k) != pilots.end()) {
                block[k] = {1.0, 0.0};
            } else {
                if (data_ptr < symbols.size())
                    block[k] = symbols[data_ptr++];
                else
                    block[k] = {0.0, 0.0};
            }
        }

        out.insert(out.end(), block.begin(), block.end());
    }

    return out;
}

vector<complex<double>> ofdm_modulator(const vector<complex<double>>& freq_symbols, SharedData& sd) {
    const int N = sd.ofdm.n_subcarriers;
    const int CP = sd.ofdm.cp_len;
    if (freq_symbols.size() % N != 0) return {};

    vector<complex<double>> ofdm_signal;
    fftw_complex* in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex* out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_plan p = fftw_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);

    size_t ptr = 0;
    while (ptr < freq_symbols.size()) {
        for (int i = 0; i < N; ++i) {
            in[i][0] = freq_symbols[ptr].real();
            in[i][1] = freq_symbols[ptr].imag();
            ptr++;
        }

        fftw_execute(p);

        vector<complex<double>> time_sym(N);
        for (int i = 0; i < N; ++i) time_sym[i] = {out[i][0] / N, out[i][1] / N};

        time_sym.insert(time_sym.begin(), time_sym.end() - CP, time_sym.end());
        ofdm_signal.insert(ofdm_signal.end(), time_sym.begin(), time_sym.end());
    }

    fftw_destroy_plan(p);
    fftw_free(in);
    fftw_free(out);
    
    return ofdm_signal;
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

vector<complex<double>> ofdm_equalize(const vector<complex<double>>& signal, SharedData &sd){
    vector<complex<double>> result;

    int N = sd.ofdm.n_subcarriers;
    const auto& pilots = sd.ofdm.pilot_idx;
    complex<double> known_pilot = {1.0, 0.0};

    if (pilots.empty())
        return result;

    vector<bool> is_pilot(N, false);
    for (auto p : pilots)
        if (p >= 0 && p < N)
            is_pilot[p] = true;

    for (size_t i = 0; i + N <= signal.size(); i += N){
        vector<complex<double>> sym(signal.begin() + i,
                                    signal.begin() + i + N);

        vector<complex<double>> H(N, {0.0, 0.0});
        vector<complex<double>> equalized(N);

        for (auto k : pilots)
            H[k] = sym[k] / known_pilot;

        for (size_t p = 0; p + 1 < pilots.size(); ++p){
            int k1 = pilots[p];
            int k2 = pilots[p + 1];

            complex<double> H1 = H[k1];
            complex<double> H2 = H[k2];

            for (int k = k1 + 1; k < k2; ++k){
                if (is_guard(k, sd)) continue;

                double alpha = double(k - k1) / double(k2 - k1);
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

        double phase = 0.0;
        int pilot_count = 0;

        for (auto p : pilots){
            phase += arg(equalized[p]);
            pilot_count++;
        }

        if (pilot_count > 0)
            phase /= pilot_count;

        for (int k = 0; k < N; ++k){
            if (!is_guard(k, sd))
                equalized[k] *= exp(complex<double>(0, -phase));
        }

        for (int k = 0; k < N; ++k){
            if (is_guard(k, sd)) continue;
            if (is_pilot[k]) continue;

            result.push_back(equalized[k]);
        }
    }

    return result;
}