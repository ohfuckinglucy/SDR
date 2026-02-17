#include "modulator.h"
#include "header.h"

const complex<double> i0(0, 0);
const complex<double> i1(1, 1);
const complex<double> i2(-1, 1);
const complex<double> i3(-1, -1);
const complex<double> i4(1, -1);

vector<complex<double>> modulator(vector<int16_t> bits, int len_bits, string type){
    double I, Q;

    if (type == "QAM::2"){
        vector<complex<double>> symbols(len_bits);
        for (int i = 0; i < len_bits; ++i){
            I = 1 - 2*bits[i];
            Q = 1 - 2*bits[i];

            symbols[i] = complex<double>(I, Q)/sqrt(2);
        }
        return symbols;
    } else if (type == "QAM::4"){
        if (len_bits % 2 != 0) {
            perror("L_Bits % 2 != 0");
            exit(1);
        }
        vector<complex<double>> symbols(len_bits/2);
        for (int i = 0; i < len_bits/2; ++i){
            I = 1 - 2*bits[2*i];
            Q = 1 - 2*bits[2*i+1];

            symbols[i] = complex<double>(I, Q)/sqrt(2);
        }
        return symbols;
    } else if (type == "QAM::16"){
        if (len_bits % 4 != 0){
            perror("L_Bits % 4 != 0");
            exit(1);
        }
        vector<complex<double>> symbols(len_bits/4);
        for (int i = 0; i < len_bits/4; ++i){
            I = (1.0 - 2.0 * bits[4*i]) * (2.0 - (1.0 - 2.0 * bits[4*i + 2]));
            Q = (1.0 - 2.0 * bits[4*i + 1]) * (2.0 - (1.0 - 2.0 * bits[4*i + 3]));

            symbols[i] = complex<double>(I, Q)/sqrt(10);
        }
        return symbols;
    } else {
        perror("unluck");
        exit(1);
    }
}

int bits_per_symbol(string type){
    if (type == "QAM::2"){
        return 1;
    } else if( type == "QAM::4"){
        return 2;
    } else {
        return 4;
    }
}

vector<complex<double>> ofdm_modulator(const vector<complex<double>>& symbols, struct SharedData& sd) {
    const auto& cfg = sd.ofdm;

    int N = cfg.n_subcarriers;
    int cp_len = cfg.cp_len;
    int n_symbols = cfg.n_ofdm_symbols;
    const auto& pilot_idx = cfg.pilot_idx;

    vector<int> data_idx;
    for (int k = 0; k < N; ++k) {
        if (find(pilot_idx.begin(), pilot_idx.end(), k) == pilot_idx.end()) {
            data_idx.push_back(k);
        }
    }

    int n_data_per_symbol = data_idx.size();
    int total_needed = n_symbols * n_data_per_symbol;

    complex<double> pilot_val = {1.0 / sqrt(2.0), 1.0 / sqrt(2.0)}; 

    vector<complex<double>> ofdm_signal;
    ofdm_signal.reserve(n_symbols * (N + cp_len));

    int sym_ptr = 0;

    fftw_complex* in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex* out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
    fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);

    for (int s = 0; s < n_symbols; ++s) {
        for (int k = 0; k < N; ++k) {
            in[k][0] = 0.0;
            in[k][1] = 0.0;
        }

        for (int k : pilot_idx) {
            in[k][0] = pilot_val.real();
            in[k][1] = pilot_val.imag();
        }

        for (int k : data_idx) {
            const auto& sym = symbols[sym_ptr++];
            in[k][0] = sym.real();
            in[k][1] = sym.imag();
        }

        fftw_execute(plan);

        vector<complex<double>> time_domain(N);
        for (int n = 0; n < N; ++n) {
            double scale = 1.0 / N;
            time_domain[n] = complex<double>(out[n][0] * scale, out[n][1] * scale);
        }

        vector<complex<double>> with_cp;
        with_cp.reserve(N + cp_len);

        for (int i = N - cp_len; i < N; ++i) {
            with_cp.push_back(time_domain[i]);
        }
        for (int i = 0; i < N; ++i) {
            with_cp.push_back(time_domain[i]);
        }

        ofdm_signal.insert(ofdm_signal.end(), with_cp.begin(), with_cp.end());
    }

    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);

    return ofdm_signal;
}