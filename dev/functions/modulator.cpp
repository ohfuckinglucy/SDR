#include "modulator.h"
#include "header.h"
#include <algorithm>

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

vector<complex<double>> generate_shmidt_preamble(SharedData& sd){
    int N = sd.ofdm.n_subcarriers;
    vector<complex<double>> freq(N, {0,0});

    for (int k = 1; k < N/2; ++k){
        if (k % 2 == 0){
            freq[k] = {1,0};
            freq[N-k] = {1,0};
        }
    }

    vector<complex<double>> preamble = ofdm_modulator(freq, sd);
    preamble.erase(preamble.begin(), preamble.begin() + sd.ofdm.cp_len);
    
    return preamble;
}

// vector<complex<double>> generate_shmidt_preamble(SharedData& sd){
//     int N = sd.ofdm.n_subcarriers;
//     vector<complex<double>> freq(N, {0,0});

//     for (int k = 1; k < N/2; ++k){
//         if (k % 2 == 0){
//             freq[k] = {1,0};
//             freq[N-k] = {1,0};
//         }
//     }

//     return ofdm_modulator(freq, sd);
// }

vector<complex<double>> insert_pilots(const vector<complex<double>>& symbols, SharedData& sd) {
    const int N = sd.ofdm.n_subcarriers;
    const int dc = 0;
    const int nyq = N / 2;
    const auto& pilots = sd.ofdm.pilot_idx;

    int usable = N - 2 - pilots.size();
    if (usable <= 0) return {};

    size_t num_ofdm = (symbols.size() + usable - 1) / usable;
    vector<complex<double>> out;
    out.reserve(num_ofdm * N);

    size_t data_ptr = 0;

    for (size_t s = 0; s < num_ofdm; ++s) {
        vector<complex<double>> block(N, {0.0, 0.0});

        for (int k = 0; k < N; ++k) {
            if (k == dc || k == nyq) continue;

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

void update_pilots(SharedData& sd) {
    sd.ofdm.pilot_idx.clear();
    int N = sd.ofdm.n_subcarriers;
    int num = sd.ofdm.num_pilots;
    if (num <= 0 || N <= 0) return;

    int usable = N - 2;
    int step = usable / (num + 1);

    for (int i = 0; i < num; ++i) {
        int pos = 1 + (i + 1) * step;
        if (pos != N/2 && pos < N)
            sd.ofdm.pilot_idx.push_back(pos);
    }
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