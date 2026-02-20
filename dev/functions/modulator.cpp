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

vector<complex<double>> generate_known_preamble(int N) {
    vector<complex<double>> preamble(N, {0.0, 0.0});
    
    const complex<double> pilot_seq[] = {
        {1.0, 1.0}, {-1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0},
        {1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {1.0, 1.0},
        {-1.0, -1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0},
        {1.0, 1.0}, {-1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0},
        {-1.0, 1.0}, {1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0},
        {1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}, {-1.0, 1.0},
        {-1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}, {1.0, -1.0},
        {1.0, -1.0}, {-1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}
    };
    
    int seq_idx = 0;
    for (int i = 2; i < N; i += 2) {
        preamble[i] = pilot_seq[seq_idx++ % 32] / sqrt(2.0);
    }
    
    return preamble;
}

vector<complex<double>> preamble_generate(struct SharedData& sd){
    return ofdm_modulator(generate_known_preamble(sd.ofdm.n_subcarriers), sd);
}

vector<complex<double>> insert_pilots(const vector<complex<double>>& symbols, struct SharedData& sd){
    const auto& cfg = sd.ofdm;
    const int N = cfg.n_subcarriers;
    const int data_per_symbol = N - cfg.pilot_idx.size();

    if (symbols.empty() || symbols.size() % data_per_symbol != 0) return {};

    vector<complex<double>> output_signal;

    size_t num_symbols = symbols.size() / data_per_symbol;
    size_t data_ptr = 0;

    for (size_t s = 0; s < num_symbols; ++s) {
        vector<complex<double>> sym_block(N, {0.0, 0.0});

        for (int k = 0; k < N; ++k) {
            bool is_pilot = (find(cfg.pilot_idx.begin(), cfg.pilot_idx.end(), k) != cfg.pilot_idx.end());

            if (is_pilot) {
                sym_block[k] = {1.0, 0.0}; 
            } else {
                if (data_ptr < symbols.size()) {
                    sym_block[k] = symbols[data_ptr];
                    data_ptr++;
                }
            }
        }

        output_signal.insert(output_signal.end(), sym_block.begin(), sym_block.end());
    }

    return output_signal;
}

vector<complex<double>> ofdm_modulator(const vector<complex<double>>& symbols, struct SharedData& sd) {
    
    const auto& cfg = sd.ofdm;
    vector<complex<double>> sym_blocks;
    vector<complex<double>> result;
    vector<complex<double>> ofdm_signal;
    
    const int N = cfg.n_subcarriers;
    
    if (symbols.size() < N || (symbols.size() % N != 0)) return {};

    fftw_complex *in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);

    fftw_plan p = fftw_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);

    for (size_t i = 0; i + N <= symbols.size(); i += N){
        sym_blocks.clear();
        result.clear();

        sym_blocks.insert(sym_blocks.begin(), symbols.begin() + i, symbols.begin() + i + N);
        result.resize(N);

        for (size_t j = 0; j < N; ++j) {
            in[j][0] = sym_blocks[j].real();
            in[j][1] = sym_blocks[j].imag();
        }

        fftw_execute(p);

        for (size_t j = 0; j < N; ++j) {
            result[j] = { out[j][0] / N, out[j][1] / N };
        }

        result.insert(result.begin(), result.end() - cfg.cp_len, result.end());

        ofdm_signal.insert(ofdm_signal.end(), result.begin(), result.end());
    }

    fftw_destroy_plan(p);
    fftw_free(in);
    fftw_free(out);

    return ofdm_signal;
}