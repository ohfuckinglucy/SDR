#include "modulator.h"

const complex<double> i0(0, 0);
const complex<double> i1(1, 1);
const complex<double> i2(-1, 1);
const complex<double> i3(-1, -1);
const complex<double> i4(1, -1);

vector<complex<double>> modulator(int16_t* bits, int len_bits, string type){
    double I, Q;

    if (type == "QAM::2"){
        vector<complex<double>> symbols(len_bits); // Массив Символов
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
        vector<complex<double>> symbols(len_bits/2); // Массив Символов
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
        vector<complex<double>> symbols(len_bits/4); // Массив Символов
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