#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex>
#include <map>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

vector<complex<double>> modulator(vector<int16_t> bits, int len_bits, string type);
void update_pilots(struct SharedData& sd);