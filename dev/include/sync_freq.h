#ifndef SYNC_FREQ_H
#define SYNC_FREQ_H
#include "common.h"

complex<double> costas_loop(SharedData& sd, complex<double> r);
complex<double> costas_loop_16qam(SharedData& sd, complex<double> r);
vector<complex<double>> cfo_est(const vector<complex<double>> &signal, SharedData &sd);
vector<complex<double>> freq_sync(const vector<complex<double>> &signal, SharedData &sd);
vector<complex<double>> cfo_sync_shmid_cox(const vector<complex<double>> &signal, SharedData &sd);

#endif