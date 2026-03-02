#ifndef SDR_HW_H
#define SDR_HW_H
#include "common.h"

SDRConfig SDRinit(char *usb, struct SharedData &sd);
vector<SoapySDRKwargs> find_pluto_devices();
void reconfig_sdr(SharedData &sd, SDRConfig &config);

#endif