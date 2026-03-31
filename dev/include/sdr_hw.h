#pragma once

#include "common.h"

SDRConfig SDRinit(char *usb);
std::vector<SoapySDRKwargs> find_pluto_devices();
void reconfig_sdr(SharedData &sd, SDRConfig &config);