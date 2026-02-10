#include "header.h"
#include "modulator.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <atomic>

int bit_size = 100000;

void tx_back(SharedData& sd){
    string uri;
    uri = sd.tx_uri;
    struct SDRConfig config = SDRinit(const_cast<char*>(uri.c_str()), sd);
    sd.bits.resize(bit_size);
    const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
    for (int i = 0; i < 26; ++i) {
        sd.bits[i] = barker13[i % 13];
    }
    for (int i = 26; i < bit_size; ++i) {
        sd.bits[i] = rand() % 2;
    }

    string mod_type;

    if (sd.modulation_index == 0){
        mod_type = "QAM::2";
    } else if (sd.modulation_index == 2){
        mod_type = "QAM::16";
    } else {
        mod_type = "QAM::4";
    }

    vector<complex<double>> symbols = modulator(sd.bits.data(), bit_size, mod_type);
    vector<complex<double>> symbols_UL = UpSampler(symbols.data(), symbols.size(), 10);
    // filter(symbols_UL.data(), symbols_UL.size(), L);
    
    vector<int16_t> tx_samples(2 * symbols_UL.size());
    
    for (size_t i = 0; i < symbols_UL.size(); i++) {
        tx_samples[2*i] = (int16_t)((real(symbols_UL[i])) * 16000);  // I
        tx_samples[2*i+1] = (int16_t)((imag(symbols_UL[i])) * 16000); // Q
    }
    
    int cnt = 0;
    cout << "Send " << N_BUFFERS << " buffers:" << endl;
    for (size_t samples_sent = 0; samples_sent < symbols_UL.size(); ++samples_sent) {
        if (!sd.tx_running){
            break;
        }
        size_t to_send = min(static_cast<size_t>(config.tx_mtu),
        symbols_UL.size() - samples_sent);
        
        void *rx_buffs[] = {config.rx_buffer};
        const void *tx_buffs[] = { tx_samples.data() + samples_sent * 2 };
        int flags = 0;
        long long timeNs = 0;
        
        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
        
        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;
        
        if (samples_sent % 520 == 0 && samples_sent != 0) {
            cnt++;
        }
        int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, to_send, &flags, tx_time, TIMEOUT);
        (void)st;
    }
}