#include "header.h"
#include "modulator.h"
#include <iostream>
#include <ctime>
#include <cstring>
#include <cmath>

#define _USE_MATH_DEFINES

const complex<double> i0(0, 0);
const complex<double> i1(1, 1);
const complex<double> i2(-1, 1);
const complex<double> i3(-1, -1);
const complex<double> i4(1, -1);

const map<string, complex<double>> qpsk_map = {
    {"00", i1},
    {"01", i2},
    {"11", i3},
    {"10", i4}
};

template<typename T>
void Show_Array(const char* title, T* array, int len) {
    printf("%s: ", title);
    for (int i = 0; i < len; i++) {
        cout << array[i];
    }
    printf("\n");
}

template void Show_Array<int16_t>(const char*, int16_t*, int);
template void Show_Array<complex<double>>(const char*, complex<double>*, int);

SDRConfig SDRinit(char *usb, struct SharedData &sd) {
    SDRConfig config = {};
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", usb);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");

    cout << "Opening device: " << usb << endl;

    config.sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    config.sample_rate = 1e6;
    config.carrier_freq = 734750000;

    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_RX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, config.carrier_freq, nullptr);
    SoapySDRDevice_setSampleRate(config.sdr, SOAPY_SDR_TX, 0, config.sample_rate);
    SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, config.carrier_freq, nullptr);

    int channels = 0;
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, channels, 10);
    SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, channels, -30);

    size_t rx_channels[] = {0};
    size_t tx_channels[] = {0};
    size_t channel_count = 1;

    config.rxStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, rx_channels, channel_count, nullptr);
    config.txStream = SoapySDRDevice_setupStream(config.sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, tx_channels, channel_count, nullptr);

    SoapySDRDevice_activateStream(config.sdr, config.rxStream, 0, 0, 0);
    SoapySDRDevice_activateStream(config.sdr, config.txStream, 0, 0, 0);

    config.rx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.rxStream);
    config.tx_mtu = SoapySDRDevice_getStreamMTU(config.sdr, config.txStream);

    config.tx_buff = (int16_t*)calloc(2 * config.tx_mtu, sizeof(int16_t));
    config.rx_buffer = (int16_t*)calloc(2 * config.rx_mtu, sizeof(int16_t));

    return config;
}

vector<complex<double>> UpSampler(complex<double> *symbols, int len_s, int L){
    vector<complex<double>> symbols_ups(len_s * L);
    for (size_t i = 0; i < len_s*L; i++){
        symbols_ups[i] = i0;
    }
    for (size_t i = 0; i < len_s; i ++){
        symbols_ups[i*L] = symbols[i];
    }

    return symbols_ups;
}

void filter(complex<double>* symbols_ups, int len_symbols_ups, int L) {
    if (L <= 1 || len_symbols_ups <= 0) return;

    vector<complex<double>> impulse(L, 1.0);
    vector<complex<double>> sum(len_symbols_ups, 0.0);

    for (int i = 0; i < len_symbols_ups; i++) {
        for (int j = 0; j < L && (i - j) >= 0; j++) {
            sum[i] += impulse[j] * symbols_ups[i - j];
        }
    }

    for (int i = 0; i < len_symbols_ups; i++) {
        symbols_ups[i] = sum[i];
    }
}

complex<double> mf_filter(SharedData& sd, complex<double> x) {
    if (sd.mf_L <= 1) return x;

    if (!sd.mf_init) {
        sd.mf_sum += x;
        if (sd.mf_index < sd.mf_delay.size()) {
            sd.mf_delay[sd.mf_index] = x;
        }
        sd.mf_index++;
        if (sd.mf_index >= sd.mf_L) {
            sd.mf_init = true;
        }
        return sd.mf_sum / static_cast<double>(sd.mf_index);
    } else {
        complex<double> oldest = sd.mf_delay[sd.mf_index % sd.mf_delay.size()];
        sd.mf_sum = sd.mf_sum - oldest + x;
        sd.mf_delay[sd.mf_index % sd.mf_delay.size()] = x;
        sd.mf_index++;
        return sd.mf_sum / static_cast<double>(sd.mf_L);
    }
}

void sym_sync(SharedData& sd, const vector<complex<double>>& buf){
    if (!sd.sym_sync_enabled || buf.size() < 3) return;

    double teta = (sd.BnTs / sd.Nsp) / (sd.zeta + 1.0/(4.0*sd.zeta));
    double Kp = 4.0;
    double K1 = (-4 * sd.zeta * teta) / ((1 + 2*sd.zeta*teta + teta*teta) * Kp);
    double K2 = (-4 * teta * teta) / ((1 + 2*sd.zeta*teta + teta*teta) * Kp);
    
    size_t start = (sd.ss_last_index == 0) ? 1 : sd.ss_last_index;
    if (start >= buf.size()) return;
    
    for (size_t i = start; i < buf.size() - 1; ++i) {
        double e_real = buf[i].real() * (buf[i+1].real() - buf[i-1].real());
        double e_imag = buf[i].imag() * (buf[i+1].imag() - buf[i-1].imag());
        double error = e_real + e_imag;

        sd.ss_p1 += error * K2;
        sd.ss_p2 += sd.ss_p1 + error * K1;
    }

    sd.ss_last_index = buf.size();
    sd.ss_phase = fmod(sd.ss_phase + sd.ss_p2, sd.Nsp);
    if (sd.ss_phase < 0) sd.ss_phase += sd.Nsp;
    sd.ss_offset = static_cast<int>(sd.ss_phase);
}

complex<double> costas_loop(SharedData& sd, complex<double> r){
    auto arg = polar(1.0, -sd.cl_theta_hat);

    complex<double> r_corrected = r * arg;

    double I = r_corrected.real();
    double Q = r_corrected.imag();

    double sign_I = (I > 0) ? 1.0 : (I < 0 ? -1.0 : 0.0);
    double sign_Q = (Q > 0) ? 1.0 : (Q < 0 ? -1.0 : 0.0);

    double error = sign_I * Q - sign_Q * I;

    sd.cl_integrator += error;

    sd.cl_theta_hat += sd.cl_Kp * error + sd.cl_Ki * sd.cl_integrator;

    sd.cl_theta_hat = fmod(sd.cl_theta_hat + M_PI, 2*M_PI) - M_PI;

    return r_corrected;
}