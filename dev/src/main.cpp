#include "header.h"
#include "modulator.h"

constexpr size_t N_BUFFERS = 100000;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

int main(int argc, char *argv[]){
    if (argc < 3) {
        printf("Usage: %s <pluto_addr> <tx|rx>\n", argv[0]);
        return -1;
    }
    srand(time(0));
    struct SDRConfig config = SDRinit(argv[1]);

    int len_bits = 1600000;
    int L = 10;
    
    const int16_t barker13[13] = {1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1};
    int16_t *bits = (int16_t*)malloc(len_bits * sizeof(int16_t));

    for (int i = 0; i < 26; ++i) {
        bits[i] = barker13[i % 13];
    }

    for (int i = 26; i < len_bits; ++i) {
        bits[i] = rand() % 2;
    }
    for (auto i = 0*2; i < len_bits; i ++) bits[i] = (rand() % 2);

    vector<complex<double>> symbols = modulator(bits, len_bits, "QAM::16");
    vector<complex<double>> symbols_UL = UpSampler(symbols.data(), symbols.size(), L);
    filter(symbols_UL.data(), symbols_UL.size(), L);

    // Show_Array("bits", bits, len_bits);
    // Show_Array("psk", symbols.data(), symbols.size());
    // Show_Array("UL", symbols_UL.data(), symbols_UL.size());

    vector<int16_t> tx_samples(2 * symbols_UL.size());

    for (size_t i = 0; i < symbols_UL.size(); i++) {
        tx_samples[2*i] = (int16_t)((real(symbols_UL[i])) * 16000);  // I
        tx_samples[2*i+1] = (int16_t)((imag(symbols_UL[i])) * 16000); // Q
    }

    FILE *rx = fopen("rx.pcm", "wb"); 

    size_t total_samples = tx_samples.size();
    size_t samples_sent = 0;

    int cnt = 0;
    cout << "Send " << N_BUFFERS << " buffers:" << endl;
    for (size_t samples_sent = 0; samples_sent < symbols_UL.size(); ++samples_sent) {
        size_t to_send = min(static_cast<size_t>(config.tx_mtu),
                          symbols_UL.size() - samples_sent);

        void *rx_buffs[] = {config.rx_buffer};
        const void *tx_buffs[] = { tx_samples.data() + samples_sent * 2 };
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
        (void)sr;
        if (strcmp(argv[2], "tx") != 0){
            fwrite(rx_buffs[0], sizeof(int16_t), 2 * config.rx_mtu, rx);
        }

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        if (strcmp(argv[2], "tx") == 0){
            if (samples_sent % 520 == 0 && samples_sent != 0) {
                cnt++;
                cout << "Seconds: " << cnt << "\t" << "Buffers: " << samples_sent << endl;
            }
            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, to_send, &flags, tx_time, TIMEOUT);
            (void)st;
        }
    }

    fclose(rx);
    
    return 0;
}