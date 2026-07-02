#include "ofdm_core.hpp"

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fftw3.h>

bool is_guard(int k, int N)
{
    int nyq = N / 2;

    if (k >= nyq && k <= nyq + 27)
        return true;

    if (k >= nyq - 27 && k <= nyq - 1)
        return true;

    if (k == 0)
        return true;

    return false;
}

void update_pilots(SharedData &sd)
{
    sd.ofdmcfg.pilot_idx.clear();
    int N = sd.ofdmcfg.N;
    int num = sd.ofdmcfg.num_pilots;

    if (num < 2 || N <= 0)
        return;

    int nyq = N / 2;

    int left_start = 1;
    int left_end = nyq - 28;
    int right_start = nyq + 28;
    int right_end = N - 1;

    int num_left = num / 2;
    int num_right = num - num_left;

    auto distribute = [&](int start, int end, int count)
    {
        if (count <= 0)
            return;
        if (count == 1)
        {
            sd.ofdmcfg.pilot_idx.push_back((start + end) / 2);
            return;
        }
        float step = static_cast<float>(end - start) / (count - 1);
        for (int i = 0; i < count; ++i)
        {
            sd.ofdmcfg.pilot_idx.push_back(static_cast<int>(start + i * step + 0.5f));
        }
    };

    distribute(left_start, left_end, num_left);
    distribute(right_start, right_end, num_right);

    std::sort(sd.ofdmcfg.pilot_idx.begin(), sd.ofdmcfg.pilot_idx.end());
    sd.ofdmcfg.pilot_idx.erase(std::unique(sd.ofdmcfg.pilot_idx.begin(), sd.ofdmcfg.pilot_idx.end()), sd.ofdmcfg.pilot_idx.end());
}

std::vector<std::complex<float>> insert_pilots(const std::vector<std::complex<float>> &symbols, SharedData &sd)
{
    const int N = sd.ofdmcfg.N;
    const auto &pilots = sd.ofdmcfg.pilot_idx;

    std::vector<uint8_t> subcarrier_map(N, 0);

    for (int k = 0; k < N; ++k)
        if (is_guard(k, N))
            subcarrier_map[k] = 1;

    for (int p : pilots)
        if (p >= 0 && p < N)
            subcarrier_map[p] = 2;

    int usable = 0;
    for (int k = 0; k < N; ++k)
        if (!is_guard(k, N))
            usable++;

    usable -= pilots.size();
    if (usable <= 0)
        return {};

    size_t num_ofdm = (symbols.size() + usable - 1) / usable;

    std::vector<std::complex<float>> out;
    out.resize(num_ofdm * N, { 0.0f, 0.0f });

    size_t data_ptr = 0;
    size_t out_ptr = 0;

    for (size_t s = 0; s < num_ofdm; ++s)
    {
        for (int k = 0; k < N; ++k)
        {
            uint8_t type = subcarrier_map[k];

            if (type == 1)
            {
                out_ptr++;
                continue;
            }

            if (type == 2)
                out[out_ptr++] = { 1.0f, 0.0f };

            else
            {
                if (data_ptr < symbols.size())
                    out[out_ptr++] = symbols[data_ptr++];
                else
                    out_ptr++;
            }
        }
    }

    return out;
}

std::vector<std::complex<float>> ofdm_modulator(const std::vector<std::complex<float>> &freq_symbols, SharedData &sd)
{
    const int N = sd.ofdmcfg.N;
    const int CP = sd.ofdmcfg.CP;
    if (freq_symbols.size() % N != 0)
        return {};

    std::vector<std::complex<float>> ofdm_signal;
    ofdm_signal.reserve(freq_symbols.size() + (freq_symbols.size() / N) * CP);

    size_t ptr = 0;
    while (ptr < freq_symbols.size())
    {
        for (int i = 0; i < N; ++i)
        {
            sd.fftplans.in_ifft[i][0] = freq_symbols[ptr].real();
            sd.fftplans.in_ifft[i][1] = freq_symbols[ptr].imag();
            ptr++;
        }

        fftwf_execute(sd.fftplans.plan_ifft);

        std::vector<std::complex<float>> time_sym(N);

        for (int i = 0; i < N; ++i)
            time_sym[i] = { static_cast<float>(sd.fftplans.out_ifft[i][0]) / N, static_cast<float>(sd.fftplans.out_ifft[i][1]) / N };

        time_sym.insert(time_sym.begin(), time_sym.end() - CP, time_sym.end());
        ofdm_signal.insert(ofdm_signal.end(), time_sym.begin(), time_sym.end());
    }

    return ofdm_signal;
}

std::vector<std::complex<float>> FFT_ofdm(const std::vector<std::complex<float>> &signal, SharedData &sd)
{
    const int Pl_len = sd.ofdmcfg.N;
    const int CP_len = sd.ofdmcfg.CP;
    const int N = Pl_len + CP_len;

    if (signal.size() < static_cast<size_t>(N) || signal.size() > 1000000)
        return {};

    const float norm = 1.0f / Pl_len;

    std::vector<std::complex<float>> ofdm_signal;
    size_t num_symbols = signal.size() / N;

    ofdm_signal.resize(num_symbols * Pl_len);

    for (size_t s = 0; s < num_symbols; ++s)
    {
        size_t offset = s * N;

        memcpy(sd.fftplans.in_fft, &signal[offset + CP_len], Pl_len * sizeof(std::complex<float>));

        fftwf_execute(sd.fftplans.plan_fft);

        std::complex<float> *out_ptr = &ofdm_signal[s * Pl_len];

        for (int j = 0; j < Pl_len; ++j)
            out_ptr[j] = { sd.fftplans.out_fft[j][0] * norm, sd.fftplans.out_fft[j][1] * norm };
    }

    return ofdm_signal;
}

std::vector<std::complex<float>> ofdm_equalize(const std::vector<std::complex<float>> &signal, SharedData &sd)
{
    int N;
    std::vector<uint8_t> pilots;
    {
        std::lock_guard<std::mutex> lock(sd.mtx);
        N = sd.ofdmcfg.N;
        pilots = sd.ofdmcfg.pilot_idx;
    }

    float accumulated_phase = 0;

    if (pilots.empty() || signal.empty() || N <= 0)
        return {};
    if (signal.size() < static_cast<size_t>(N))
        return {};

    std::complex<float> known_pilot = { 1.0, 0.0 };

    std::vector<bool> is_pilot(N, false);
    for (auto p : pilots)
        if (p < N)
            is_pilot[p] = true;

    std::vector<std::complex<float>> sym(N), H(N), equalized(N);
    std::vector<std::complex<float>> result;

    int n_symbols = signal.size() / N;
    result.reserve(n_symbols * (N - pilots.size()));

    for (size_t i = 0; i + N <= signal.size(); i += N)
    {
        std::copy(signal.begin() + i, signal.begin() + i + N, sym.begin());

        for (auto k : pilots)
            H[k] = sym[k] / known_pilot;

        for (size_t p = 0; p + 1 < pilots.size(); ++p)
        {
            int k1 = pilots[p], k2 = pilots[p + 1];
            float mag1 = std::abs(H[k1]), mag2 = std::abs(H[k2]);
            float ph1 = std::arg(H[k1]), ph2 = std::arg(H[k2]);

            float dphi = ph2 - ph1;
            if (dphi > M_PI)
                dphi -= 2 * M_PI;
            if (dphi < -M_PI)
                dphi += 2 * M_PI;

            for (int k = k1 + 1; k < k2; ++k)
            {
                float a = float(k - k1) / float(k2 - k1);
                H[k] = std::polar(mag1 + a * (mag2 - mag1), ph1 + a * dphi);
            }
        }

        for (int k = 0; k < pilots.front(); ++k)
            if (!is_guard(k, N))
                H[k] = H[pilots.front()];

        for (int k = pilots.back() + 1; k < N; ++k)
            if (!is_guard(k, N))
                H[k] = H[pilots.back()];

        for (int k = 0; k < N; ++k)
            equalized[k] = (std::abs(H[k]) > 1e-12f) ? sym[k] / H[k] : sym[k];

        float cpe = 0;
        for (auto k : pilots)
            cpe += std::arg(equalized[k] / known_pilot);

        cpe /= pilots.size();
        accumulated_phase += cpe;

        std::complex<float> rot = std::exp(std::complex<float>(0, -accumulated_phase));
        for (int k = 0; k < N; ++k)
            if (!is_guard(k, N))
                equalized[k] *= rot;

        for (int k = 0; k < N; ++k)
        {
            if (is_guard(k, N) || is_pilot[k])
                continue;
            result.push_back(equalized[k]);
        }
    }

    return result;
}
