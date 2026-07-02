#include "common.hpp"
#include "fec.hpp"
#include "modulation.hpp"
#include "ofdm_core.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

bool LoadFileForTX(SharedData &sd)
{
    std::ifstream f(sd.tx_file_path, std::ios::binary | std::ios::ate);
    if (!f.is_open())
        return false;

    std::streamsize sz = f.tellg();
    if (sz <= 0)
        return false;

    f.seekg(0, std::ios::beg);
    sd.tx_file_data.resize(static_cast<size_t>(sz));
    f.read(reinterpret_cast<char *>(sd.tx_file_data.data()), sz);

    std::filesystem::path p(sd.tx_file_path);
    sd.tx_file_name = p.filename().string();
    if (sd.tx_file_name.size() > 255)
        sd.tx_file_name.resize(255);

    sd.tx_file_total_chunks = (sd.tx_file_data.size() + FILE_CHUNK_BYTES - 1) / FILE_CHUNK_BYTES;
    sd.tx_file_chunk_idx = 0;
    sd.tx_file_loaded = true;
    return true;
}

bool SaveReceivedFile(SharedData &sd)
{
    if (sd.rx_file_chunks_buf.empty())
        return false;

    std::string save_path = "image.png";
    std::ofstream f(save_path, std::ios::binary);
    if (!f.is_open())
        return false;

    f.write(reinterpret_cast<const char *>(sd.rx_file_chunks_buf.data()), static_cast<std::streamsize>(sd.rx_file_chunks_buf.size()));

    sd.rx_file_name = save_path;
    sd.rx_file_save_path = save_path;
    return true;
}

std::vector<int16_t> GenerateSignal(SharedData &sd)
{
    std::vector<int16_t> bits;
    size_t num_bits = sd.num_samples;

    switch (sd.type_of_signal)
    {
    case SignalType::Random: {
        bits.reserve(num_bits);
        for (size_t i = 0; i < num_bits; ++i)
            bits.push_back(rand() % 2);
        break;
    }

    case SignalType::Text: {
        const std::string &text = sd.tx_text;
        num_bits = text.size() * 8;
        bits.reserve(num_bits);
        for (size_t c = 0; c < text.size(); ++c)
            for (int b = 0; b < 8; ++b)
                bits.push_back((text[c] >> b) & 1);
        break;
    }

    case SignalType::File: {
        if (!sd.tx_file_loaded || sd.tx_file_data.empty())
            break;

        size_t chunk_idx = sd.tx_file_chunk_idx;
        size_t total_bytes = sd.tx_file_data.size();
        size_t offset = chunk_idx * FILE_CHUNK_BYTES;

        if (offset >= total_bytes)
            break;

        size_t chunk_size = std::min(FILE_CHUNK_BYTES, total_bytes - offset);

        std::vector<uint8_t> payload;
        payload.reserve(chunk_size);

        payload.insert(payload.end(), sd.tx_file_data.begin() + static_cast<ptrdiff_t>(offset), sd.tx_file_data.begin() + static_cast<ptrdiff_t>(offset + chunk_size));

        num_bits = payload.size() * 8;
        bits.reserve(num_bits);
        for (uint8_t byte : payload)
            for (int b = 0; b < 8; ++b)
                bits.push_back((byte >> b) & 1);
        break;
    }
    }

    std::vector<int16_t> header_bits;

    for (int i = 0; i < 8; ++i)
        header_bits.push_back((MAGIC_NUMBER >> i) & 1);

    for (int i = 0; i < 16; ++i)
        header_bits.push_back(((num_bits + 16) >> i) & 1);

    for (int i = 0; i < 4; ++i)
        header_bits.push_back((static_cast<int>(sd.type_of_modulation) >> i) & 1);

    uint8_t current_flags = 0;
    if (sd.is_first)
        current_flags |= FrameFlag::IsFirst;
    if (sd.is_last)
        current_flags |= FrameFlag::IsLast;

    for (int i = 0; i < 8; ++i)
        header_bits.push_back((current_flags >> i) & 1);

    for (int i = 0; i < 2; ++i)
        header_bits.push_back((static_cast<int>(sd.type_of_signal) >> i) & 1);

    auto crc = calculateCRC16(bits);
    bits.insert(bits.end(), crc.begin(), crc.end());

    auto header_symbols = modulator(header_bits, SignalModulation::BPSK);
    auto header = insert_pilots(header_symbols, sd);
    header = ofdm_modulator(header, sd);

    auto symbols = modulator(bits, sd.type_of_modulation);
    auto ZC = generate_zc_preamble(sd);
    auto ofdm_signal = insert_pilots(symbols, sd);
    ofdm_signal = ofdm_modulator(ofdm_signal, sd);

    std::vector<std::complex<float>> signal;
    signal.insert(signal.end(), ZC.begin(), ZC.end());
    signal.insert(signal.end(), header.begin(), header.end());
    signal.insert(signal.end(), ofdm_signal.begin(), ofdm_signal.end());

    std::vector<int16_t> out;
    out.reserve(signal.size() * 2);
    const float amplitude = 100000.0f;
    for (const auto &s : signal)
    {
        out.push_back(static_cast<int16_t>(s.real() * amplitude));
        out.push_back(static_cast<int16_t>(s.imag() * amplitude));
    }

    return out;
}

std::vector<float> Spectrum_calulations(SharedData &sd, std::vector<std::complex<float>> raw_buffer)
{
    for (size_t i = 0; i < raw_buffer.size(); i++)
    {
        sd.fftplans.in_spectre[i][0] = raw_buffer[i].real();
        sd.fftplans.in_spectre[i][1] = raw_buffer[i].imag();
    }

    fftwf_execute(sd.fftplans.plan_spectre);

    std::vector<float> spec_db(sd.fftplans.N_spec);
    for (int i = 0; i < sd.fftplans.N_spec; i++)
    {
        int shift_idx = (i + sd.fftplans.N_spec / 2) % sd.fftplans.N_spec;
        float mag_sq = sd.fftplans.out_spectre[i][0] * sd.fftplans.out_spectre[i][0] + sd.fftplans.out_spectre[i][1] * sd.fftplans.out_spectre[i][1];
        spec_db[shift_idx] = 10.0f * log10f(mag_sq + 1e-20f);
    }

    return spec_db;
}

Header parse_header(const std::vector<std::complex<float>> &symbols, SharedData &sd)
{
    Header h;
    if (symbols.size() < static_cast<size_t>(sd.ofdmcfg.N + sd.ofdmcfg.CP))
        return h;

    std::vector<std::complex<float>> header;
    header.insert(header.begin(), symbols.begin(), symbols.begin() + sd.ofdmcfg.N + sd.ofdmcfg.CP);
    header = FFT_ofdm(header, sd);
    header = ofdm_equalize(header, sd);
    std::vector<int16_t> bits = demodulator(header, SignalModulation::BPSK);

    uint8_t magic_val = 0;
    for (int i = 0; i < 8; ++i)
        if (bits[i])
            magic_val |= (1 << i);

    if (magic_val != 0x5A)
        return h;

    h.is_valid = true;

    uint16_t samples_val = 0;
    for (int i = 0; i < 16; ++i)
        if (bits[8 + i])
            samples_val |= (1 << i);
    h.num_samples = static_cast<size_t>(samples_val);

    uint8_t mod_val = 0;
    for (int i = 0; i < 4; ++i)
        if (bits[8 + 16 + i])
            mod_val |= (1 << i);
    h.modulation = static_cast<SignalModulation>(mod_val);

    uint8_t flag_val = 0;
    for (int i = 0; i < 8; ++i)
        if (bits[8 + 16 + 4 + i])
            flag_val |= (1 << i);
    h.flag = flag_val;

    uint8_t sig_val = 0;
    for (int i = 0; i < 2; ++i)
        if (bits[8 + 16 + 4 + 8 + i])
            sig_val |= (1 << i);
    h.sig_type = static_cast<SignalType>(sig_val);

    return h;
}

const char *GetModulationName(SignalModulation mod)
{
    switch (mod)
    {
    case SignalModulation::BPSK:
        return "BPSK";
    case SignalModulation::QPSK:
        return "QPSK";
    case SignalModulation::QAM16:
        return "QAM16";
    case SignalModulation::QAM64:
        return "QAM64";
    default:
        return "Unknown";
    }
}

uint16_t bits_per_sym(SignalModulation mod_type)
{
    switch (mod_type)
    {
    case SignalModulation::BPSK:
        return 1;
    case SignalModulation::QPSK:
        return 2;
    case SignalModulation::QAM16:
        return 4;
    case SignalModulation::QAM64:
        return 6;
    default:
        return 0;
    }
}

std::vector<std::complex<float>> get_reference_constellation(SignalModulation type)
{
    int bps = 1;
    if (type == SignalModulation::QPSK)
        bps = 2;
    if (type == SignalModulation::QAM16)
        bps = 4;
    if (type == SignalModulation::QAM64)
        bps = 6;

    int num_points = 1 << bps;

    std::vector<int16_t> bits;
    for (int i = 0; i < num_points; ++i)
        for (int b = 0; b < bps; ++b)
            bits.push_back((i >> b) & 1);

    return modulator(bits, type);
}

std::complex<float> find_nearest_symbol(std::complex<float> received, const std::vector<std::complex<float>> &reference)
{
    float min_dist = 1e30f;
    std::complex<float> best = reference[0];

    for (const auto &sym : reference)
    {
        float dist = norm(received - sym);
        if (dist < min_dist)
        {
            min_dist = dist;
            best = sym;
        }
    }
    return best;
}

float EVM_calculate(const std::vector<std::complex<float>> &received, const std::vector<std::complex<float>> &reference)
{
    if (received.empty() || reference.empty())
        return 100.f;

    float P_error = 0.f;
    float P_signal = 0.f;

    for (const auto &sym : received)
    {
        std::complex<float> closest = find_nearest_symbol(sym, reference);
        P_error += norm(sym - closest);
        P_signal += norm(closest);
    }

    if (P_signal < 1e-10f)
        return 100.f;

    return 100.f * sqrtf(P_error / P_signal);
}

const char *GetSignalTypeName(SignalType type)
{
    switch (type)
    {
    case SignalType::Random:
        return "Random";
    case SignalType::Text:
        return "Text";
    case SignalType::File:
        return "File";
    }
    return "Unknown";
}
