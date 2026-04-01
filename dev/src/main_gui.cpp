#include "common.h"
#include "logger.hpp"
#include "ofdm_core.h"
#include "sdr_hw.h"
#include <thread>

#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_sdl2.h"
#include "imgui.h"
#include "implot.h"
#include <GL/glew.h>
#include <SDL2/SDL.h>

int main() {
    struct SDRConfig config = {};

    auto sdr_devices = find_pluto_devices();
    int selected_device_index = 0;

    SharedData sd;

    std::thread Back, Stream;

    sd.flags.ofdm_config_changed = true;

    sd.fft.fft_in = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * sd.fft.FFT_SIZE);
    sd.fft.fft_out = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * sd.fft.FFT_SIZE);

    sd.fft.spectrum_plan =
        fftw_plan_dft_1d(sd.fft.FFT_SIZE, sd.fft.fft_in, sd.fft.fft_out, FFTW_FORWARD, FFTW_ESTIMATE);

    rebuild_ofdm_plans(sd);
    update_pilots(std::ref(sd));

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);

    SDL_Window *window = SDL_CreateWindow("Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1920, 1080,
                                          SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_SetSwapInterval(0);

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 10.f;
    style.FrameRounding = 8.f;
    style.ChildRounding = 8.f;
    style.ScrollbarRounding = 10.f;
    style.TabRounding = 8.f;
    style.WindowBorderSize = 0.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 0.0f;
    style.WindowPadding = ImVec2(10, 10);
    style.FramePadding = ImVec2(4, 4);
    style.ItemSpacing = ImVec2(8, 8);
    style.ItemInnerSpacing = ImVec2(4, 4);

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
                sd.flags.g_running = false;
            }
        }

        if (sd.flags.ofdm_config_changed) {
            if (sd.flags.g_running) {
                sd.flags.g_running = false;

                sd.flags.ofdm_eq_enabled = false;

                if (Back.joinable())
                    Back.join();
                if (Stream.joinable())
                    Stream.join();

                rebuild_ofdm_plans(sd);

                sd.flags.g_running = true;

                Back = std::thread(rx_back, std::ref(sd));
                Stream = std::thread(SDRStream, std::ref(sd), std::ref(config));
            } else {
                rebuild_ofdm_plans(sd);
            }
        }

        sdr_devices = find_pluto_devices();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_None);

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("Device")) {
                for (size_t i = 0; i < sdr_devices.size(); ++i) {
                    const char *label = SoapySDRKwargs_get(&sdr_devices[i], "label");
                    const char *uri = SoapySDRKwargs_get(&sdr_devices[i], "uri");
                    bool is_selected = (static_cast<size_t>(selected_device_index) == i);

                    if (ImGui::MenuItem(label, nullptr, is_selected)) {
                        selected_device_index = static_cast<int>(i);
                        {
                            std::lock_guard<std::mutex> lock(sd.mtx);
                            sd.dev_f.selected_uri = uri ? std::string(uri) : "";
                        }
                    }
                }
                ImGui::EndMenu();
            }

            if (ImGui::MenuItem("Start", nullptr, false, !sd.flags.g_running)) {
                std::string uri;
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    uri = sd.dev_f.selected_uri;
                }

                if (uri.empty() && !sdr_devices.empty()) {
                    uri = SoapySDRKwargs_get(&sdr_devices[0], "uri");
                }

                if (uri.empty()) {
                    logs::sdr.info("No device URI available!");
                    continue;
                }

                config = SDRinit(const_cast<char *>(uri.c_str()));

                if (!config.sdr) {
                    logs::sdr.info("Failed to initialize SDR device!");
                    continue;
                }

                sd.flags.g_running = true;
                Back = std::thread(rx_back, std::ref(sd));
                Stream = std::thread(SDRStream, std::ref(sd), std::ref(config));
            } else {
                if (ImGui::MenuItem("Stop TX", nullptr, true, sd.flags.g_running)) {
                    sd.flags.g_running = false;
                    if (Back.joinable()) {
                        Back.join();
                    }
                    if (Stream.joinable()) {
                        Stream.join();
                    }
                }
            }

            if (ImGui::MenuItem("Exit", nullptr, false, true)) {
                sd.flags.g_running = false;
                if (Back.joinable()) {
                    Back.join();
                }
                if (Stream.joinable()) {
                    Stream.join();
                }
                running = false;
            }

            bool loopback = sd.flags.loopback_flag;
            if (ImGui::Checkbox("Loopback", &loopback)) {
                sd.flags.loopback_flag = loopback;
            }

            if (sd.flags.loopback_flag) {
                ImGui::SameLine();
                if (ImGui::Button("Config")) {
                    ImGui::OpenPopup("TX Settings");
                }
            }

            if (ImGui::BeginPopup("TX Settings")) {
                ImGui::SeparatorText("TX Configuration");

                static int tx_mode = 0;

                const char *tx_modes[] = {"QAM::2",        "QAM::4",        "QAM::16",        "QAM::64",
                                          "QAM::2 + OFDM", "QAM::4 + OFDM", "QAM::16 + OFDM", "QAM::64 + OFDM"};

                if (ImGui::Combo("TX Mode", &tx_mode, tx_modes, IM_ARRAYSIZE(tx_modes))) {
                    std::lock_guard<std::mutex> lock(sd.mtx);

                    if (tx_mode < 4) {
                        sd.flags.ofdm_enabled_tx = false;
                        sd.flags.modulation_index = tx_mode;
                    } else {
                        sd.flags.ofdm_enabled_tx = true;
                        sd.flags.modulation_index = tx_mode - 4;
                    }

                    sd.flags.tx_regenerate = true;
                }

                static int tx_symbol_count = 256;

                if (ImGui::SliderInt("TX Symbols", &tx_symbol_count, 16, 4096)) {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    sd.tx_symbol_count = tx_symbol_count;
                    sd.flags.tx_regenerate = true;
                }

                if (!sd.flags.loopback_flag) {
                    ImGui::CloseCurrentPopup();
                }

                if (tx_mode >= 3) {
                    ImGui::SeparatorText("OFDM Settings");

                    int old_n = sd.ofdm.n_subcarriers;

                    if (ImGui::SliderInt("Symbol Len", &sd.ofdm.n_subcarriers, 1, 128)) {
                        std::lock_guard<std::mutex> lock(sd.mtx);
                        if (sd.ofdm.n_subcarriers != old_n) {
                            sd.flags.ofdm_config_changed = true;
                        }
                    }

                    int old_cp = sd.ofdm.cp_len;

                    if (ImGui::SliderInt("Prefix Len", &sd.ofdm.cp_len, 1, sd.ofdm.n_subcarriers / 4)) {
                        if (sd.ofdm.cp_len != old_cp) {
                            sd.flags.ofdm_config_changed = true;
                        }
                    }
                    ImGui::SliderInt("Num Pilots", &sd.ofdm.num_pilots, 1, 20);

                    if (ImGui::Button("Update Pilots"))
                        update_pilots(std::ref(sd));

                    ImGui::SliderInt("Guard DC", &sd.ofdm.guard_dc, 1, 20);
                    ImGui::SliderInt("Guard Edge", &sd.ofdm.guard_edge, 1, 20);
                }

                ImGui::EndPopup();
            }

            ImGui::EndMainMenuBar();
        }

        ImGui::Begin("Control Panel", nullptr, ImGuiWindowFlags_NoCollapse);

        ImGui::SeparatorText("Debug");
        ImGui::Text("FPS: %.1f (%.3f ms)", io.Framerate, 1000.0f / io.Framerate);
        ImGui::Text("DSP Time %f", sd.avg_time);
        ImGui::Text("Stream Time %f", sd.avg_stream_time);

        ImGui::SeparatorText("Stats");
        ImGui::Text("[BLER] Blocks: %ld", sd.bler_total_blocks);
        ImGui::Text("Errors: %ld", sd.bler_error_blocks);
        ImGui::Text("Rate %f", (sd.bler_value * 100.0));
        ImGui::Text("EVM %f", sd.EVM);
        ImGui::Text("SNR %f Db", sd.SNR_DB);

        ImGui::SeparatorText("Hamming Stats");

        ImGui::Text("Blocks processed: %lu", sd.Ham_stats.blocks_processed);
        ImGui::Text("Blocks with errors: %lu (%.2f%%)", sd.Ham_stats.blocks_with_errors,
                    sd.Ham_stats.blocks_processed > 0
                        ? 100.0f * sd.Ham_stats.blocks_with_errors / sd.Ham_stats.blocks_processed
                        : 0.0f);

        ImGui::Text("Bits corrected: %lu", sd.Ham_stats.bits_corrected);

        ImGui::Text("Uncorrectable errors: %lu", sd.Ham_stats.uncorrectable);

        if (ImGui::Button("Reset Stats")) {
            sd.Ham_stats = {};
        }

        ImGui::SeparatorText("SDR Config");

        float rx_gain = sd.rx_gain;
        if (ImGui::SliderFloat("rx gain", &rx_gain, -40, 40)) {
            sd.rx_gain = rx_gain;
            sd.flags.rx_gain_changed = true;
        }

        float tx_gain = sd.tx_gain;
        if (ImGui::SliderFloat("Tx gain", &tx_gain, -40, 89)) {
            sd.tx_gain = tx_gain;
            sd.flags.tx_gain_changed = true;
        }

        float freq = sd.freq;
        if (ImGui::SliderFloat("Carrier Freq", &freq, 200e6, 900e6, "%e")) {
            sd.freq = freq;
            sd.freq = freq;
            sd.flags.rx_freq_changed = true;
            sd.flags.tx_freq_changed = true;
        }

        float rx_bandwidth = sd.rx_bandwidth;
        if (ImGui::SliderFloat("Rx Sample Rate", &rx_bandwidth, 0.2e6, 10e6, "%e")) {
            sd.rx_bandwidth = rx_bandwidth;
            sd.flags.rx_bw_changed = true;
        }

        float tx_bandwidth = sd.tx_bandwidth;
        if (ImGui::SliderFloat("TX BandWidth", &tx_bandwidth, 0.2e6, 10e6, "%e")) {
            sd.tx_bandwidth = tx_bandwidth;
            sd.flags.tx_bw_changed = true;
        }

        ImGui::SeparatorText("OFDM Control Panel");

        ImGui::Checkbox("Ofdm receiver", &sd.flags.ofdm_enabled);

        if (sd.flags.ofdm_enabled) {
            ImGui::Checkbox("Time Sync", &sd.flags.ofdm_time_est);
            ImGui::SameLine();
            ImGui::Text("Signal Begin: %d", sd.ofdm.sig_begin);
            ImGui::SameLine();
            ImGui::SliderInt("Manual Sync", &sd.ofdm.sig_begin, 0, 1920);
            ImGui::Checkbox("Cut Begin", &sd.flags.cut_begin);
            ImGui::Checkbox("Decode Header", &sd.flags.header_dec);
            ImGui::SameLine();
            ImGui::Text("Packet Len %d", sd.ofdm_sync.packet_len);
            ImGui::Checkbox("CFO Sync", &sd.flags.cfo_est_enabled);
            ImGui::SameLine();
            ImGui::Text("CFO Est: %f", sd.ofdm_sync.cfo_estimate);
            ImGui::Checkbox("FFT", &sd.flags.ofdm_fft_enabled);
            ImGui::Checkbox("EQ", &sd.flags.ofdm_eq_enabled);
        }

        ImGui::End();

        ImGui::Begin("First TX Bits", nullptr, ImGuiWindowFlags_NoCollapse);

        int N_tx = std::min(50, static_cast<int>(sd.bits.size()) / 2);
        if (N_tx > 0) {
            ImGui::Text("Idx |   I   |   Q");
            ImGui::Separator();
            for (int i = 0; i < N_tx; ++i) {
                int16_t I = sd.bits[2 * i];
                int16_t Q = sd.bits[2 * i + 1];
                ImGui::Text("%3d | %5d | %5d", i, I, Q);
            }
        } else {
            ImGui::Text("No TX samples yet");
        }

        ImGui::End();

        ImGui::Begin("First RX Bits", nullptr, ImGuiWindowFlags_NoCollapse);

        int N_rx = std::min(50, static_cast<int>(sd.rx_bits.size()) / 2);
        if (N_rx > 0) {
            ImGui::Text("Idx |   I   |   Q");
            ImGui::Separator();
            for (int i = 0; i < N_rx; ++i) {
                if ((2 * i + 1) > 2 * N_rx || (2 * i) > 2 * N_rx)
                    break;
                int16_t I = sd.rx_bits[2 * i];
                int16_t Q = sd.rx_bits[2 * i + 1];
                ImGui::Text("%3d | %5d | %5d", i, I, Q);
            }
        } else {
            ImGui::Text("No RX samples yet");
        }

        ImGui::End();

        ImGui::Begin("Constellation and RX Scope", nullptr, ImGuiWindowFlags_NoTitleBar);

        if (ImPlot::BeginPlot("Constellation", ImVec2(600, 600))) {
            std::vector<float> plot_real, plot_imag;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                size_t limit = std::min(sd.buffer.size(), (size_t) 500);
                plot_real.reserve(sd.buffer.size() / 2);
                plot_imag.reserve(sd.buffer.size() / 2);
                for (size_t i = 0; i < limit; ++i) {
                    plot_real.push_back(sd.buffer[i].real());
                    plot_imag.push_back(sd.buffer[i].imag());
                }
            }
            if (!plot_real.empty()) {
                ImPlot::SetupAxesLimits(-2, 2, -2, 2);
                ImPlot::PlotScatter("IQ", plot_real.data(), plot_imag.data(), plot_real.size());
            }
            ImPlot::EndPlot();
        }

        ImGui::SameLine();

        if (ImPlot::BeginPlot("RX Scope", ImVec2(-1, 600))) {
            std::vector<float> scope_I, scope_Q;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                scope_I.reserve(sd.buffer.size() / 2);
                scope_Q.reserve(sd.buffer.size() / 2);
                for (const auto &val : sd.buffer) {

                    scope_I.push_back(val.real());
                    scope_Q.push_back(val.imag());
                }
            }

            if (!scope_I.empty()) {
                ImPlot::SetupAxesLimits(0, scope_I.size(), -20000, 20000);
                ImPlot::PlotLine("I", scope_I.data(), scope_I.size());
                ImPlot::PlotLine("Q", scope_Q.data(), scope_Q.size());
            } else {
                ImPlot::SetupAxesLimits(0, 100, -20000, 20000);
            }
            ImPlot::EndPlot();
        }

        ImGui::End();

        ImGui::Begin("TX Samples", nullptr, ImGuiWindowFlags_NoTitleBar);

        if (ImPlot::BeginPlot("TX Scope", ImVec2(-1, 600))) {
            std::vector<float> scope_I, scope_Q;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);

                scope_I.reserve(sd.tx_samples.size() / 2);
                scope_Q.reserve(sd.tx_samples.size() / 2);

                for (size_t i = 0; i < sd.tx_samples.size() / 2; ++i) {
                    scope_I.push_back(sd.tx_samples[2 * i]);
                    scope_Q.push_back(sd.tx_samples[2 * i + 1]);
                }
            }

            if (!scope_I.empty()) {
                ImPlot::SetupAxesLimits(0, scope_I.size(), -20000, 20000);
                ImPlot::PlotLine("I", scope_I.data(), scope_I.size());
                ImPlot::PlotLine("Q", scope_Q.data(), scope_Q.size());
            } else {
                ImPlot::SetupAxesLimits(0, 100, -20000, 20000);
            }
            ImPlot::EndPlot();
        }

        ImGui::End();

        ImGui::Begin("RX Samples", nullptr, ImGuiWindowFlags_NoTitleBar);

        if (ImPlot::BeginPlot("TX Scope", ImVec2(-1, 600))) {
            std::vector<float> scope_I, scope_Q;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);

                scope_I.reserve(sd.buffer_without_dsp.size());
                scope_Q.reserve(sd.buffer_without_dsp.size());

                for (size_t i = 0; i < sd.buffer_without_dsp.size(); ++i) {
                    scope_I.push_back(sd.buffer_without_dsp[i].real());
                    scope_Q.push_back(sd.buffer_without_dsp[i].imag());
                }
            }

            if (!scope_I.empty()) {
                ImPlot::SetupAxesLimits(0, scope_I.size(), -20000, 20000);
                ImPlot::PlotLine("I", scope_I.data(), scope_I.size());
                ImPlot::PlotLine("Q", scope_Q.data(), scope_Q.size());
            } else {
                ImPlot::SetupAxesLimits(0, 100, -20000, 20000);
            }
            ImPlot::EndPlot();
        }

        ImGui::End();

        ImGui::Begin("FFT", nullptr, ImGuiWindowFlags_NoTitleBar);
        if (ImPlot::BeginPlot("Spectre", ImVec2(-1, 400))) {
            std::vector<float> local_mag;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                if (sd.flags.fft_ready) {
                    local_mag = sd.fft.fft_magnitude;
                }
            }
            if (!local_mag.empty()) {
                std::vector<float> shifted(local_mag.size());
                size_t half = local_mag.size() / 2;
                for (size_t i = 0; i < half; i++) {
                    shifted[i] = local_mag[i + half];
                    shifted[i + half] = local_mag[i];
                }
                ImPlot::PlotLine("Magnitude", shifted.data(), shifted.size());
            }
            ImPlot::EndPlot();
        }

        ImGui::End();

        ImGui::Begin("Other", nullptr, ImGuiWindowFlags_NoTitleBar);

        if (ImPlot::BeginPlot("Timing Offsets", ImVec2(-1, 300))) {
            ImPlot::PlotLine("Offset", sd.timing_offsets.data(), sd.timing_offsets.size());
            ImPlot::EndPlot();
        }
        ImGui::End();

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            SDL_GL_MakeCurrent(window, gl_context);
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            SDL_GL_MakeCurrent(window, gl_context);
        }
        SDL_GL_SwapWindow(window);
    }

    if (sd.flags.g_running) {
        sd.flags.g_running = false;
        if (Back.joinable()) {
            Back.join();
        }
    }

    if (config.sdr) {
        SoapySDRDevice_deactivateStream(config.sdr, config.rxStream, 0, 0);
        SoapySDRDevice_deactivateStream(config.sdr, config.txStream, 0, 0);
        SoapySDRDevice_closeStream(config.sdr, config.rxStream);
        SoapySDRDevice_closeStream(config.sdr, config.txStream);
        SoapySDRDevice_unmake(config.sdr);
        config.sdr = nullptr;
    }

    if (sd.fft.ofdm_ifft_plan)
        fftw_destroy_plan(sd.fft.ofdm_ifft_plan);
    if (sd.fft.ifft_in)
        fftw_free(sd.fft.ifft_in);
    if (sd.fft.ifft_out)
        fftw_free(sd.fft.ifft_out);
    if (sd.fft.ofdm_fft_plan)
        fftw_destroy_plan(sd.fft.ofdm_fft_plan);
    if (sd.fft.ofdm_rx_in)
        fftw_free(sd.fft.ofdm_rx_in);
    if (sd.fft.ofdm_rx_out)
        fftw_free(sd.fft.ofdm_rx_out);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}