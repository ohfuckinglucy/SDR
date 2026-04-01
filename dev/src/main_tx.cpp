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

std::thread tx_thread;

int main() {
    SharedData sd;

    sd.flags.loopback_flag = true;

    struct SDRConfig config = {};
    auto sdr_devices = find_pluto_devices();
    int selected_device_index = 0;

    sd.flags.ofdm_config_changed = true;
    rebuild_ofdm_plans(sd);

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);

    SDL_Window *window = SDL_CreateWindow("Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 360, 720,
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
            if (event.type == SDL_QUIT)
                running = false;
        }

        if (sd.flags.ofdm_config_changed) {
            if (sd.flags.g_running) {
                sd.flags.g_running = false;

                if (tx_thread.joinable())
                    tx_thread.join();

                rebuild_ofdm_plans(sd);

                sd.flags.g_running = true;

                tx_thread = std::thread(SDRStream, std::ref(sd), std::ref(config));
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
                sdr_devices = find_pluto_devices();
                for (size_t i = 0; i < sdr_devices.size(); ++i) {
                    const char *label = SoapySDRKwargs_get(&sdr_devices[i], "label");
                    const char *uri = SoapySDRKwargs_get(&sdr_devices[i], "uri");
                    bool is_selected = (static_cast<size_t>(selected_device_index) == i);
                    if (ImGui::MenuItem(label, nullptr, is_selected)) {
                        selected_device_index = static_cast<int>(i);
                        std::lock_guard<std::mutex> lock(sd.mtx);
                        sd.dev_f.selected_uri = uri ? std::string(uri) : "";
                    }
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        ImGui::Begin("TX Control", nullptr, ImGuiWindowFlags_NoCollapse);

        static int tx_mode = 0;

        const char *tx_modes[] = {"QAM::2",        "QAM::4",        "QAM::16",        "QAM::64",
                                  "QAM::2 + OFDM", "QAM::4 + OFDM", "QAM::16 + OFDM", "QAM::64 + OFDM"};

        if (tx_mode >= 4) {
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

        if (!sd.flags.g_running) {
            if (ImGui::Button("Start TX")) {
                if (sd.dev_f.selected_uri.empty()) {
                    if (!sdr_devices.empty()) {
                        sd.dev_f.selected_uri = SoapySDRKwargs_get(&sdr_devices[0], "uri");
                    } else {
                        logs::sdr.warn("o PlutoSDR devices found!");
                        ImGui::Text("No devices found!");
                    }
                }
                if (!sd.dev_f.selected_uri.empty()) {
                    config = SDRinit(const_cast<char *>(sd.dev_f.selected_uri.c_str()));
                    if (config.sdr) {
                        sd.flags.g_running = true;
                        tx_thread = std::thread(SDRStream, std::ref(sd), std::ref(config));
                    }
                }
            }
        } else {
            if (ImGui::Button("Stop TX")) {
                sd.flags.g_running = false;
                if (tx_thread.joinable()) {
                    tx_thread.join();
                    sd.flags.g_running = false;
                }
            }
        }

        if (ImGui::Button("Exit")) {
            running = false;
            sd.flags.g_running = false;
            if (tx_thread.joinable())
                tx_thread.join();
        }

        ImGui::End();

        ImGui::Begin("Control Panel", nullptr, ImGuiWindowFlags_NoCollapse);

        float tx_gain = sd.tx_gain;
        if (ImGui::SliderFloat("TX Gain", &tx_gain, 0, 89)) {
            if (sd.flags.g_running) {
                sd.tx_gain = tx_gain;
                sd.flags.tx_gain_changed = true;
            }
        }

        float freq = sd.freq;
        if (ImGui::SliderFloat("Carrier Freq", &freq, 200e6, 900e6, "%e")) {
            sd.freq = freq;
            if (sd.flags.g_running && config.sdr) {
                sd.freq = freq;
                sd.flags.tx_freq_changed = true;
            }
        }

        float tx_bandwidth = sd.tx_bandwidth;
        if (ImGui::SliderFloat("TX BandWidth", &tx_bandwidth, 0.2e6, 10e6, "%e")) {
            if (sd.flags.g_running) {
                sd.tx_bandwidth = tx_bandwidth;
                sd.flags.tx_bw_changed = true;
            }
        }

        ImGui::End();

        ImGui::Begin("First bits", nullptr, ImGuiWindowFlags_NoCollapse);

        int N = std::min(50, static_cast<int>(sd.bits.size()) / 2);
        if (N > 0) {
            ImGui::Text("Idx |   I   |   Q");
            ImGui::Separator();
            for (int i = 0; i < N; ++i) {
                int16_t I = sd.bits[2 * i];
                int16_t Q = sd.bits[2 * i + 1];
                ImGui::Text("%3d | %5d | %5d", i, I, Q);
            }
        } else {
            ImGui::Text("No TX samples yet");
        }

        ImGui::End();

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(window);
    }

    if (tx_thread.joinable())
        tx_thread.join();
    if (config.sdr) {
        SoapySDRDevice_deactivateStream(config.sdr, config.rxStream, 0, 0);
        SoapySDRDevice_deactivateStream(config.sdr, config.txStream, 0, 0);
        SoapySDRDevice_closeStream(config.sdr, config.rxStream);
        SoapySDRDevice_closeStream(config.sdr, config.txStream);
        SoapySDRDevice_unmake(config.sdr);
        config.sdr = nullptr;
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}