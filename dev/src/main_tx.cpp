#include "header.h"
#include "modulator.h"

#include <GL/glew.h>
#include <SDL2/SDL.h>

#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>

#include <SoapySDR/Device.h>
#include <SoapySDR/Types.h>

// ImGui
#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_opengl3.h"

atomic<bool> g_running{false};
bool device_active = false;

thread tx_thread;

int bit_size = 1920*2;

void tx_back(SharedData& sd, SDRConfig &config) {
    while (g_running) {
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
        vector<complex<double>> symbols_UL = UpSampler(symbols.data(), symbols.size(), sd.mf_L);
        if (sd.tx_filter) {
            filter(symbols_UL.data(), symbols_UL.size(), sd.mf_L);
        }

        vector<int16_t> tx_samples(2 * symbols_UL.size());
        for (size_t i = 0; i < symbols_UL.size(); i++) {
            tx_samples[2*i]   = static_cast<int16_t>(real(symbols_UL[i]) * 16000);
            tx_samples[2*i+1] = static_cast<int16_t>(imag(symbols_UL[i]) * 16000);
        }

        size_t total = tx_samples.size() / 2;
        size_t sent = 0;

        while (sent < total) {
            size_t to_send = min(static_cast<size_t>(config.tx_mtu), total - sent);
            const void* tx_buffs[] = { tx_samples.data() + sent * 2 };
            void* rx_buffs[] = { config.rx_buffer };

            int flags = 0;
            long long timeNs = 0;

            int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
            if (sr <= 0) {
                timeNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()
                ).count();
            }

            long long tx_time = timeNs + TX_DELAY;
            flags = SOAPY_SDR_HAS_TIME;

            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, to_send, &flags, tx_time, TIMEOUT);

            sent += to_send;
        }
    }
}

int main() {
    SharedData sd;
    sd.mf_delay.resize(sd.mf_L - 1, 0.0);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        cerr << "SDL_Init Error: " << SDL_GetError() << endl;
        return -1;
    }

    struct SDRConfig config = {};
    auto sdr_devices = find_pluto_devices();
    int selected_device_index = 0;
    bool device_active = false;

    // Создаём ОДНО окно SDL
    SDL_Window* window = SDL_CreateWindow(
        "PlutoSDR Modulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        360, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::LoadIniSettingsFromDisk(io.IniFilename);
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    int modulation_idx = 0;
    const char* modulation_types[] = { "QAM::2", "QAM::4", "QAM::16" };
    bool running = true;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                running = false;
        }

        sdr_devices = find_pluto_devices();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("Device")) {
                sdr_devices = find_pluto_devices();
                for (size_t i = 0; i < sdr_devices.size(); ++i) {
                    const char* label = SoapySDRKwargs_get(&sdr_devices[i], "label");
                    const char* uri = SoapySDRKwargs_get(&sdr_devices[i], "uri");
                    bool is_selected = (static_cast<size_t>(selected_device_index) == i);
                    if (ImGui::MenuItem(label, nullptr, is_selected)) {
                        selected_device_index = static_cast<int>(i);
                        lock_guard<mutex> lock(sd.mtx);
                        sd.selected_uri = uri ? string(uri) : "";
                    }
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        ImGui::SetNextWindowPos(ImVec2(10, 30));
        ImGui::SetNextWindowSize(ImVec2(360, 200));
        ImGui::Begin("TX Control", nullptr, ImGuiWindowFlags_NoCollapse);

        ImGui::Combo("Modulation", &modulation_idx, modulation_types, IM_ARRAYSIZE(modulation_types));
        sd.modulation_index = modulation_idx;

        int L = sd.mf_L;
            if (ImGui::SliderInt("L", &L, 1, 100)) {
                lock_guard<mutex> lock(sd.mtx);
                sd.mf_L = L;
            }
        
        ImGui::Checkbox("TX Filter", &sd.tx_filter);
        
        if (!g_running) {
            if (ImGui::Button("Start TX")) {
                if (sd.selected_uri.empty()) {
                    if (!sdr_devices.empty()) {
                        sd.selected_uri = SoapySDRKwargs_get(&sdr_devices[0], "uri");
                    } else {
                        cerr << "No PlutoSDR devices found!" << endl;
                        ImGui::Text("No devices found!");
                    }
                }
                if (!sd.selected_uri.empty()) {
                    config = SDRinit(const_cast<char*>(sd.selected_uri.c_str()), sd);
                    if (config.sdr) {
                        g_running = true;
                        device_active = true;
                        tx_thread = thread(tx_back, ref(sd), ref(config));
                    }
                }
            }
        } else {
            if (ImGui::Button("Stop TX")) {
                g_running = false;
                if (tx_thread.joinable()) {
                    tx_thread.join();
                    device_active = false;
                }
            }
        }

        if (ImGui::Button("Exit")) {
            running = false;
            g_running = false;
            if (tx_thread.joinable()) tx_thread.join();
        }

        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(10, 240));
        ImGui::SetNextWindowSize(ImVec2(350, 200));
        ImGui::Begin("Control Panel", nullptr, ImGuiWindowFlags_NoCollapse);

        float tx_gain = sd.tx_gain;
        if (ImGui::SliderFloat("TX Gain", &tx_gain, -90, 40)) {
            sd.tx_gain = tx_gain;
            if (device_active && config.sdr) {
                SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, 0, tx_gain);
            }
        }

        int freq = sd.freq;
        if (ImGui::SliderInt("Carrier Freq", &freq, 200000000, 900000000)) {
            sd.freq = freq;
            if (device_active && config.sdr) {
                SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, freq, nullptr);
            }
        }

        ImGui::End();

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(window);
    }

    if (tx_thread.joinable()) tx_thread.join();
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