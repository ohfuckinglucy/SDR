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

bool rx_running = false;
bool tx_running = false;
bool g_running = true;

thread rx_thread;
thread tx_thread;

vector<SoapySDRKwargs> find_pluto_devices() {
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    
    size_t length;
    SoapySDRKwargs *devices = SoapySDRDevice_enumerate(&args, &length);
    
    vector<SoapySDRKwargs> result(devices, devices + length);
    
    SoapySDRKwargs_clear(&args);
    
    return result;
}

int main() {
    SharedData sd;
    sd.mf_delay.resize(sd.mf_L - 1, 0.0);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        cerr << "SDL_Init Error: " << SDL_GetError() << endl;
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "PlutoSDR Modulatora", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);

    // Инициализация ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui::LoadIniSettingsFromDisk(io.IniFilename);

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    ImVec2 plotsize(1600, 600);

    bool tx_running = false;
    thread tx_thread;

    int modulation_idx = 0;
    int symbol_length = 1;
    const char* modulation_types[] = { "QAM::2", "QAM::4", "QAM::16"};
    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) running = false;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowSize(ImVec2(350, 250), ImGuiCond_Once);
        ImGui::Begin("TX Control");

        ImGui::Combo("Modulation", &modulation_idx, modulation_types, IM_ARRAYSIZE(modulation_types));

        ImGui::SliderInt("Symbol length", &symbol_length, 1, 50);

        if (ImGui::BeginCombo("TX Device", sd.tx_uri.empty() ? "None" : sd.tx_uri.c_str())) {
            auto sdr_devices = find_pluto_devices();
            for (size_t i = 0; i < sdr_devices.size(); ++i) {
                const char* label = SoapySDRKwargs_get(&sdr_devices[i], "label");
                const char* uri   = SoapySDRKwargs_get(&sdr_devices[i], "uri");
                bool is_selected = (uri && sd.tx_uri == uri);
                if (ImGui::Selectable(label, is_selected)) {
                    lock_guard<mutex> lock(sd.mtx);
                    sd.tx_uri = uri ? string(uri) : "";
                }
            }
            ImGui::EndCombo();
        }

        if (!sd.tx_running) {
            if (ImGui::Button("Start TX")) {
                sd.modulation_index = modulation_idx;
                sd.tx_running = true;
                tx_thread = thread(tx_back, ref(sd));
            }
        } else {
            if (ImGui::Button("Stop TX")) {
                sd.tx_running = false;
                if (tx_thread.joinable()) tx_thread.join();
            }
        }

        if (ImGui::Button("Exit")) {
            running = false;
            g_running = false;
            sd.tx_running = false;
            if (tx_thread.joinable()) tx_thread.join();
        }

        ImGui::End();

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
        }
        SDL_GL_SwapWindow(window);
    }

    // Очистка
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}