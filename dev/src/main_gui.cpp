#include "header.h"
#include "modulator.h"

#include <GL/glew.h>
#include <SDL2/SDL.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <atomic>

// ImGui
#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_opengl3.h"

std::atomic<bool> g_running{true};
const uint bit_size = 20000;

struct SharedData{
    std::vector<int16_t> bits;
    std::vector<std::complex<double>> symbols;
    std::mutex mtx;
};

void Backend(SharedData& sd) {
    sd.bits.resize(bit_size);
    while (g_running) {
        const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
        for (int i = 0; i < 26; ++i) {
            sd.bits[i] = barker13[i % 13];
        }
        for (int i = 26; i < bit_size; ++i) {
            sd.bits[i] = rand() % 2;
        }

        auto symbols = modulator(sd.bits.data(), bit_size, "QAM::4");

        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            sd.symbols = std::move(symbols);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main(int argc, char *argv[]) {
    SharedData shared_data;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "PlutoSDR Modulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);

    // Инициализация ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    std::thread Back(Backend, std::ref(shared_data));

    // Главный цикл
    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                running = false;
            if (event.type == SDL_QUIT) {
                running = false;
                g_running = false;
            }
        }

        // Начало ImGui-фрейма
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Окно: входные биты
        ImGui::Begin("Input Bits");
        {
            std::lock_guard<std::mutex> lock(shared_data.mtx);
            for (int i = 0; i < std::min(100, (int)shared_data.bits.size()); ++i) {
                ImGui::SameLine();
                ImGui::Text("%d", (int)shared_data.bits[i]);
                if ((i + 1) % 20 == 0) ImGui::Text("");
            }
        }
        ImGui::End();

        // Окно: управление
        ImGui::Begin("Control");
        if (ImGui::Button("Regenerate Bits")) {
            for (int i = 26; i < bit_size; ++i) {
                shared_data.bits[i] = rand() % 2;
            }
        }
        ImGui::Text("Total bits: %d", bit_size);
        ImGui::End();

        if (ImPlot::BeginPlot("Modulated Signal")) {
            std::vector<double> real, imag;
            {
                std::lock_guard<std::mutex> lock(shared_data.mtx);
                real.reserve(shared_data.symbols.size());
                imag.reserve(shared_data.symbols.size());
                for (const auto& c : shared_data.symbols) {
                    real.push_back(c.real());
                    imag.push_back(c.imag());
                }
            }
            ImPlot::PlotLine("I", real.data(), real.size());
            ImPlot::PlotLine("Q", imag.data(), imag.size());
            ImPlot::EndPlot();
        }

        // Рендеринг
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
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