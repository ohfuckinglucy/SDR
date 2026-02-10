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
const int L = 10;
constexpr size_t N_BUFFERS = 100000;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

struct SharedData{
    std::vector<int16_t> bits;
    std::vector<std::complex<double>> tx;
    std::mutex mtx;
    char* usb;
    char* type;
    std::vector<int16_t> buffer;
};

void Backend(SharedData& sd) {
    struct SDRConfig config = SDRinit(sd.usb);
    sd.bits.resize(bit_size);
    const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
    for (int i = 0; i < 26; ++i) {
        sd.bits[i] = barker13[i % 13];
    }
    for (int i = 26; i < bit_size; ++i) {
        sd.bits[i] = rand() % 2;
    }

    std::vector<std::complex<double>> symbols = modulator(sd.bits.data(), bit_size, "QAM::4");
    std::vector<std::complex<double>> symbols_UL = UpSampler(symbols.data(), symbols.size(), L);
    filter(symbols_UL.data(), symbols_UL.size(), L);
    
    std::vector<int16_t> tx_samples(2 * symbols_UL.size());
    
    for (size_t i = 0; i < symbols_UL.size(); i++) {
        tx_samples[2*i] = (int16_t)((real(symbols_UL[i])) * 16000);  // I
        tx_samples[2*i+1] = (int16_t)((imag(symbols_UL[i])) * 16000); // Q
    }
    
    int cnt = 0;
    cout << "Send " << N_BUFFERS << " buffers:" << endl;
    for (size_t samples_sent = 0; samples_sent < symbols_UL.size(); ++samples_sent) {
        if (g_running == 0){
            exit(1);
        }
        size_t to_send = std::min(static_cast<size_t>(config.tx_mtu),
        symbols_UL.size() - samples_sent);
        
        void *rx_buffs[] = {config.rx_buffer};
        const void *tx_buffs[] = { tx_samples.data() + samples_sent * 2 };
        int flags = 0;
        long long timeNs = 0;
        
        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
        (void)sr;
        
        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;
        
        if (strcmp(sd.type, "tx") == 0){
            if (samples_sent % 520 == 0 && samples_sent != 0) {
                cnt++;
            }
            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, to_send, &flags, tx_time, TIMEOUT);
            (void)st;
        }

        if (strcmp(sd.type, "rx") != 0){
            continue;
        }
        
        int16_t* data_ptr = static_cast<int16_t*>(config.rx_buffer);
        
        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            for(int i = 0; i < config.rx_mtu*2; i ++)
                sd.buffer.push_back(data_ptr[i]); 
        }
        if (sd.buffer.size() > 50000){
            sd.buffer.erase(sd.buffer.begin(), sd.buffer.begin() + sd.buffer.size() - 50000);
        }

        
    }
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Usage: %s <pluto_addr> <tx|rx>\n", argv[0]);
        return -1;
    }
    SharedData shared_data;

    shared_data.usb = argv[1];
    shared_data.type = argv[2];

    std::thread Back(Backend, std::ref(shared_data));

    if (strcmp(shared_data.type, "rx") == 0){

        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
            std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
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
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    
        ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
        ImGui_ImplOpenGL3_Init("#version 330");
    
    
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

            ImVec2 plotsize(1600, 600);

            std::vector<double> plot_real, plot_imag;
            {
                std::lock_guard<std::mutex> lock(shared_data.mtx);
                if (!shared_data.buffer.empty()) {
                    for (size_t i = 0; i + 1 < shared_data.buffer.size(); i += 2) {
                        plot_real.push_back(static_cast<double>(shared_data.buffer[i]));
                        plot_imag.push_back(static_cast<double>(shared_data.buffer[i+1]));
                    }
                }
            }

            if (ImPlot::BeginPlot("Scatter Plot", plotsize)){
                if (!plot_real.empty()) {
                    ImPlot::PlotScatter("Plot", plot_real.data(), plot_imag.data(), plot_real.size());
                }
                ImPlot::EndPlot();
            }
    
            if (ImPlot::BeginPlot("Modulated Signal", plotsize)) {
                if (!plot_real.empty()) {
                    ImPlot::PlotLine("I", plot_real.data(), plot_real.size());
                    ImPlot::PlotLine("Q", plot_imag.data(), plot_imag.size());
                }
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
    }
    Back.join();

    return 0;
}