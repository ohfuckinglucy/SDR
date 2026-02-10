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
#include <atomic>

#include <SoapySDR/Device.h>
#include <SoapySDR/Types.h>

// ImGui
#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_opengl3.h"

atomic<bool> g_running{true};
const uint bit_size = 20000;
const int L = 10;
constexpr size_t N_BUFFERS = 100000;
constexpr long long TIMEOUT = 400000;
constexpr long long TX_DELAY = 4000000;

void Backend(SharedData& sd) {
    string uri;
    {
        lock_guard<mutex> lock(sd.mtx);
        uri = sd.selected_uri;
    }
    struct SDRConfig config = SDRinit(const_cast<char*>(uri.c_str()), sd);
    sd.bits.resize(bit_size);
    const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
    for (int i = 0; i < 26; ++i) {
        sd.bits[i] = barker13[i % 13];
    }
    for (int i = 26; i < bit_size; ++i) {
        sd.bits[i] = rand() % 2;
    }

    vector<complex<double>> symbols = modulator(sd.bits.data(), bit_size, "QAM::16");
    vector<complex<double>> symbols_UL = UpSampler(symbols.data(), symbols.size(), L);
    // filter(symbols_UL.data(), symbols_UL.size(), L);
    
    vector<int16_t> tx_samples(2 * symbols_UL.size());
    
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
        size_t to_send = min(static_cast<size_t>(config.tx_mtu),
        symbols_UL.size() - samples_sent);
        
        void *rx_buffs[] = {config.rx_buffer};
        const void *tx_buffs[] = { tx_samples.data() + samples_sent * 2 };
        int flags = 0;
        long long timeNs = 0;
        
        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);
        
        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;
        
        if (strcmp(sd.type, "rx") == 0){
            if (samples_sent % 520 == 0 && samples_sent != 0) {
                cnt++;
            }
            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, to_send, &flags, tx_time, TIMEOUT);
            (void)st;
        }

        if ((strcmp(sd.type, "rx") != 0) || (sr <= 0)) continue;

        int16_t* data_ptr = static_cast<int16_t*>(config.rx_buffer);
        
        {
            lock_guard<mutex> lock(sd.mtx);

            for (int i = 0; i < sr; ++i) {
                double I = static_cast<double>(data_ptr[2*i]);
                double Q = static_cast<double>(data_ptr[2*i + 1]);

                complex<double> x(I, Q);

                if (sd.filter_enabled){
                    x = mf_filter(sd, x);
                }
                if (sd.costas_loop_enabled){
                    x = costas_loop(sd, x);
                }

                sd.raw_buffer.push_back(x);
            }

            if (sd.raw_buffer.size() > SharedData::MAX_SAMPLES) {
                sd.raw_buffer.erase(sd.raw_buffer.begin(), sd.raw_buffer.end() - SharedData::MAX_SAMPLES);
            }
            if (sd.sym_sync_enabled) {
                sym_sync(sd, sd.raw_buffer);
                sd.symbols.clear();
                for (int i = sd.ss_offset; i < sd.raw_buffer.size(); i += SharedData::Nsp) {
                    sd.symbols.push_back(sd.raw_buffer[i]);
                }
                if (sd.symbols.size() > SharedData::MAX_SYMBOLS) {
                    sd.symbols.erase(sd.symbols.begin(), sd.symbols.end() - SharedData::MAX_SYMBOLS);
                }
            } else {
                sd.symbols = sd.raw_buffer;
            }
        }
    }
}

vector<SoapySDRKwargs> find_pluto_devices() {
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    
    size_t length;
    SoapySDRKwargs *devices = SoapySDRDevice_enumerate(&args, &length);
    
    std::vector<SoapySDRKwargs> result(devices, devices + length);
    
    SoapySDRKwargs_clear(&args);
    
    return result;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Usage: %s <pluto_addr> <tx|rx>\n", argv[0]);
        return -1;
    }

    auto sdr_devices = find_pluto_devices();
    int selected_device_index = 0;

    SharedData sd;

    sd.mf_delay.resize(sd.mf_L - 1, 0.0);

    thread Back;
    bool backend_started = false;

    sd.usb = argv[1];
    sd.type = argv[2];

    if (strcmp(sd.type, "rx") == 0){

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

            if (ImGui::BeginMainMenuBar()) {
                if (ImGui::BeginMenu("Device")) {
                    for (size_t i = 0; i < sdr_devices.size(); ++i) {
                        const char* label = SoapySDRKwargs_get(&sdr_devices[i], "label");
                        const char* uri = SoapySDRKwargs_get(&sdr_devices[i], "uri");
                        bool is_selected = (static_cast<size_t>(selected_device_index) == i);
                        
                        if (ImGui::MenuItem(label, nullptr, is_selected)) {
                            selected_device_index = static_cast<int>(i);
                            {
                                lock_guard<mutex> lock(sd.mtx);
                                sd.selected_uri = uri ? string(uri) : "";
                            }
                        }
                    }
                    ImGui::EndMenu();
                }

                if (ImGui::MenuItem("Start", nullptr, false, !backend_started)) {
                    if (sd.selected_uri.empty()) {
                        sd.selected_uri = SoapySDRKwargs_get(&sdr_devices[0], "uri");
                    }
                    Back = thread(Backend, ref(sd));
                    backend_started = true;
                }

                if (ImGui::MenuItem("Exit", nullptr, false, true)) {
                    g_running = false;
                    if (backend_started) {
                        Back.join();
                    }
                    running = false;
                }

                ImGui::EndMainMenuBar();
            }
            ImGuiViewport* vp = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(ImVec2(vp->Pos.x, vp->Pos.y + ImGui::GetFrameHeight()));
            ImGui::SetNextWindowSize(ImVec2(260, vp->Size.y - ImGui::GetFrameHeight()));

            ImGui::Begin("Control Panel", nullptr,
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoCollapse);

            float tx_gain = sd.tx_gain;
            if (ImGui::SliderFloat("tx gain", &tx_gain, -90, 40)) {
                lock_guard<mutex> lock(sd.mtx);
                sd.tx_gain = tx_gain;
            }

            float rx_gain = sd.rx_gain;
            if (ImGui::SliderFloat("rx gain", &rx_gain, -90, 40)) {
                lock_guard<mutex> lock(sd.mtx);
                sd.rx_gain = rx_gain;
            }

            int freq = sd.freq;
            if (ImGui::SliderInt("Carrier Freq", &freq, 200000000, 900000000)){
                lock_guard<mutex> lock(sd.mtx);
                sd.freq = freq;
            }

            bool enabled = sd.filter_enabled;
            if (ImGui::Checkbox("Enable Square Filter", &enabled)) {
                {
                    lock_guard<mutex> lock(sd.mtx);
                    sd.filter_enabled = enabled;
                    
                    if (!enabled) {
                        sd.mf_init = false;
                        sd.mf_index = 0;
                        sd.mf_sum = complex<double>(0.0);
                        fill(sd.mf_delay.begin(), sd.mf_delay.end(), complex<double>(0.0));
                    }
                }
            }

            int L = sd.mf_L;
            if (ImGui::SliderInt("Filter Length", &L, 2, 50)) {
                lock_guard<mutex> lock(sd.mtx);
                sd.mf_L = L;
                sd.mf_delay.resize(L - 1, 0.0);
                sd.mf_init = false;
                sd.mf_index = 0;
                sd.mf_sum = 0.0;
            }

            bool sync_enabled = sd.sym_sync_enabled;
            if (ImGui::Checkbox("Symbol Sync", &sync_enabled)) {
                sd.sym_sync_enabled = sync_enabled;
            }
            float BnTs = sd.BnTs;
            if (ImGui::SliderFloat("Bnts", &BnTs, 0, 1)) {
                lock_guard<mutex> lock(sd.mtx);
                sd.BnTs = BnTs;
            }
            ImGui::Text("Offset: %d", sd.ss_offset);

            ImGui::Checkbox("Costas Loop", &sd.costas_loop_enabled);
            ImGui::SliderFloat("Kp", &sd.cl_Kp, 0.0f, 0.3f);
            ImGui::SliderFloat("Ki", &sd.cl_Ki, 0.0f, 0.3f);

            ImGui::End();
        ImGui::SetNextWindowPos(ImVec2(
            vp->Pos.x + 260,
            vp->Pos.y + ImGui::GetFrameHeight()
        ));
        ImGui::SetNextWindowSize(ImVec2(
            vp->Size.x - 260,
            vp->Size.y - ImGui::GetFrameHeight()
        ));

        ImGui::Begin("Plots",
            nullptr,
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoTitleBar
        );

        ImVec2 size(2000,600);

        if (ImPlot::BeginPlot("Scatter Plot", size)) {
            vector<double> plot_real, plot_imag;
            {
                lock_guard<mutex> lock(sd.mtx);
                for (auto& s : sd.symbols) {
                    plot_real.push_back(s.real());
                    plot_imag.push_back(s.imag());
                }
            }
            if (!plot_real.empty())
                ImPlot::PlotScatter("IQ", plot_real.data(), plot_imag.data(), plot_real.size());
            ImPlot::EndPlot();
        }

        if (ImPlot::BeginPlot("Modulated Signal", size)) {
            vector<double> I, Q;
            {
                lock_guard<mutex> lock(sd.mtx);
                for (auto& s : sd.raw_buffer) {
                    I.push_back(s.real());
                    Q.push_back(s.imag());
                }
            }
            if (!I.empty()) {
                ImPlot::PlotLine("I", I.data(), I.size());
                ImPlot::PlotLine("Q", Q.data(), Q.size());
            }
            ImPlot::EndPlot();
        }

        ImGui::End();

        // Рендеринг
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
    
    // Очистка
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

    return 0;
}