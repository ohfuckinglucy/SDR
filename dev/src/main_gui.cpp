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

#include <fftw3.h>

atomic<bool> g_running{false};

int bit_size = 1920*2;

void Backend(SharedData& sd, SDRConfig &config) {
    if (!config.sdr || !config.rxStream) {
        cerr << "ERROR: SDR config!" << endl;
        g_running = false;
        return;
    }

    vector<complex<double>> symbols;
    vector<complex<double>> symbols_UL;
    vector<int16_t> tx_samples(2 * config.tx_mtu, 0);

    sd.bits.resize(bit_size);
    const int16_t barker13[13] = {1,1,1,1,1,0,0,1,1,0,1,0,1};
    for (int i = 0; i < 26; ++i) {
        sd.bits[i] = barker13[i % 13];
    }
    for (int i = 26; i < bit_size; ++i) {
        sd.bits[i] = rand() % 2;
    }

    int cnt = 0;
    for (size_t samples_sent = 0; g_running; ++samples_sent) {
        if (sd.loopback_flag){
            string mod_type;

            if (sd.modulation_index == 0){
                mod_type = "QAM::2";
            } else if (sd.modulation_index == 2){
                mod_type = "QAM::16";
            } else {
                mod_type = "QAM::4";
            }

            vector<complex<double>> symbols = modulator(sd.bits.data(), bit_size, mod_type);

            vector<complex<double>> symbols_UL = UpSampler(symbols.data(), symbols.size(), sd.tx_l);

            if (sd.tx_filter) {
                filter(symbols_UL.data(), symbols_UL.size(), sd.tx_l);
            }

            
            tx_samples.resize(2 * symbols_UL.size());
            for (size_t i = 0; i < symbols_UL.size(); i++) {
                tx_samples[2*i]   = static_cast<int16_t>(real(symbols_UL[i]) * 16000);
                tx_samples[2*i+1] = static_cast<int16_t>(imag(symbols_UL[i]) * 16000);
            }
        }
        
        void *rx_buffs[] = {config.rx_buffer};
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(config.sdr, config.rxStream, rx_buffs, config.rx_mtu, &flags, &timeNs, TIMEOUT);

        long long tx_time = timeNs + TX_DELAY;
        flags = SOAPY_SDR_HAS_TIME;

        if (sd.loopback_flag && !tx_samples.empty()) {
            size_t total = tx_samples.size() / 2;
            if (samples_sent >= total) {
                samples_sent = 0;
            }
            size_t to_send = min(static_cast<size_t>(config.tx_mtu), total - samples_sent);
            const void* tx_buffs[] = { tx_samples.data() + samples_sent * 2 };
            int st = SoapySDRDevice_writeStream(config.sdr, config.txStream, tx_buffs, to_send, &flags, tx_time, TIMEOUT);
            samples_sent += to_send;
        }

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
                for (size_t i = sd.ss_offset; i < sd.raw_buffer.size(); i += SharedData::Nsp) {
                    sd.symbols.push_back(sd.raw_buffer[i]);
                }
                if (sd.symbols.size() > SharedData::MAX_SYMBOLS) {
                    sd.symbols.erase(sd.symbols.begin(), sd.symbols.end() - SharedData::MAX_SYMBOLS);
                }
            } else {
                sd.symbols = sd.raw_buffer;
            }

            if(sd.fft_flag){
                size_t n = min(sd.raw_buffer.size(), SharedData::FFT_SIZE);
                for (size_t i = 0; i < n; i++) {
                    sd.fft_in[i][0] = sd.raw_buffer[sd.raw_buffer.size() - n + i].real(); // I
                    sd.fft_in[i][1] = sd.raw_buffer[sd.raw_buffer.size() - n + i].imag(); // Q
                }

                for (size_t i = n; i < SharedData::FFT_SIZE; i++) {
                    sd.fft_in[i][0] = 0.0;
                    sd.fft_in[i][1] = 0.0;
                }

                fftw_execute(sd.fft_plan);

                for (size_t i = 0; i < SharedData::FFT_SIZE; i++) {
                    double re = sd.fft_out[i][0];
                    double im = sd.fft_out[i][1];
                    sd.fft_magnitude[i] = log10(re * re + im * im + 1e-10); // лог-масштаб
                }

                sd.fft_ready = true;
            }
        }
    }
}

int main() {
    struct SDRConfig config = {};

    auto sdr_devices = find_pluto_devices();
    int selected_device_index = 0;

    SharedData sd;

    sd.mf_delay.resize(sd.mf_L - 1, 0.0);

    thread Back;

    sd.fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * SharedData::FFT_SIZE);
    sd.fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * SharedData::FFT_SIZE);

    sd.fft_plan = fftw_plan_dft_1d(
        SharedData::FFT_SIZE,
        sd.fft_in,
        sd.fft_out,
        FFTW_FORWARD,
        FFTW_ESTIMATE
    );

    sd.fft_buffer.resize(SharedData::FFT_SIZE);
    sd.fft_magnitude.resize(SharedData::FFT_SIZE);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        cerr << "SDL_Init Error: " << SDL_GetError() << endl;
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "PlutoSDR Modulatora", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui::LoadIniSettingsFromDisk(io.IniFilename);

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    ImVec2 plotsize(1600, 600);

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
                g_running = false;
            }
        }

        sdr_devices = find_pluto_devices();
        
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

            if (ImGui::MenuItem("Start", nullptr, false, !g_running)) {
                string uri;
                {
                    lock_guard<mutex> lock(sd.mtx);
                    uri = sd.selected_uri;
                }

                if (uri.empty() && !sdr_devices.empty()) {
                    uri = SoapySDRKwargs_get(&sdr_devices[0], "uri");
                }

                if (uri.empty()) {
                    cerr << "No device URI available!" << endl;
                    continue;
                }

                config = SDRinit(const_cast<char*>(uri.c_str()), sd);

                if (!config.sdr) {
                    cerr << "Failed to initialize SDR device!" << endl;
                    continue;
                }

                g_running = true;
                Back = thread(Backend, ref(sd), ref(config));
            } else {
                if (ImGui::MenuItem("Stop TX", nullptr, true, g_running)) {
                g_running = false;
                if (Back.joinable()) {
                    Back.join();
                }
            }
            }

            if (ImGui::MenuItem("Exit", nullptr, false, true)) {
                g_running = false;
                if (Back.joinable()) {
                    Back.join();
                }
                running = false;
            }

            bool loopback = sd.loopback_flag;
            if (ImGui::Checkbox("Loopback", &loopback)) {
                sd.loopback_flag = loopback;
            }

            if (sd.loopback_flag) {
                ImGui::SameLine();
                if (ImGui::Button("Config")) {
                    ImGui::OpenPopup("TX Settings");
                }
            }

            if (ImGui::BeginPopup("TX Settings")) {
                ImGui::Text("TX Configuration");
                ImGui::Separator();

                const char* modulation_types[] = { "QAM::2", "QAM::4", "QAM::16" };
                static int modulation_idx = sd.modulation_index;
                if (ImGui::Combo("Modulation", &modulation_idx, modulation_types, IM_ARRAYSIZE(modulation_types))) {
                    sd.modulation_index = modulation_idx;
                }

                int L = sd.tx_l;
                if (ImGui::SliderInt("L", &L, 1, 100)) {
                    lock_guard<mutex> lock(sd.mtx);
                    sd.tx_l = L;
                }

                bool tx_filter = sd.tx_filter;
                if (ImGui::Checkbox("TX Filter", &tx_filter)) {
                    sd.tx_filter = tx_filter;
                }

                if (!sd.loopback_flag) {
                    ImGui::CloseCurrentPopup();
                }

                ImGui::EndPopup();
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

        float rx_gain = sd.rx_gain;
        if (ImGui::SliderFloat("rx gain", &rx_gain, -90, 40)) {
            if (g_running){
                lock_guard<mutex> lock(sd.mtx);
                SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_RX, 0, rx_gain);
            }
        }

        int freq = sd.freq;
        if (ImGui::SliderInt("Carrier Freq", &freq, 200000000, 900000000)){
            sd.freq = freq;
            if (g_running && config.sdr) {
                SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_RX, 0, static_cast<double>(freq), nullptr);
            }
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
        ImGui::Checkbox("Spectrum", &sd.fft_flag);

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

        float available_width = ImGui::GetContentRegionAvail().x;
        float plot_width = (available_width - ImGui::GetStyle().ItemSpacing.x) / 2.0f;
        ImVec2 plot_size1(-1,600);
        ImVec2 plot_size2(plot_width, 600);
        
        if (ImPlot::BeginPlot("Scatter Plot", plot_size2)) {
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

        ImGui::SameLine();

        if (ImPlot::BeginPlot("Modulated Signal", plot_size2)) {
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

        if (sd.fft_flag){
            if (ImPlot::BeginPlot("FFT", plot_size1)) {
                vector<double> local_mag;
                {
                    lock_guard<mutex> lock(sd.mtx);
                    if (sd.fft_ready) {
                        local_mag = sd.fft_magnitude;
                    }
                }
                if (!local_mag.empty()) {
                    vector<double> shifted(local_mag.size());
                    size_t half = local_mag.size() / 2;
                    for (size_t i = 0; i < half; i++) {
                        shifted[i] = local_mag[i + half];
                        shifted[i + half] = local_mag[i];
                    }
                    ImPlot::PlotLine("Magnitude", shifted.data(), shifted.size());
                }
                ImPlot::EndPlot();
            }
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

    if (g_running) {
        g_running = false;
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

    if (sd.fft_plan) {
        fftw_destroy_plan(sd.fft_plan);
        fftw_free(sd.fft_in);
        fftw_free(sd.fft_out);
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