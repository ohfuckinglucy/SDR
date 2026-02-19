#include "header.h"
#include "modulator.h"

int main() {
    struct SDRConfig config = {};

    auto sdr_devices = find_pluto_devices();
    int selected_device_index = 0;

    SharedData sd;

    sd.FormFilter.mf_delay.resize(sd.FormFilter.rx_l - 1, 0.0);

    thread Back;

    sd.fft.fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sd.fft.FFT_SIZE);
    sd.fft.fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sd.fft.FFT_SIZE);

    sd.fft.fft_plan = fftw_plan_dft_1d(
        sd.fft.FFT_SIZE,
        sd.fft.fft_in,
        sd.fft.fft_out,
        FFTW_FORWARD,
        FFTW_ESTIMATE
    );

    sd.fft.fft_buffer.resize(sd.fft.FFT_SIZE);
    sd.fft.fft_magnitude.resize(sd.fft.FFT_SIZE);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        cerr << "SDL_Init Error: " << SDL_GetError() << endl;
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "PlutoSDR Modulatora", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(0);

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
                sd.flags.g_running = false;
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
                            sd.dev_f.selected_uri = uri ? string(uri) : "";
                        }
                    }
                }
                ImGui::EndMenu();
            }

            if (ImGui::MenuItem("Start", nullptr, false, !sd.flags.g_running)) {
                string uri;
                {
                    lock_guard<mutex> lock(sd.mtx);
                    uri = sd.dev_f.selected_uri;
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

                sd.flags.g_running = true;
                Back = thread(rx_back, ref(sd), ref(config));
            } else {
                if (ImGui::MenuItem("Stop TX", nullptr, true, sd.flags.g_running)) {
                sd.flags.g_running = false;
                if (Back.joinable()) {
                    Back.join();
                }
            }
            }

            if (ImGui::MenuItem("Exit", nullptr, false, true)) {
                sd.flags.g_running = false;
                if (Back.joinable()) {
                    Back.join();
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
                ImGui::Text("TX Configuration");
                ImGui::Separator();

                float tx_gain = sd.tx_gain;
                if (ImGui::SliderFloat("rx gain", &tx_gain, -40, 40)) {
                    if (sd.flags.g_running){
                        sd.tx_gain = tx_gain;
                        sd.flags.tx_gain_changed = true;
                    }
                }

                const char* modulation_types[] = { "QAM::2", "QAM::4", "QAM::16" };
                static int modulation_idx = sd.flags.modulation_index;
                if (ImGui::Combo("Modulation", &modulation_idx, modulation_types, IM_ARRAYSIZE(modulation_types))) {
                    sd.flags.modulation_index = modulation_idx;
                }

                int L = sd.FormFilter.tx_l;
                if (ImGui::SliderInt("L", &L, 1, 100)) {
                    lock_guard<mutex> lock(sd.mtx);
                    sd.FormFilter.tx_l = L;
                }

                bool upsampling_enabled = sd.flags.upsampling_enabled;
                if (ImGui::Checkbox("UpSampling", &upsampling_enabled)) {
                    sd.flags.upsampling_enabled = upsampling_enabled;
                }

                bool tx_filter = sd.flags.tx_filter;
                if (ImGui::Checkbox("TX Filter", &tx_filter)) {
                    sd.flags.tx_filter = tx_filter;
                }

                if (!sd.flags.loopback_flag) {
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

        ImGui::Text("FPS: %.1f (%.3f ms)", io.Framerate, 1000.0f / io.Framerate);

        float rx_gain = sd.rx_gain;
        if (ImGui::SliderFloat("rx gain", &rx_gain, -40, 40)) {
            if (sd.flags.g_running){
                sd.rx_gain = rx_gain;
                sd.flags.rx_gain_changed = true;
            }
        }

        float freq = sd.freq;
        if (ImGui::SliderFloat("Carrier Freq", &freq, 200e6, 900e6, "%e")) {
            sd.freq = freq;
            if (sd.flags.g_running) {
                sd.freq = freq;
                sd.flags.rx_freq_changed = true;
            }
        }

        bool enabled = sd.flags.filter_enabled;
        if (ImGui::Checkbox("Enable Square Filter", &enabled)) {
            {
                lock_guard<mutex> lock(sd.mtx);
                sd.flags.filter_enabled = enabled;
                
                if (!enabled) {
                    sd.flags.mf_init = false;
                    sd.FormFilter.mf_index = 0;
                    sd.FormFilter.mf_sum = complex<double>(0.0);
                    fill(sd.FormFilter.mf_delay.begin(), sd.FormFilter.mf_delay.end(), complex<double>(0.0));
                }
            }
        }

        int L = sd.FormFilter.rx_l;
        if (ImGui::SliderInt("Filter Length", &L, 2, 50)) {
            lock_guard<mutex> lock(sd.mtx);
            sd.FormFilter.rx_l = L;
            sd.FormFilter.mf_delay.resize(L - 1, 0.0);
            sd.flags.mf_init = false;
            sd.FormFilter.mf_index = 0;
            sd.FormFilter.mf_sum = 0.0;
        }

        bool sync_enabled = sd.gardner.sym_sync_enabled;
        if (ImGui::Checkbox("Symbol Sync", &sync_enabled)) {
            sd.gardner.sym_sync_enabled = sync_enabled;
        }
        float BnTs = sd.gardner.BnTs;
        float KpG = sd.gardner.Kp;

        if (ImGui::SliderFloat("BnTs", &BnTs, 0.0f, 0.1f))
        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            sd.gardner.BnTs = BnTs;

            sd.gardner.ss_p1 = 0.0;
            sd.gardner.ss_p2 = 0.0;
        }

        if (ImGui::SliderFloat("Kp Gar", &KpG, 0.0f, 10.0f))
        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            sd.gardner.Kp = KpG;

            sd.gardner.ss_p1 = 0.0;
            sd.gardner.ss_p2 = 0.0;
        }


        ImGui::Text("Offset: %d", sd.gardner.ss_offset);

        ImGui::Checkbox("Costas Loop", &sd.flags.costas_loop_enabled);
        ImGui::SliderFloat("Kp", &sd.costas.cl_Kp, 0.0f, 0.3f);
        ImGui::SliderFloat("Ki", &sd.costas.cl_Ki, 0.0f, 0.3f);
        ImGui::Checkbox("Spectrum", &sd.flags.fft_flag);

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
        
        if (ImPlot::BeginPlot("Constellation", plot_size2)) {
            vector<double> plot_real, plot_imag;
            {
                lock_guard<mutex> lock(sd.mtx);
                size_t limit = min(sd.symbols.size(), (size_t)500);
                for (size_t i = 0; i < limit; ++i) {
                    plot_real.push_back(sd.symbols[i].real());
                    plot_imag.push_back(sd.symbols[i].imag());
                }
            }
            if (!plot_real.empty()) {
                ImPlot::SetupAxesLimits(-2, 2, -2, 2);
                ImPlot::PlotScatter("IQ", plot_real.data(), plot_imag.data(), plot_real.size());
            }
            ImPlot::EndPlot();
        }

        ImGui::SameLine();

        if (ImPlot::BeginPlot("RX Scope", plot_size2)) {
            vector<double> scope_I, scope_Q;
            {
                lock_guard<mutex> lock(sd.mtx);

                if (!sd.scope_buffer.empty()){
                    size_t count = sd.scope_filled ? sd.SCOPE_SIZE : sd.scope_head;
                    for (size_t i = 0; i < count; ++i) {
                        size_t idx = (sd.scope_head + i) % sd.SCOPE_SIZE;
                        scope_I.push_back(sd.scope_buffer[idx].real());
                        scope_Q.push_back(sd.scope_buffer[idx].imag());
                    }
                }
            }
            if (!scope_I.empty()) {
                ImPlot::SetupAxesLimits(0, scope_I.size(), -20000, 20000);
                ImPlot::PlotLine("I", scope_I.data(), scope_I.size());
                ImPlot::PlotLine("Q", scope_Q.data(), scope_Q.size());
            }
            ImPlot::EndPlot();
        }

        if (sd.flags.fft_flag){
            if (ImPlot::BeginPlot("FFT", plot_size1)) {
                vector<double> local_mag;
                {
                    lock_guard<mutex> lock(sd.mtx);
                    if (sd.flags.fft_ready) {
                        local_mag = sd.fft.fft_magnitude;
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

    if (sd.fft.fft_plan) {
        fftw_destroy_plan(sd.fft.fft_plan);
        fftw_free(sd.fft.fft_in);
        fftw_free(sd.fft.fft_out);
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