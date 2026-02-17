#include "header.h"
#include "modulator.h"

thread tx_thread;

int main() {
    SharedData sd;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        cerr << "SDL_Init Error: " << SDL_GetError() << endl;
        return -1;
    }

    struct SDRConfig config = {};
    auto sdr_devices = find_pluto_devices();
    int selected_device_index = 0;

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
                        sd.dev_f.selected_uri = uri ? string(uri) : "";
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
        sd.flags.modulation_index = modulation_idx;

        int L = sd.FormFilter.tx_l;
        if (ImGui::SliderInt("L", &L, 1, 100)) {
            lock_guard<mutex> lock(sd.mtx);
            sd.FormFilter.tx_l = L;
        }
        bool upS = sd.flags.upsampling_enabled;
        if (ImGui::Checkbox("Upsampling", &upS)){
            lock_guard<mutex> lock(sd.mtx);
            sd.flags.upsampling_enabled = upS;
        }

        ImGui::Checkbox("FormFilter", &sd.flags.tx_filter);
        bool ofdm_flag = sd.flags.ofdm_enabled;
        if (ImGui::Checkbox("ofdm", &ofdm_flag)){
            lock_guard<mutex> lock(sd.mtx);
            sd.flags.ofdm_enabled = ofdm_flag;
        }
        
        if (!sd.flags.g_running) {
            if (ImGui::Button("Start TX")) {
                if (sd.dev_f.selected_uri.empty()) {
                    if (!sdr_devices.empty()) {
                        sd.dev_f.selected_uri = SoapySDRKwargs_get(&sdr_devices[0], "uri");
                    } else {
                        cerr << "No PlutoSDR devices found!" << endl;
                        ImGui::Text("No devices found!");
                    }
                }
                if (!sd.dev_f.selected_uri.empty()) {
                    config = SDRinit(const_cast<char*>(sd.dev_f.selected_uri.c_str()), sd);
                    if (config.sdr) {
                        sd.flags.g_running = true;
                        tx_thread = thread(tx_back, ref(sd), ref(config));
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
            if (tx_thread.joinable()) tx_thread.join();
        }

        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(10, 240));
        ImGui::SetNextWindowSize(ImVec2(350, 200));
        ImGui::Begin("Control Panel", nullptr, ImGuiWindowFlags_NoCollapse);

        float tx_gain = sd.tx_gain;
        if (ImGui::SliderFloat("TX Gain", &tx_gain, -90, 40)) {
            if (sd.flags.g_running){
                lock_guard<mutex> lock(sd.mtx);
                SoapySDRDevice_setGain(config.sdr, SOAPY_SDR_TX, 0, tx_gain);
            }
        }

        int freq = sd.freq;
        if (ImGui::SliderInt("Carrier Freq", &freq, 200000000, 900000000)) {
            sd.freq = freq;
            if (sd.flags.g_running && config.sdr) {
                SoapySDRDevice_setFrequency(config.sdr, SOAPY_SDR_TX, 0, freq, nullptr);
            }
        }

        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(10, 450));
        ImGui::SetNextWindowSize(ImVec2(350, 250));
        ImGui::Begin("First bits", nullptr, ImGuiWindowFlags_NoCollapse);

        int N = min(50, static_cast<int>(sd.last_tx_samples.size()) / 2);
        if (N > 0) {
            ImGui::Text("Idx |   I   |   Q");
            ImGui::Separator();
            for (int i = 0; i < N; ++i) {
                int16_t I = sd.last_tx_samples[2*i];
                int16_t Q = sd.last_tx_samples[2*i + 1];
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