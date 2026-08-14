#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_sdl2.h"
#include "common.hpp"
#include "functions.hpp"
#include "imgui.h"
#include "implot.h"
#include "logger.hpp"
#include "ofdm_core.hpp"

#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <cstddef>
#include <fftw3.h>
#include <thread>
#include <vector>

int main()
{
    struct SharedData sd = {};
    update_pilots(sd);
    ensure_fft_plans(sd);

    logs::gui.info("Looking for SDR devices...");
    auto devices = SDR::findDevices();
    if (devices.empty())
    {
        logs::sdr.error("No SDR devices found");
        return -1;
    }

    std::string device_uri = devices[0].at("uri");
    logs::gui.info("Opening device: {}", device_uri);
    SDR sdr(device_uri);
    logs::gui.info("Device opened");

    logs::gui.info("Starting worker threads...");
    std::thread sdr_thread(SDRStream, std::ref(sd), std::ref(sdr));
    std::thread dsp_thread(DSPThread, std::ref(sd));

    logs::gui.info("Initializing SDL...");
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
    {
        logs::gui.critical("SDL_Init failed: {}", SDL_GetError());
        return -1;
    }

    SDL_Window *window = SDL_CreateWindow("Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (window == nullptr)
    {
        logs::gui.critical("SDL_CreateWindow failed: {}", SDL_GetError());
        return -1;
    }
    logs::gui.info("Window created");

    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (gl_context == nullptr)
    {
        logs::gui.critical("SDL_GL_CreateContext failed: {}", SDL_GetError());
        return -1;
    }
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
    while (running)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
            {
                running = false;
                sd.allRunning = false;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_None);

        ImGui::Begin("Panel");

        if (ImGui::CollapsingHeader("Debug", ImGuiTreeNodeFlags_DefaultOpen))
        {
            int best_idx = -1;
            float cfo = 0.0f;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                best_idx = sd.ofdmcfg.best_idx;
                cfo = sd.ofdmcfg.cfo_est;
            }
            ImGui::Text("FPS: %.1f (%.3f ms)", io.Framerate, 1000.0f / io.Framerate);
            ImGui::Text("Stream latency: %.2f us", sd.avg_stream_time);
            ImGui::Text("DSP latency: %.2f us", sd.avg_dsp_time);
            ImGui::Text("PSS Sync: %d", sd.dspflags.PSS ? best_idx : -1);
            ImGui::Text("CFO: %.2f", sd.dspflags.CFO ? cfo : -1);
            ImGui::Text("BLER: %.2f", sd.stats.BLER);
        }

        if (ImGui::CollapsingHeader("Header", ImGuiTreeNodeFlags_DefaultOpen))
        {
            Header hdr_copy;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                hdr_copy = sd.hdr;
            }
            ImGui::Text("Num Samples: %zu", hdr_copy.num_samples);
            ImGui::Text("Modulation Type: %s", GetModulationName(hdr_copy.modulation));

            ImGui::Text("Flags:");
            if (hdr_copy.flag != 0)
            {
                if (hdr_copy.flag & FrameFlag::IsFirst)
                {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "[FIRST]");
                }
                if (hdr_copy.flag & FrameFlag::IsLast)
                {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "[LAST]");
                }
            }
            else
            {
                ImGui::SameLine();
                ImGui::TextDisabled("None");
            }

            ImGui::Text("Signal Type: %s", GetSignalTypeName(hdr_copy.sig_type));
        }

        if (ImGui::CollapsingHeader("SDR Settings"))
        {
            ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.3f);
            if (ImGui::SliderFloat("RX Gain", &sd.SDR.rx_gain, 0.0f, 73.0f, "%.1f dB"))
                sd.SDR.dirty_mask |= SDRField::RxGain;
            ImGui::SameLine();
            if (ImGui::SliderFloat("TX Gain ", &sd.SDR.tx_gain, 0.0f, 89.0f, "%.1f dB"))
                sd.SDR.dirty_mask |= SDRField::TxGain;

            if (ImGui::InputFloat("RX Hz   ", &sd.SDR.rx_freq, 1000.0f, 1000000.0f, "%.3e"))
                sd.SDR.dirty_mask |= SDRField::RxFreq;
            ImGui::SameLine();
            if (ImGui::InputFloat("TX Hz   ", &sd.SDR.tx_freq, 1000.0f, 1000000.0f, "%.3e"))
                sd.SDR.dirty_mask |= SDRField::TxFreq;

            if (ImGui::InputFloat("RX Bw   ", &sd.SDR.rx_bw, 100000.0f, 1000000.0f, "%.3e"))
                sd.SDR.dirty_mask |= SDRField::RxBW;
            ImGui::SameLine();
            if (ImGui::InputFloat("TX Bw   ", &sd.SDR.tx_bw, 100000.0f, 1000000.0f, "%.3e"))
                sd.SDR.dirty_mask |= SDRField::TxBW;

            if (ImGui::InputFloat("RX SRate", &sd.SDR.rx_sample_rate, 100000.0f, 1000000.0f, "%.3e"))
                sd.SDR.dirty_mask |= SDRField::RxSampleRate;
            ImGui::SameLine();
            if (ImGui::InputFloat("TX SRate", &sd.SDR.tx_sample_rate, 100000.0f, 1000000.0f, "%.3e"))
                sd.SDR.dirty_mask |= SDRField::TxSampleRate;

            ImGui::PopItemWidth();
        }

        if (ImGui::CollapsingHeader("Transmission Control"))
        {
            static int sel_sig = 0;
            const char *names_sig[] = { "Random", "Text", "File" };
            if (ImGui::Combo("Type", &sel_sig, names_sig, 3))
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                sd.type_of_signal = static_cast<SignalType>(sel_sig);
                sd.sig_changed = true;
            }

            if (sel_sig == 1)
            {
                static char tx_buf[1500] = "Text";
                ImGui::InputTextMultiline("Message", tx_buf, sizeof(tx_buf), ImVec2(-1, 60));
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    sd.tx_text = tx_buf;
                }
                if (ImGui::IsItemDeactivatedAfterEdit())
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    sd.sig_changed = true;
                }

                ImGui::SeparatorText("Received");
                ImGui::BeginChild("##rx_text", ImVec2(-1, 100), ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    ImGui::TextWrapped("%s", sd.decoded_text.c_str());
                    if (sd.decoded_text.empty())
                        ImGui::TextDisabled("Waiting for text...");
                }
                ImGui::EndChild();
            }

            if (sel_sig == 2)
            {
                ImGui::SeparatorText("TX File");

                static char file_path_buf[1024] = "";
                ImGui::SetNextItemWidth(-80.0f);
                ImGui::InputText("##filepath", file_path_buf, sizeof(file_path_buf));
                ImGui::SameLine();

                if (ImGui::Button("Load##file"))
                {
                    {
                        std::lock_guard<std::mutex> lock(sd.mtx);
                        sd.tx_file_path = file_path_buf;
                    }
                    if (LoadFileForTX(sd))
                    {
                        {
                            std::lock_guard<std::mutex> lock(sd.mtx);
                            sd.sig_changed = true;
                        }
                        logs::sdr.info("File loaded: {} ({} bytes)", sd.tx_file_name, sd.tx_file_data.size());
                    }
                    else
                    {
                        {
                            std::lock_guard<std::mutex> lock(sd.mtx);
                            sd.tx_file_loaded = false;
                        }
                        logs::sdr.error("Failed to open file: {}", sd.tx_file_path);
                    }
                }

                size_t tx_chunks_total = 0;
                size_t tx_chunk_idx = 0;
                bool file_loaded = false;
                bool file_received = false;
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    tx_chunks_total = sd.tx_file_total_chunks;
                    tx_chunk_idx = sd.tx_file_chunk_idx;
                    file_loaded = sd.tx_file_loaded;
                    file_received = sd.file_received;
                }

                if (file_loaded)
                {
                    size_t file_kb = sd.tx_file_data.size() / 1024;
                    size_t file_b = sd.tx_file_data.size() % 1024;
                    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Loaded: %s  (%zu KB %zu B)", sd.tx_file_name.c_str(), file_kb, file_b);
                    ImGui::Text("Chunks: %zu x %zu B", tx_chunks_total, FILE_CHUNK_BYTES);

                    ImGui::Spacing();
                    if (ImGui::Button("Send File", ImVec2(-1, 0)))
                    {
                        std::lock_guard<std::mutex> lock(sd.mtx);
                        sd.tx_file_chunk_idx = 0;
                        sd.tx_once = true;
                    }

                    if (tx_chunks_total > 0)
                    {
                        float progress = static_cast<float>(tx_chunk_idx) / static_cast<float>(tx_chunks_total);
                        char prog_label[64];
                        snprintf(prog_label, sizeof(prog_label), "%zu / %zu chunks", tx_chunk_idx, tx_chunks_total);
                        ImGui::ProgressBar(progress, ImVec2(-1, 0), prog_label);
                    }
                }
                else
                {
                    ImGui::TextDisabled("No file loaded");
                    ImGui::BeginDisabled();
                    ImGui::Button("Send File", ImVec2(-1, 0));
                    ImGui::EndDisabled();
                }

                ImGui::SeparatorText("RX File");
                if (file_received)
                {
                    {
                        std::lock_guard<std::mutex> lock(sd.mtx);
                        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Received: %s", sd.rx_file_name.c_str());
                        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Saved to: %s", sd.rx_file_save_path.c_str());
                        ImGui::Text("Size: %zu bytes", sd.rx_file_chunks_buf.size());

                        if (ImGui::Button("Clear##rxfile"))
                        {
                            sd.file_received = false;
                            sd.rx_file_name.clear();
                            sd.rx_file_chunks_buf.clear();
                            sd.rx_file_save_path.clear();
                        }
                    }
                }
                else
                {
                    size_t rx_size = 0;
                    {
                        std::lock_guard<std::mutex> lock(sd.mtx);
                        rx_size = sd.rx_file_chunks_buf.size();
                    }
                    if (rx_size > 0)
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Receiving... (%zu bytes so far...)", rx_size);
                        float prog = static_cast<float>(rx_size % 10000) / 10000.0f;
                        ImGui::ProgressBar(prog, ImVec2(-1, 0), "receiving...");
                    }
                    else
                    {
                        ImGui::TextDisabled("Waiting for file...");
                    }
                }
            }

            if (sel_sig == 0)
            {
                static int bits_count = 100;
                if (ImGui::SliderInt("Bits Count", &bits_count, 0, 10000))
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    if (sd.num_samples != static_cast<size_t>(bits_count))
                    {
                        sd.num_samples = static_cast<size_t>(bits_count);
                        sd.sig_changed = true;
                    }
                }
            }

            static int sel_mod = 0;
            const char *names_mod[] = { "BPSK", "QPSK", "QAM16", "QAM64" };
            if (ImGui::Combo("Modulation", &sel_mod, names_mod, 4))
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                sd.type_of_modulation = static_cast<SignalModulation>(sel_mod);
                sd.sig_changed = true;
            }

            if (sel_sig != 2)
            {
                if (ImGui::Button("Send Burst"))
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    sd.tx_once = true;
                }
                ImGui::SameLine();
                {
                    bool cont = false;
                    {
                        std::lock_guard<std::mutex> lock(sd.mtx);
                        cont = sd.tx_continuous;
                    }
                    if (ImGui::Checkbox("Continuous TX", &cont))
                    {
                        std::lock_guard<std::mutex> lock(sd.mtx);
                        sd.tx_continuous = cont;
                        sd.sig_changed = true;
                    }
                }
            }
        }

        if (ImGui::CollapsingHeader("OFDM Settings"))
        {
            int N_val = sd.ofdmcfg.N;
            int CP_val = sd.ofdmcfg.CP;
            int q_val = sd.ofdmcfg.q;
            int pilots_val = sd.ofdmcfg.num_pilots;

            bool changed = false;

            if (ImGui::SliderInt("Symbol Len (N)", &N_val, 64, 2048))
                changed = true;

            int max_cp = N_val / 2;
            if (ImGui::SliderInt("Prefix Len (CP)", &CP_val, 4, max_cp))
                changed = true;

            if (ImGui::SliderInt("ZC Root (q)", &q_val, 1, 127))
                changed = true;

            if (ImGui::SliderInt("Pilots Count", &pilots_val, 2, N_val / 8))
                changed = true;

            if (changed)
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                sd.ofdmcfg.N = N_val;
                sd.ofdmcfg.CP = CP_val;
                sd.ofdmcfg.q = static_cast<int16_t>(q_val);
                sd.ofdmcfg.num_pilots = pilots_val;
                sd.sig_changed = true;
                update_pilots(sd);
                sd.SDR.dirty_mask |= SDRField::OFDMConfig;
            }
        }

        if (ImGui::CollapsingHeader("DSP Control"))
        {
            if (ImGui::Checkbox("PSS", &sd.dspflags.PSS))
            {
                if (!sd.dspflags.PSS)
                {
                    std::lock_guard<std::mutex> lock(sd.mtx);
                    sd.timing_offsets.clear();
                    sd.gui_timing_offsets.clear();
                }
            }
            ImGui::SameLine();
            ImGui::SliderInt("Mystery Offset", &sd.ofdmcfg.mystery_offset, -25, 25);

            ImGui::Checkbox("FFT", &sd.dspflags.FFT);
            ImGui::SameLine();
            ImGui::Checkbox("CFO", &sd.dspflags.CFO);
            ImGui::SameLine();
            ImGui::Checkbox("EQ ", &sd.dspflags.EQ);
        }

        ImGui::End();

        ImGui::Begin("Raw Signal");
        if (ImPlot::BeginPlot("##Raw Signal", ImVec2(-1, -1)))
        {
            std::vector<float> I, Q;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                I.reserve(sd.gui_buffer.size() / 2);
                Q.reserve(sd.gui_buffer.size() / 2);
                for (const auto &val : sd.gui_buffer)
                {
                    I.push_back(val.real());
                    Q.push_back(val.imag());
                }
            }
            if (!I.empty())
            {
                ImPlot::PlotLine("In-phase", I.data(), I.size());
                ImPlot::PlotLine("Quadrature", Q.data(), Q.size());
            }
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Timing offsets");
        if (ImPlot::BeginPlot("##Timing offsets", ImVec2(-1, -1)))
        {
            std::lock_guard<std::mutex> lock(sd.mtx);
            ImPlot::PlotLine("Offset", sd.gui_timing_offsets.data(), sd.gui_timing_offsets.size());
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Constellation", nullptr, ImGuiWindowFlags_NoTitleBar);
        if (ImPlot::BeginPlot("##Constellation", ImVec2(-1, -1)))
        {
            static std::vector<float> I, Q;
            const size_t MAX_POINTS = 1000;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                size_t total_size = sd.gui_buffer.size();
                size_t points_plot = std::min(total_size, MAX_POINTS);
                I.clear();
                Q.clear();
                I.reserve(points_plot);
                Q.reserve(points_plot);
                size_t start_idx = total_size - points_plot;
                for (size_t i = start_idx; i < total_size; ++i)
                {
                    I.push_back(sd.gui_buffer[i].real());
                    Q.push_back(sd.gui_buffer[i].imag());
                }
            }
            if (!I.empty())
                ImPlot::PlotScatter("IQ", I.data(), Q.data(), I.size());
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("Spectre");
        if (ImPlot::BeginPlot("##SpectrePlot", ImVec2(-1, -1)))
        {
            ImPlot::SetupAxes("Frequency (MHz)", "Magnitude (dB)");

            std::vector<float> local_spec;
            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                local_spec = sd.spectrum;
            }

            std::vector<float> freq_axis(local_spec.size());
            float fs = sd.SDR.rx_sample_rate;
            float carrer = sd.SDR.rx_freq;
            for (size_t i = 0; i < freq_axis.size(); ++i)
                freq_axis[i] = (carrer - fs / 2.0f + i * (fs / freq_axis.size())) / 1e6f;

            if (!local_spec.empty())
                ImPlot::PlotLine("Spectrum", freq_axis.data(), local_spec.data(), local_spec.size());

            if (ImPlot::IsPlotHovered())
            {
                ImPlotPoint mouse = ImPlot::GetPlotMousePos();
                ImPlot::DragLineX(555, &mouse.x, ImVec4(1, 1, 0, 0.1f), 1, ImPlotDragToolFlags_NoInputs);
                ImPlot::DragLineY(666, &mouse.y, ImVec4(1, 1, 0, 0.1f), 1, ImPlotDragToolFlags_NoInputs);
                ImGui::BeginTooltip();
                ImGui::Text("Freq: %.3f MHz", mouse.x);
                ImGui::Text("Ampl: %.1f dB", mouse.y);
                ImGui::EndTooltip();
            }
            ImPlot::EndPlot();
        }
        ImGui::End();

        ImGui::Begin("##Other", nullptr, ImGuiWindowFlags_NoTitleBar);
        {
            static std::vector<float> evm_render_buffer;
            static std::vector<float> snr_render_buffer;
            size_t samples_count = 0;

            {
                std::lock_guard<std::mutex> lock(sd.mtx);
                if (evm_render_buffer.size() != sd.stats.vec_size)
                    evm_render_buffer.resize(sd.stats.vec_size);
                if (snr_render_buffer.size() != sd.stats.vec_size)
                    snr_render_buffer.resize(sd.stats.vec_size);

                samples_count = std::min((size_t)sd.stats.frames_processed, (size_t)sd.stats.vec_size);

                size_t render_start = (sd.stats.vec_offset + 1) % sd.stats.vec_size;
                for (size_t i = 0; i < samples_count; ++i)
                {
                    evm_render_buffer[i] = sd.stats.EVM_vec[(render_start + i) % sd.stats.vec_size];
                    snr_render_buffer[i] = sd.stats.SNR_vec[(render_start + i) % sd.stats.vec_size];
                }
            }

            if (samples_count > 0)
            {
                if (ImPlot::BeginPlot("EVM", ImVec2(-1, 300)))
                {
                    ImPlot::SetupAxes("Frames", "EVM (%)");
                    ImPlot::SetupAxesLimits(0, sd.stats.vec_size, 0, 100, ImGuiCond_Once);
                    ImPlot::PlotLine("EVM", evm_render_buffer.data(), (int)samples_count);
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("SNR", ImVec2(-1, 300)))
                {
                    ImPlot::SetupAxes("Frames", "EVM dB");
                    ImPlot::SetupAxesLimits(0, sd.stats.vec_size, 0, 100, ImGuiCond_Once);
                    ImPlot::PlotLine("SNR", snr_render_buffer.data(), (int)samples_count);
                    ImPlot::EndPlot();
                }
            }
        }
        ImGui::End();

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            SDL_GL_MakeCurrent(window, gl_context);
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            SDL_GL_MakeCurrent(window, gl_context);
        }
        SDL_GL_SwapWindow(window);
    }

    sd.allRunning = false;
    if (sdr_thread.joinable())
        sdr_thread.join();
    if (dsp_thread.joinable())
        dsp_thread.join();

    fftwf_destroy_plan(sd.fftplans.plan_spectre);
    fftwf_destroy_plan(sd.fftplans.plan_fft);
    fftwf_destroy_plan(sd.fftplans.plan_ifft);
    fftwf_free(sd.fftplans.in_spectre);
    fftwf_free(sd.fftplans.out_spectre);
    fftwf_free(sd.fftplans.in_fft);
    fftwf_free(sd.fftplans.in_ifft);
    fftwf_free(sd.fftplans.out_fft);
    fftwf_free(sd.fftplans.out_ifft);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}