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
    auto devices = SDR::findDevices();
    if (devices.empty())
    {
        logs::sdr.error("SDR doesn't found");
        return -1;
    }

    std::string device_uri = devices[0].at("uri");
    SDR sdr(device_uri);

    sd.fftplans.in_spectre = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * sd.fftplans.N_spec);
    sd.fftplans.out_spectre = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * sd.fftplans.N_spec);
    sd.fftplans.plan_spectre = fftwf_plan_dft_1d(sd.fftplans.N_spec, sd.fftplans.in_spectre, sd.fftplans.out_spectre, FFTW_FORWARD, FFTW_ESTIMATE);

    sd.fftplans.in_ifft = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * sd.ofdmcfg.N);
    sd.fftplans.out_ifft = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * sd.ofdmcfg.N);
    sd.fftplans.plan_ifft = fftwf_plan_dft_1d(sd.ofdmcfg.N, sd.fftplans.in_ifft, sd.fftplans.out_ifft, FFTW_BACKWARD, FFTW_ESTIMATE);

    sd.fftplans.in_fft = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * sd.ofdmcfg.N);
    sd.fftplans.out_fft = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * sd.ofdmcfg.N);
    sd.fftplans.plan_fft = fftwf_plan_dft_1d(sd.ofdmcfg.N, sd.fftplans.in_fft, sd.fftplans.out_fft, FFTW_FORWARD, FFTW_ESTIMATE);

    std::thread sdr_thread(SDRStream, std::ref(sd), std::ref(sdr));
    std::thread dsp_thread(DSPThread, std::ref(sd));

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);

    SDL_Window *window = SDL_CreateWindow("Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
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
            ImGui::Text("FPS: %.1f (%.3f ms)", io.Framerate, 1000.0f / io.Framerate);
            ImGui::Text("Stream latency: %.2f us", sd.avg_stream_time);
            ImGui::Text("DSP latency: %.2f us", sd.avg_dsp_time);
            ImGui::Text("PSS Sync: %d", sd.dspflags.PSS ? sd.ofdmcfg.best_idx : -1);
            ImGui::Text("CFO: %.2f", sd.dspflags.CFO ? sd.ofdmcfg.cfo_est : -1);
            ImGui::Text("BLER: %.2f", sd.stats.BLER);
        }

        if (ImGui::CollapsingHeader("Header", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Text("Num Samples: %ld", sd.hdr.num_samples);
            ImGui::Text("Modulation Type: %s", GetModulationName(sd.hdr.modulation));

            ImGui::Text("Flags:");
            if (sd.hdr.flag != 0)
            {
                if (sd.hdr.flag & FrameFlag::IsFirst)
                {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "[FIRST]");
                }
                if (sd.hdr.flag & FrameFlag::IsLast)
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

            ImGui::Text("Signal Type: %s", GetSignalTypeName(sd.hdr.sig_type));
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
                sd.type_of_signal = static_cast<SignalType>(sel_sig);
                sd.sig_changed = true;
            }

            if (sel_sig == 1)
            {
                static char tx_buf[1500] = "Text";
                ImGui::InputTextMultiline("Message", tx_buf, sizeof(tx_buf), ImVec2(-1, 60));
                sd.tx_text = tx_buf;
                if (ImGui::IsItemDeactivatedAfterEdit())
                    sd.sig_changed = true;

                ImGui::SeparatorText("Received");
                ImGui::BeginChild("##rx_text", ImVec2(-1, 100), ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
                ImGui::TextWrapped("%s", sd.decoded_text.c_str());
                if (sd.decoded_text.empty())
                    ImGui::TextDisabled("Waiting for text...");
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
                    sd.tx_file_path = file_path_buf;
                    if (LoadFileForTX(sd))
                    {
                        sd.sig_changed = true;
                        logs::sdr.info("File loaded: {} ({} bytes)", sd.tx_file_name, sd.tx_file_data.size());
                    }
                    else
                    {
                        sd.tx_file_loaded = false;
                        logs::sdr.error("Failed to open file: {}", sd.tx_file_path);
                    }
                }

                if (sd.tx_file_loaded)
                {
                    size_t file_kb = sd.tx_file_data.size() / 1024;
                    size_t file_b = sd.tx_file_data.size() % 1024;
                    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Loaded: %s  (%zu KB %zu B)", sd.tx_file_name.c_str(), file_kb, file_b);
                    ImGui::Text("Chunks: %zu x %zu B", sd.tx_file_total_chunks, FILE_CHUNK_BYTES);

                    ImGui::Spacing();
                    if (ImGui::Button("Send File", ImVec2(-1, 0)))
                    {
                        sd.tx_file_chunk_idx = 0;
                        sd.tx_once = true;
                    }

                    if (sd.tx_file_total_chunks > 0)
                    {
                        float progress = static_cast<float>(sd.tx_file_chunk_idx) / static_cast<float>(sd.tx_file_total_chunks);
                        char prog_label[64];
                        snprintf(prog_label, sizeof(prog_label), "%zu / %zu chunks", sd.tx_file_chunk_idx, sd.tx_file_total_chunks);
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
                if (sd.file_received)
                {
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
                else if (!sd.rx_file_name.empty())
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Receiving: %s (%zu bytes so far...)", sd.rx_file_name.c_str(), sd.rx_file_chunks_buf.size());
                    float prog = static_cast<float>(sd.rx_file_chunks_buf.size() % 10000) / 10000.0f;
                    ImGui::ProgressBar(prog, ImVec2(-1, 0), "receiving...");
                }
                else
                {
                    ImGui::TextDisabled("Waiting for file...");
                }
            }

            if (sel_sig == 0)
            {
                size_t old_num_samples = sd.num_samples;
                ImGui::SliderInt("Bits Count", (int *)&sd.num_samples, 0, 10000);
                if (old_num_samples != sd.num_samples)
                    sd.sig_changed = true;
            }

            static int sel_mod = 0;
            const char *names_mod[] = { "BPSK", "QPSK", "QAM16", "QAM64" };
            if (ImGui::Combo("Modulation", &sel_mod, names_mod, 4))
            {
                sd.type_of_modulation = static_cast<SignalModulation>(sel_mod);
                sd.sig_changed = true;
            }

            if (sel_sig != 2)
            {
                if (ImGui::Button("Send Burst"))
                    sd.tx_once = true;
                ImGui::SameLine();
                ImGui::Checkbox("Continuous TX", &sd.tx_continuous);
            }
        }

        if (ImGui::CollapsingHeader("OFDM Settings"))
        {
            bool changed = false;

            if (ImGui::SliderInt("Symbol Len (N)", &sd.ofdmcfg.N, 64, 2048))
            {
                changed = true;
                sd.sig_changed = true;
            }

            int max_cp = sd.ofdmcfg.N / 2;
            if (ImGui::SliderInt("Prefix Len (CP)", &sd.ofdmcfg.CP, 4, max_cp))
            {
                changed = true;
                sd.sig_changed = true;
            }

            if (ImGui::SliderInt("ZC Root (q)", (int *)&sd.ofdmcfg.q, 1, 127))
            {
                changed = true;
                sd.sig_changed = true;
            }

            if (ImGui::SliderInt("Pilots Count", &sd.ofdmcfg.num_pilots, 2, sd.ofdmcfg.N / 8))
            {
                changed = true;
                sd.sig_changed = true;
            }

            if (changed)
            {
                sd.SDR.dirty_mask |= SDRField::OFDMConfig;
                update_pilots(sd);
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