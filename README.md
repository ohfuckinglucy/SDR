# PlutoSDR OFDM Transceiver

An educational OFDM (Orthogonal Frequency Division Multiplexing) transceiver for the
[Analog Devices PlutoSDR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/adalm-pluto.html),
built on top of [SoapySDR](https://github.com/TelecomDep/SoapySDR). It features a
real-time DSP chain (synchronization, channel estimation/equalization, FEC) and a
Dear ImGui / ImPlot GUI for debugging and transmission control.

## Features

- **Frame format** per burst:
  `[ZC preamble] [header (38 bits, BPSK)] [payload (BPSK/QPSK/QAM16/QAM64)]`
  - ZC (Zadoff-Chu, N=127) preamble for timing synchronization
  - Header carries magic, payload bit count, modulation, frame flags, signal type
  - CRC-16 over the payload, verified on receive
- **DSP blocks** (toggleable from the GUI):
  - `PSS` — Zadoff-Chu sliding-correlation timing sync
  - `CFO` — carrier frequency offset estimation/correction via CP correlation
  - `FFT` — OFDM demodulation
  - `EQ` — pilot-based channel estimation, interpolation and equalization
- **Signal modes**: Random bits, Text messages, and File transfer
  (file is split into chunks with First/Last frame flags)
- **Live metrics**: spectrum, constellation, EVM, SNR, BLER, timing-offset plot

## Project layout

```
dev/
├── CMakeLists.txt
├── include/           # shared headers (common.hpp, ofdm_core.hpp, ...)
├── functions/         # DSP implementation
│   ├── ofdm_core.cpp      # guard bands, pilots, IFFT/FFT, equalization
│   ├── sync_time.cpp      # ZC preamble generation + timing sync
│   ├── sync_freq.cpp      # CFO estimation/correction
│   ├── modulator.cpp      # BPSK/QPSK/QAM16/QAM64 modulation/demodulation
│   ├── error_interleaving.cpp  # CRC-16
│   ├── functions.cpp      # frame assembly/parsing, spectrum, EVM
│   └── threads.cpp        # RX DSP thread + SDR stream thread
└── src/
    ├── main_gui.cpp   # main receiver/transceiver GUI application
    └── main_tx.cpp    # minimal TX-only GUI template
```

## Dependencies

Build-time packages (Debian/Ubuntu):

```bash
sudo apt install cmake g++ libsdl2-dev libgl1-mesa-dev libglew-dev libfftw3-dev
sudo apt install libsoapysdr-dev libsoapysdr0.8
```

> Note: the project was developed against the `soapy-sdr-0.8.1` branch of
> SoapySDR together with the `sdr_gadget_timestamping` branch of SoapyPlutoSDR.
> If your distribution ships different versions, adjust accordingly.

[ImGui](https://github.com/ocornut/imgui) and [ImPlot](https://github.com/epezent/implot)
are provided as git submodules (see `thirdparty/`); [spdlog](https://github.com/gabime/spdlog)
is fetched automatically by CMake.

### Submodules

```bash
git submodule update --init --recursive
```

## Build

```bash
cd dev
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Run

```bash
cd dev/build
./rx.elf   # main GUI application: RX stream + TX control
./tx.elf   # TX-only GUI template
```

> **Do not run with `sudo`** — GUI processes under root cannot access your
> X/Wayland display and will hang without a window. If you get USB permission
> errors from SoapySDR, install a udev rule for the Pluto instead (one time):

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0456", MODE="0666", GROUP="plugdev"' \
    | sudo tee /etc/udev/rules.d/80-plutosdr.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> **Note:** the udev rule above is only required on **Arch Linux** (and other
> distros where the `libiio`/`SoapyPlutoSDR` packages do not ship USB rules).
> On Debian/Ubuntu these rules are installed automatically with `libiio`.

Unplug/replug the Pluto (or reboot), then launch without `sudo`.

## Usage hints

- Enable **PSS**, **FFT**, **EQ** (and optionally **CFO**) in the `DSP Control`
  panel — the full RX chain is only active when these are enabled.
- With **PSS** enabled, the received frame is aligned to the ZC preamble; the
  `Mystery Offset` slider fine-tunes the alignment manually.
- `SDR Settings` — gains, frequency, bandwidth and sample rate are applied live
  (sample-rate changes restart the streams automatically).
- `OFDM Settings` — symbol length `N`, cyclic prefix `CP`, ZC root `q` and pilot
  count can be tweaked at runtime; FFT plans are rebuilt automatically.
- `Transmission Control` — choose Random/Text/File, pick modulation, then
  **Send Burst** or enable **Continuous TX** for streaming.

## Known limitations

- Header payload bit count is limited to 16 bits (max ~8191 bytes per frame).
- Received files are always saved as `image.png` in the working directory
  (filename is not transmitted).
