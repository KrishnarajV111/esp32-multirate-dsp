# Real-Time Multirate Audio DSP Receiver (Simulation & HIL)

This project simulates and processes real-time audio signals using **Multirate Digital Signal Processing (DSP)** in MATLAB. 

It is designed to run in two modes:
1. **Simulation Mode (Default):** Runs entirely in MATLAB without requiring any external hardware. It synthesizes a live binary data stream matching an ESP32's exact byte format to verify the DSP pipeline.
2. **Hardware-in-the-Loop (HIL) Mode:** Connects to a physical **ESP32 microcontroller** over USB-Serial to process real-world analog audio samples in real-time.

---

## What does this project do?

The goal of this project is to take a high-rate digital audio signal ($F_s = 8000\text{ Hz}$), downsample it to a lower rate ($F_{s,\text{dec}} = 2000\text{ Hz}$), and eliminate aliasing artifacts using real-time FIR filtering.

Specifically, the MATLAB pipeline:
* Parses raw binary stream frames in real-time.
* Applies a **48-tap FIR Anti-Aliasing Filter** ($f_c = 900\text{ Hz}$).
* Performs **Decimation by $M = 4$**.
* Displays a live 4-subplot dashboard showing time-domain waveforms and FFT frequency spectrums before and after decimation.

---

## What is inside this repository?

```text
├── matlab/
│   └── multirate_dsp_receiver.m   # Main DSP script (Simulation + HIL)
├── esp32_firmware/
│   └── esp32_multirate_dsp.ino   # Firmware C++ code (Optional - for physical ESP32)
└── README.md