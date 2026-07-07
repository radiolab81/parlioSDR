## 🧩 parlioSDR Gateware: FPGA-Assisted I/Q Processing

Welcome to the gateware directory of the parlioSDR project!

While the ESP32-P4’s PARLIO interface is an incredible leap forward for microcontroller-based SDRs, pushing raw high-frequency RF data directly from the MCU can quickly bottleneck the system. To unlock a massive frequency range and preserve the ESP32's CPU cycles for networking and control, we are introducing hardware acceleration via FPGAs.

## ⚡ The Strategy

By utilizing a dedicated FPGA (such as Lattice ECP5, Gowin Tang Nano, or Altera Cyclone boards) between the ESP32 and the DAC/ADC, the architecture transforms:

    1. I/Q via PARLIO: The ESP32 streams lightweight, baseband I/Q data over the PARLIO bus instead of heavy RF samples.
    
    2. Hardware Mixing: The generalized Verilog DSP core inside the FPGA takes these I/Q samples, applies a Hardware-NCO, and performs Digital Up/Down Conversion (DUC/DDC).

    3. RF Output: The FPGA directly drives the external high-speed DACs and ADCs.

Note: This Verilog DSP core shares its DNA with our sister-project smiSDR (for the Raspberry Pi). Only the bus-interface wrapper changes! So for more details, visit: https://github.com/radiolab81/smisdr/tree/main/gateware
