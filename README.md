# parlioSDR  [ESP32 based Software-Defined-Radio TX]

Baseband/RF over Ethernet - powered by ESP32-P4 PARLIO/Parallel IO Interface (DMA - driven).

![main01](https://github.com/radiolab81/parlioSDR/blob/main/www/schematic.jpg)

For high-speed connection of a DAC/ADC in SDR applications, some new MCUs from espressif, like ESP32-P4, offers the PARLIO/Parallel IO Interface.
https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/parlio/index.html

This interface is very well suited for price-sensitive experiments (rx and tx) in the field of software-defined radio.
The project shown here is a feasibility study on how to send such baseband or RF data from an application like GNU Radio or Cohiradia to an ESP32-P4 via Ethernet.

Although Espressif's new microcontrollers are Gigabit-capable and can therefore be a good alternative to the smisdr (Raspberry 4 / https://github.com/radiolab81/smisdr), the existing dev-boards are not making use of this yet, so the bandwidth is still somewhat limited, but it is sufficient for projects such as AMWaveSynth (https://github.com/radiolab81/AMWaveSynth) or Cohiradia (https://github.com/radiolab81/COHIRADIAStreamer). As an alternative to Ethernet, we are planning to evaluate the USB 2.0 interface, which should overcome this bandwidth limitation for many application areas.

The parlioSDR represents a very cost-effective approach/replacement to the obsolete FL2000 USB-VGA adapter (8-bit only) in some cases. Usable DACs range from simple 8-10 bit R2R-ladder-DACs to common 12-14 bit parallel-DACs (and ADCs) from manufacturers like Analog, Microchip, or TI. The ability to easily build your own HATs with the desired bit width provides access to all frequency ranges within signal bandwidth corresponding to the PARLIO-Interface.
