# parlioSDR  [ESP32 based Software-Defined-Radio TX]

Baseband/RF over Ethernet - powered by ESP32-P4 PARLIO/Parallel IO Interface (DMA - driven).

![main01](https://github.com/radiolab81/parlioSDR/blob/main/www/schematic.jpg)

For high-speed connection of a DAC/ADC in SDR applications, some new MCUs from espressif, like ESP32-P4, offers the PARLIO/Parallel IO Interface.
https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/parlio/index.html

This interface is very well suited for price-sensitive experiments (rx and tx) in the field of software-defined radio.
The project shown here is a feasibility study on how to send such baseband or RF data from an application like GNU Radio or Cohiradia to an ESP32-P4 via Ethernet.

Although Espressif's new microcontrollers are Gigabit-capable and can therefore be a good alternative to the smisdr (Raspberry 4 / https://github.com/radiolab81/smisdr), the existing dev-boards are not making use of this yet, so the bandwidth is still somewhat limited, but it is sufficient for projects such as AMWaveSynth (https://github.com/radiolab81/AMWaveSynth) or Cohiradia (https://github.com/radiolab81/COHIRADIAStreamer). As an alternative to Ethernet, we are planning to evaluate the USB 2.0 interface, which should overcome this bandwidth limitation for many application areas.

The parlioSDR represents a very cost-effective approach/replacement to the obsolete FL2000 USB-VGA adapter (8-bit only) in some cases. Usable DACs range from simple 8-10 bit R2R-ladder-DACs to common 12-14 bit parallel-DACs (and ADCs) from manufacturers like Analog, Microchip, or TI. The ability to easily build your own HATs with the desired bit width provides access to all frequency ranges within signal bandwidth corresponding to the PARLIO-Interface.

The project is build with actual ESP-IDF 6.1, so this few lines can do the job

```console
idf.py build

idf.py -p YOUR_PORT flash

idf.py -p YOUR_PORT monitor
```
Immediately after starting, parlioSDR waits for an IP address, which should preferably be provided via DHCP.

```
I (2157) esp_eth.netif.netif_glue: ethernet attached to netif
I (4857) PARLIOSDR: Hardware Loop neugestartet: 8 Bit, 5.00 MSPS
I (4857) main_task: Returned from app_main()
I (4857) PARLIOSDR: Warte auf TCP Daten-Verbindung (Port 1234)...
I (4857) PARLIOSDR: Control-Server lauscht auf Port 5000
I (7357) esp_netif_handlers: eth ip: 192.168.1.71, mask: 255.255.255.0, gw: 192.168.1.1
I (7357) PARLIOSDR: Ethernet OK! IP: 192.168.1.71
```
Like the smisdr, it expects the RF data stream on port 1234, and can listen for the same control commands via port 5000, just like its larger Raspberry Pi sibling.

for example:

#### set sample rate to 5 MSPS
echo -n "rate 5" | nc -w 1 192.168.1.71 5000

#### set sample rate to 10 MSPS
echo -n "rate 10" | nc -w 1 192.168.1.71 5000

#### set sample rate to 12.5 MSPS
echo -n "rate 12.5" | nc -w 1 192.168.1.71 5000

#### set 8 bit dac width
echo -n "width 8" | nc -w 1 192.168.1.71 5000

#### set 16 bit dac width
echo -n "width 16" | nc -w 1 192.168.1.71 5000

Apps like COHIRADIAStreamer (https://github.com/radiolab81/COHIRADIAStreamer) will do the config on-the-fly.

The pin assignment of the header to the DAC (or ADC) is shown in th esp32_p4_***.h files.


