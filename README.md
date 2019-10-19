![DroneBridge logo](https://github.com/DroneBridge/ESP32/blob/master/wiki/DroneBridgeLogo_text.png)

# DroneBridge for ESP32
DroneBridge enabled firmware for the popular ESP32 modules from Espressif Systems. Probably the cheapest way to 
communicate with your drone, UAV, UAS, ground based vehicle or whatever you may call them.

Also allows for a fully transparent serial to wifi pass through with variable packet size 
(Continuous stream of data required).

![DroneBridge for ESP32 concept](https://github.com/DroneBridge/ESP32/blob/master/wiki/db_ESP32_setup.png)

![DroneBridge for ESP32 block diagram blackbox](https://github.com/DroneBridge/ESP32/blob/master/wiki/DroneBridgeForESP32Blackbox.png)

## Features
 - Bi-directional link: MAVLink, MSP & LTM
 - Affordable: ~7€
 - Up to 150m range
 - Weight: <10 g
 - Supported by: DroneBridge for Android (app), mwptools, QGroundControl, impload etc.
 - Easy to set up: Power connection + UART connection to flight controller
 - Fully configurable through easy to use web interface
 - Parsing of LTM & MSPv2 for more reliable connection and less packet loss
 - Fully transparent telemetry downlink option for continuous streams like MAVLink or and other protocol
 - Reliable, low latency, light weight
 - Upload mission etc.

![ESP32 module with VCP](https://upload.wikimedia.org/wikipedia/commons/thumb/2/20/ESP32_Espressif_ESP-WROOM-32_Dev_Board.jpg/313px-ESP32_Espressif_ESP-WROOM-32_Dev_Board.jpg)

Tested with: DOIT ESP32 module

## Installation/Flashing using precompiled binaries

First download the latest release from this repository. 
[You can find them here](https://github.com/DroneBridge/ESP32/releases).

For flashing there are many ways of doing this. To easy ones are shown below.

#### All platforms: Use Espressif firmware flashing tool

**recommended**

   1. `pip install esptool`
   2. Connect via USB/Serial. Find out the serial port via `dmesg` on linux or device manager on windows. 
   In this example the serial connection to the ESP32 is on COM4 (in Linux e.g. `/dev/ttyUSB0`).
   3. `esptool.py -p COM4 -b 460800 --after hard_reset write_flash 0x1000 bootloader.bin 0x8000 partition-table.bin 
   0x10000 db_esp32.bin`. You might need to press the boot 
   button on your ESP to start the upload/flash process. On Windows `esptool [...]` (with out `.py`) seems to work

[Look here for more detailed information](https://github.com/espressif/esptool)

#### Windows only: Use flash download tools

   1. [Get it here](https://www.espressif.com/en/support/download/other-tools)
   2. Select the firmware, bootloader & partition table and set everything as below
   ![ESP download tool configuration](https://github.com/DroneBridge/ESP32/blob/master/wiki/ESP32Flasher.PNG)
   3. Hit Start and power cycle your ESP32 after flashing

### Wiring

   1. Connect UART of ESP32 to a 3.3V UART of your flight controller. 
   2. Set the flight controller port to the desired protocol. 

(Power the ESP32 module with a stable 5-12V power source) **Check out manufacturer datasheet! Only some modules can 
take more than 3.3V/5V on VIN PIN**

Defaults: UART2 (RX2, TX2 on GPIO 16, 17)

### Configuration
 1. Connect to the wifi `DroneBridge ESP32` with password `dronebridge`
 2. In your browser type: `dronebridge.local` (Chrome: `http://dronebridge.local`) or `192.168.2.1` into the address bar. 
 **You might need to disable the cellular connection to force the browser to use the wifi connection**
 3. Configure as you please and hit `save`

![DroneBridge for ESP32 web interface](https://github.com/DroneBridge/ESP32/blob/master/wiki/DroneBridge_for_ESP32_web_interface.png)

**Configuration Options:**
 - `Wifi password`: Up to 64 character long
 - `UART baud rate`: Same as you configured on your flight controller
 - `GPIO TX PIN Number` & `GPIO RX PIN Number`: The pins you want to use for TX & RX (UART). See pin out of manufacturer of your ESP32 device **Flight controller UART must be 3.3V or use an inverter.**
 - `UART serial protocol`: MultiWii based or MAVLink based - configures the parser
 - `Transparent packet size`: Only used with 'serial protocol' set to transparent. Length of UDP packets
 - `LTM frames per packet`: Buffer the specified number of packets and send them at once in one packet

Most options require a restart/reset of ESP32 module

## Use with DroneBridge for Android or QGroundControl
![DroneBridge for Android app screenshot](https://github.com/DroneBridge/ESP32/blob/master/wiki/dp_app-map-2017-10-29-kleiner.png)

 - Use the Android app to display live telemetry data. Mission planning capabilities for MAVLink will follow.
 - Connect via **TCP on port 5760** or **UDP on port 14550** to the ESP32 to send & receive data with a GCS of your choice. **In case of a UDP connection the GCS must send at least one packet (e.g. MAVLink heart beat etc.) to the UDP port of the ESP32 to register as an end point.**

## Compile yourself (developers)

 You will need the Espressif SDK: esp-idf + toolchain (compile it yourself). Check out their website for more info and on how to set it up.
 The code is written in pure C using the esp-idf (no arduino libs). 
 
 **This project uses the v4.0 branch of ESP-IDF**

 Compile and flash by running: `idf.py build`, `idf.py flash`
