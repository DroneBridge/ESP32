![DroneBridge logo](https://github.com/DroneBridge/ESP32/blob/master/wiki/DroneBridgeLogo_text.png)

# DroneBridge for ESP32
DroneBridge enabled firmware for the popular ESP32 modules from Espressif Systems. Probably the cheapest way to 
communicate with your drone, UAV, UAS, ground based vehicle or whatever you may call them.

Also allows for a fully transparent serial to wifi pass through with variable packet size 
(Continuous stream of data required).

![DroneBridge for ESP32 concept](https://github.com/DroneBridge/ESP32/blob/master/wiki/db_ESP32_setup.png)

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

![ESP32 module with VCP](https://github.com/DroneBridge/ESP32/blob/master/wiki/esp32_vcp_module.jpg)

Tested with: DOIT ESP32 module

## Setup
### Flashing the firmware using the precompiled binaries

First download the latest release from this repository. 
[You can find them here](https://github.com/DroneBridge/ESP32/releases).

For flashing there are many ways of doing this. To easy ones are shown below. **You might need to press the reset/boot 
button on your ESP to start the upload/flash process-**

#### Windows only: Use flash download tools

[Get it here](https://www.espressif.com/en/support/download/other-tools)

#### Use Espressif firmware flashing tool

   1. `pip install esptool`
   2. Connect via USB/Serial. Find out the serial port via `dmesg` on linux or device manager on windows. 
   In this example the serial connection to the ESP32 is on COM4.
   3. `esptool.py --port COM4 write_flash 0x1000 db_esp32.bin`

[Look here for more detailed information](https://github.com/espressif/esptool)

### Wiring

Connect UART of ESP32 to a 3.3V UART of your flight controller. Set the flight controller port to the desired protocol. 
(Power the ESP32 module with a stable 5-12V power source) **Check out manufacturer datasheet! Only some modules can 
take more than 3.3V/5V on VIN PIN**

Defaults: UART2 (RX2, TX2 on GPIO 16, 17)

### Configuration
 1. Connect to the wifi `DroneBridge ESP32` with password `dronebridge`
 2. In your browser type: `dronebridge.local` (Chrome: `http://dronebridge.local`) or `192.168.2.1` into the address bar. 
 **You might need to disable the cellular connection to force the browser to use the wifi connection**
 3. Configure as you please and hit `save`

![DroneBridge for ESP32 web interface](https://github.com/DroneBridge/ESP32/blob/master/wiki/screen_config.png)

**Configuration Options:**
 - **`Wifi password`: Up to 64 character long
 - **`UART baud rate`: Same as you configured on your flight controller
 - **`GPIO TX PIN Number` & `GPIO RX PIN Number`: The pins you want to use for TX & RX (UART). See pin out of manufacturer of your ESP32 device **Flight controller UART must be 3.3V or use an inverter.**
 - `UART serial protocol`: MultiWii based or MAVLink based - configures the parser
 - `Transparent packet size`: Only used with 'serial protocol' set to transparent. Length of UDP packets
 - `LTM frames per packet`: Buffer the specified number of packets and send them at once in one packet

** Require restart/reset of ESP32 module

## Use with DroneBridge for Android or QGroundControl
![DroneBridge for Android app screenshot](https://github.com/DroneBridge/ESP32/blob/master/wiki/dp_app-map-2017-10-29-kleiner.png)

 - Use the Android app to display live telemetry data. Mission planning capabilities for MAVLink will follow.
 - Connect via **TCP on port 14556** to the ESP32 to send & receive data with a GCS of your choice

## Compile yourself (developers)

 You will need the Espressif SDK: esp-idf + toolchain (compile it yourself). Check out their website for more info and on how to set it up.
 The code is written in pure C using the esp-idf (no arduino libs). 
 
 **This project uses the v4.0 branch of ESP-IDF**

 Compile and flash by running: `make`, `make flash`
