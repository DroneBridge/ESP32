![DroneBridge logo](https://github.com/DroneBridge/ESP32/blob/master/wiki/DroneBridgeLogo_text.png)

# DroneBridge for ESP32
DroneBridge enabled firmware for the popular ESP32 modules form Espressif Systems. Probably the cheapest way to communicate with your drone.

![DroneBridge for ESP32 concept](https://github.com/DroneBridge/ESP32/blob/master/wiki/db_ESP32_setup.png)

## Features
 - Bi-directional link: **MAVLink, MSP & LTM**
 - Very low price: **~7€**
 - Up to **150m** range
 - **Weight: <10 g**
 - Supported by: **DroneBridge for Android (app), mwptools, QGroundcontrol, etc.**
 - **Easy to set up**: Power connection + UART connection to flight controller
 - **Fully configurable through easy to use web interface**
 - **Parsing of LTM & MSPv2** for more reliable connection and less packet loss
 - **Fully transparent telemetry downlink option** for continuous streams like MAVLink or and other protocol
 - Reliable, low latency, light weight
 - Upload mission etc.

![ESP32 module with VCP](https://github.com/DroneBridge/ESP32/blob/master/wiki/esp32_vcp_module.jpg)

Tested with: DOIT ESP32 module

 ## Setup
 ### Flashing the firmware using the precompiled binarys
  1. Download latest release from this repository or check out `db_esp32\releases` folder.
  2. Get the [latest Flash Download Tools from Espressif](https://www.espressif.com/en/products/hardware/esp32/resources)
  3. Settings
    - `bootloader.bin 0x1000`
    - `partitions_singleapp.bin 0x8000`
    - `dronebridge_esp32.bin 0x10000`
  4. During flasing you might need to hold the "boot" button for 2-3 seconds because some drivers have issues with the process.
  5. If you dont get it check out google or [this link](http://iot-bits.com/esp32/esp32-flash-download-tool-tutorial/)

### Wiring
TODO

Connect UART of ESP32 to a 3.3V UART of your flight controller. Set the flight controller port to the desired protocol. (Power the ESP32 module with a stable 5-12V power source) **Check out manufacturer datasheet! Only some modules can take more than 3.3V/5V on VIN PIN**

### Configuration
 1. Connect to the wifi `DroneBridge ESP32` with password `dronebridge`
 2. In your browser type: `dronebridge.local` (Chrome: `http://dronebridge.local`) or `192.168.2.1` into the address bar
 3. Configure as you please and hit `save`

![DroneBridge for ESP32 web interface](https://github.com/DroneBridge/ESP32/blob/master/wiki/screen_config.png)

**Configuration Options:**
 - **`Wifi password`: Up to 64 character long
 - **`UART baud rate`: Same as you configured on your flight controller
 - **`GPIO TX PIN Number` & `GPIO RX PIN Number`: The pins you want to use for TX & RX (UART). See pin out of manufacturer of your ESP32 device **Flight controller UART must be 3.3V or use an inverter.**
 - `UART serial protocol`: MultiWii based or MAVLink based - configures the parser
 - `Transparent packet size`: Only used with 'serial protocol' set to transparent. Length of UDP packets
 - `LTM frames per packet`: Buffer the specified number of packets and send them at once in one packet
 - `MSP & LTM to same port`: Split MSP & LTM stream or send both to same port. Set to `Yes` if you want to use `mwptools`. Set to `No` if you use DroneBridge software (app etc.)

** Require restart/reset of ESP32 module

## Use with DroneBridge for Android
![DroneBridge for Android app screenshot](https://github.com/DroneBridge/ESP32/blob/master/wiki/dp_app-map-2017-10-29-kleiner.png)

Use the Android app to display live telemetry data. Mission planning capabilities for MAVLink and MSP will follow.

## Compile yourselfe (developers)

 You will need the Espressif SDK: esp-idf + toolchain. Check out their website for more info and on how to set it up.
 The code is written in pure C using the esp-idf (no arduino libs).

 Compile and flash by running: `make`, `make flash`
