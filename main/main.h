/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2018 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */


#ifndef DB_ESP32_MAIN_H
#define DB_ESP32_MAIN_H


enum E_DB_WIFI_MODE {
    DB_WIFI_MODE_AP = 1,            // Wi-Fi access point mode with Wi-Fi B mode enabled
    DB_WIFI_MODE_STA = 2,           // Wi-Fi client mode with Wi-Fi B and LR mode enabled
    DB_WIFI_MODE_AP_LR = 3,         // ESP32 LR Mode
    DB_WIFI_MODE_ESPNOW_AIR = 4,    // ESP-NOW Mode for broadcasting device - automatically enables Wi-Fi B and LR mode
    DB_WIFI_MODE_ESPNOW_GND = 5,    // ESP-NOW Mode for GND station - automatically enables Wi-Fi B and LR mode
};

void write_settings_to_nvs();

#endif //DB_ESP32_MAIN_H
