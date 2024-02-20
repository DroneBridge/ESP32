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

#include <driver/gpio.h>
#include <esp_wifi_types.h>
#include <lwip/apps/netbiosns.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>

#include "db_esp32_comm.h"
#include "db_esp32_control.h"
#include "db_protocol.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_spiffs.h"
#include "esp_vfs_semihost.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "freertos/event_groups.h"
#include "http_server.h"
#include "mdns.h"

#define NVS_NAMESPACE "settings"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

enum E_DB_NETIF_MODE {
    DB_WIFI_MODE_AP = 1,
    DB_WIFI_MODE_STA = 2,
    DB_WIFI_MODE_AP_LR = 3,
    DB_ETH_MODE = 4
};

void write_settings_to_nvs();

#endif  // DB_ESP32_MAIN_H
