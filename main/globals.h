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

#ifndef DB_ESP32_GLOBALS_H
#define DB_ESP32_GLOBALS_H

#include <freertos/event_groups.h>
#include <esp_wifi_types.h>
#include "db_esp32_control.h"

extern char CURRENT_CLIENT_IP[IP4ADDR_STRLEN_MAX];  // IP address of the ESP32 when we are in client mode connected

extern uint8_t DB_WIFI_IS_OFF;
extern db_esp_signal_quality_t db_esp_signal_quality;   // used on AIR/station side to store RSSI information
extern wifi_sta_list_t wifi_sta_list;      // updated when ESP32 is in ap mode. Contains RSSI of every connected station
extern uint8_t LOCAL_MAC_ADDRESS[6];       // filled with the mac address during init of WiFi interface
extern uint8_t DB_MAV_SYS_ID;              // stores the local system ID - set by heartbeat that is received via UART (UART is the local connection)

extern uint32_t serial_total_byte_count;                // Total bytes read from serial link (UART or USB/JTAG)
extern int8_t num_connected_tcp_clients;
extern udp_conn_list_t *udp_conn_list;   // List of UDP clients that the ESP32 will send to

// extern int WIFI_ESP_MAXIMUM_RETRY;

#endif //DB_ESP32_GLOBALS_H
