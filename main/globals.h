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

#define MAX_LTM_FRAMES_IN_BUFFER 5

#define DB_BUILD_VERSION 15
#define DB_MAJOR_VERSION 2
#define DB_MINOR_VERSION 1
#define DB_PATCH_VERSION 0
#define DB_MATURITY_VERSION "RC1"

/* DroneBridge Parameters */
extern uint8_t DB_RADIO_MODE;    // never change this value while the ESP32 is running, will likely lead to a crash. Assign it during startup when received from storage. Can therefore only be changed via reboot and DB_WIFI_MODE_DESIGNATED
extern uint8_t DB_RADIO_MODE_DESIGNATED; // introduced and used for settings change only since changing DB_WIFI_MODE directly while system is live will eventually lead to crashes. Stores the new WiFi mode and will be written to the settings. This means it will become active after reboot
extern uint8_t DB_WIFI_SSID[32];
extern uint8_t DB_WIFI_PWD[64];
extern char DEFAULT_AP_IP[IP4ADDR_STRLEN_MAX];
extern char CURRENT_CLIENT_IP[IP4ADDR_STRLEN_MAX];  // IP address of the ESP32 when we are in client mode connected
extern uint8_t DB_WIFI_CHANNEL;
extern uint8_t DB_WIFI_EN_GN;  // disabled: only 802.11b support for client mode - set to true: 802.11bgn mode support
extern uint8_t DB_EN_EXT_ANT;      // set to 1 to use external antenna. Set to 0 to enable onboard antenna - board must have antenna switch
extern uint8_t DB_SERIAL_PROTOCOL;  // 1=MSP, 4=MAVLink, 5=Transparent
extern uint8_t DB_DISABLE_RADIO_ARMED;   // 0 = WiFi remains enabled always, 1 = WiFi will be turned off when armed state is detected on the telemetry stream. Does not work when DB_SERIAL_PROTOCOL is set to transparent
extern uint8_t DB_UART_PIN_TX;      // set TX & RX pin to the same number to indicate vanilla system
extern uint8_t DB_UART_PIN_RX;
extern uint8_t DB_UART_PIN_RTS;
extern uint8_t DB_UART_PIN_CTS;
extern uint8_t DB_UART_RTS_THRESH;
extern int32_t DB_UART_BAUD_RATE;
extern uint16_t DB_TRANS_BUF_SIZE;          // Maximum packet size via ESP-NOW or WiFi in transparent or mavlink mode
extern uint16_t DB_SERIAL_READ_TIMEOUT_MS;  // Serial read timeout for transparent and MAVLink mode, after that the packet will be sent over the air even when the max. packet size was not reached.
extern uint8_t DB_LTM_FRAME_NUM_BUFFER;    // Number of LTM frames per UDP packet (min = 1; max = 5)
extern char DB_STATIC_STA_IP[IP4ADDR_STRLEN_MAX];   // user can specify static IP when in Wi-Fi client mode. If this is empty use auto IP
extern char DB_STATIC_STA_IP_GW[IP4ADDR_STRLEN_MAX];// if DB_STATIC_STA_IP is set then this must be set to the GW IP
extern char DB_STATIC_STA_IP_NETMASK[IP4ADDR_STRLEN_MAX]; // netmask when settings static IP in Wi-Fi client mode

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
