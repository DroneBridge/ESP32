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
#include "db_esp32_control.h"

#define MAX_LTM_FRAMES_IN_BUFFER 5
#define BUILDVERSION 10
#define MAJOR_VERSION 1
#define MINOR_VERSION 5

// can be set by user
extern uint8_t DB_WIFI_MODE;
extern uint8_t DEFAULT_SSID[32];
extern uint8_t DEFAULT_PWD[64];
extern char DEFAULT_AP_IP[32];
extern char CURRENT_CLIENT_IP[32];  // IP address of the ESP32 when we are in client mode connected
extern uint8_t DEFAULT_CHANNEL;
extern uint8_t SERIAL_PROTOCOL;  // 1=MSP, 3=MAVLink/transparent
extern uint8_t DB_UART_PIN_TX;      // set TX & RX pin to the same number to indicate vanilla system
extern uint8_t DB_UART_PIN_RX;
extern uint8_t DB_UART_PIN_RTS;
extern uint8_t DB_UART_PIN_CTS;
extern uint8_t DB_UART_RTS_THRESH;
extern int DB_UART_BAUD_RATE;
extern uint16_t TRANSPARENT_BUF_SIZE;
extern uint8_t LTM_FRAME_NUM_BUFFER;    // Number of LTM frames per UDP packet (min = 1; max = 5)
extern int station_rssi;               // updated when ESP32 is in station mode and connected to an access point

extern uint32_t uart_byte_count;
extern int8_t num_connected_tcp_clients;
extern struct udp_conn_list_t *udp_conn_list;

extern int WIFI_ESP_MAXIMUM_RETRY;

#endif //DB_ESP32_GLOBALS_H
