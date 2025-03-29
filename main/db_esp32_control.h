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

#ifndef DB_ESP32_DB_ESP32_CONTROL_H
#define DB_ESP32_DB_ESP32_CONTROL_H

#include <lwip/sockets.h>

#define MULTICAST_IPV4_ADDR "232.10.11.12" // "224.0.0.1"
#define MULTICAST_TTL 1
#define MAX_UDP_CLIENTS 8
#define TRANS_RD_BYTES_NUM  8   // amount of bytes read form serial port at once when transparent is selected
#define UDP_BUF_SIZE    2048
#define UART_BUF_SIZE   (1024)

// per client structure of connected devices in softAP mode
struct db_udp_client_t {
    uint8_t mac[6];     // MAC address of connected client
    struct sockaddr_in udp_client;    // socket address (IP & PORT) of connected client
};

typedef struct udp_conn_list_s {
    struct db_udp_client_t db_udp_clients[MAX_UDP_CLIENTS]; // The array of list items
    int size; // The number of items in the list
    int udp_socket;     // ID of UDP socket
} udp_conn_list_t;

// Used on the ESP AIR side to keep track and used to fill MAVlink RADIO STATUS msg
typedef struct {
    int8_t air_rssi;            // [dBm] RSSI of received data from AP. Updated when ESP32 is in station mode and connected to an access point
    int8_t gnd_rssi;            // [dBm] AP/GND told the rssi he is seeing when receiving our packets. Updated on every DroneBridge internal telemetry frame from GND
    int8_t air_noise_floor;     // [dBm] Noise floor on air side. Updated when ESP32 is in ESP-NOW mode and receives packet - Not supported by all ESP32 variants
    int8_t gnd_noise_floor;     // [dBm] AP/GND told the noise floor he is seeing when receiving our packets. Updated on every DroneBridge internal telemetry frame from GND - Not supported by all ESP32 variants
    uint16_t gnd_rx_packets_lost;   // Number of ESP-NOW packets the GND station lost coming from this AIR peer (based on seq. number)
} db_esp_signal_quality_t;

void db_start_control_module();
udp_conn_list_t *udp_client_list_create();
void udp_client_list_destroy(udp_conn_list_t *n_udp_conn_list);
bool add_to_known_udp_clients(udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client, bool save_to_nvm);
void db_send_to_all_clients(int tcp_clients[], udp_conn_list_t *n_udp_conn_list, uint8_t data[], uint16_t data_length);
bool remove_from_known_udp_clients(udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client);

#endif //DB_ESP32_DB_ESP32_CONTROL_H
