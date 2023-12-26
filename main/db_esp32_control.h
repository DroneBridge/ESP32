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

#define MAX_UDP_CLIENTS 8
#define TRANS_RD_BYTES_NUM  8   // amount of bytes read form serial port at once when transparent is selected
#define UDP_BUF_SIZE    2048
#define UART_BUF_SIZE   (1024)

// per client structure of connected devices in softAP mode
struct db_udp_client_t {
    uint8_t mac[6];     // MAC address of connected client
    struct sockaddr_in udp_client;    // socket address (IP & PORT) of connected client
};

// UDP connection stucture
struct db_udp_connection_t {
    int udp_socket;     // ID of UDP socket
    struct db_udp_client_t db_udp_clients[MAX_UDP_CLIENTS];
};

void control_module();
void add_udp_to_known_clients(struct db_udp_connection_t *connections, struct db_udp_client_t new_db_udp_client);
void remove_udp_from_known_clients(struct db_udp_connection_t *connections, struct db_udp_client_t db_udp_client);

#endif //DB_ESP32_DB_ESP32_CONTROL_H
