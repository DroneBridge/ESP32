/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2024 Wolfgang Christl
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

#ifndef DB_ESP32_DB_SERIAL_H
#define DB_ESP32_DB_SERIAL_H

#include "msp_ltm_serial.h"

#define UART_NUM UART_NUM_1

int open_serial_socket();
void write_to_uart(const uint8_t data_buffer[], const unsigned int data_length);
void parse_msp_ltm(int tcp_clients[], struct udp_conn_list_t *udp_connection, uint8_t msp_message_buffer[],
                   unsigned int *serial_read_bytes,
                   msp_ltm_port_t *db_msp_ltm_port);
void parse_transparent(int tcp_clients[], struct udp_conn_list_t *udp_connection, uint8_t serial_buffer[],
                       unsigned int *serial_read_bytes);

#endif //DB_ESP32_DB_SERIAL_H
