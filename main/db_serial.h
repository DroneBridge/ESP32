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
#include "db_esp32_control.h"

#define UART_NUM UART_NUM_1

enum DB_SERIAL_DATA_ORIGIN {
    DB_SERIAL_DATA_ORIGIN_FC,
    DB_SERIAL_DATA_ORIGIN_GCS
};

int open_serial_socket();
void write_to_serial(const uint8_t data_buffer[], unsigned int data_length);
void db_parse_msp_ltm(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t msp_message_buffer[],
                      unsigned int *serial_read_bytes,
                      msp_ltm_port_t *db_msp_ltm_port);
void db_read_serial_parse_mavlink(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *serial_buffer, unsigned int *serial_buff_pos);
void db_read_serial_parse_transparent(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t serial_buffer[],
                                      unsigned int *serial_read_bytes);

#endif //DB_ESP32_DB_SERIAL_H
