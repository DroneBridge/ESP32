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

#define UART_NUM UART_NUM_1             // The UART interface of the ESP32 we use
#define DB_SERIAL_READ_TIMEOUT_MS_DEFAULT  50      // Serial read timeout for transparent and MAVLink mode, after that the packet will be sent over the air even when the max. packet size was not reached.

enum DB_MAVLINK_DATA_ORIGIN {
    DB_MAVLINK_DATA_ORIGIN_SERIAL,
    DB_MAVLINK_DATA_ORIGIN_RADIO
};

typedef union {
    float f;
    uint8_t uint8;
    int8_t int8;
    uint16_t uint16;
    int16_t int16;
    uint32_t uint32;
    int32_t int32;
} float_int_union;

int open_serial_socket();
void write_to_serial(const uint8_t data_buffer[], unsigned int data_length);
void db_parse_msp_ltm(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t msp_message_buffer[],
                      unsigned int *serial_read_bytes,
                      msp_ltm_port_t *db_msp_ltm_port);
void db_read_serial_parse_mavlink(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *serial_buffer, unsigned int *serial_buff_pos);
void db_read_serial_parse_transparent(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t serial_buffer[],
                                      unsigned int *serial_read_bytes);
void db_parse_mavlink_from_radio(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *buffer, int bytes_read);
void db_route_mavlink_response(uint8_t *buffer, uint16_t length, enum DB_MAVLINK_DATA_ORIGIN origin, int *tcp_clients,
                               udp_conn_list_t *udp_conns);

#endif //DB_ESP32_DB_SERIAL_H
