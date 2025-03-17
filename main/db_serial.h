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

#include "db_esp32_control.h"
#include "msp_ltm_serial.h"

#define UART_NUM UART_NUM_1 // The UART interface of the ESP32 we use
#define DB_SERIAL_READ_TIMEOUT_MS_DEFAULT                                                                                   \
  50 // Serial read timeout for transparent and MAVLink mode, after that the packet will be sent over the air even when the
     // max. packet size was not reached.

enum DB_MAVLINK_DATA_ORIGIN { DB_MAVLINK_DATA_ORIGIN_SERIAL, DB_MAVLINK_DATA_ORIGIN_RADIO };

typedef union {
  float f;
  uint8_t uint8;
  int8_t int8;
  uint16_t uint16;
  int16_t int16;
  uint32_t uint32;
  int32_t int32;
} float_int_union;

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/

/***************************************************************************************************************************
 * Opens a serial socket for communication with a serial source. On the GND this is the GCS and on the air side this is the
 * flight controller. Depending on the configuration it may open a native UART socket or a JTAG based serial interface. The
 * JTAG serial based interface is a special feature of official DroneBridge for ESP32 boards. Uses the onboard USB for serial
 * I/O with GCS. No FTDI required.
 *
 * @return ESP_FAIL on failure
 **************************************************************************************************************************/
esp_err_t open_serial_socket();

/***************************************************************************************************************************
 * Writes data from buffer to the opened serial device
 * @param data_buffer Payload to write to UART
 * @param data_length Size of payload to write to UART
 **************************************************************************************************************************/
void write_to_serial(const uint8_t data_buffer[], unsigned int data_length);

/***************************************************************************************************************************
 * @brief Reads serial interface, parses & sends complete MSP & LTM messages over the air.
 **************************************************************************************************************************/
void db_parse_msp_ltm(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t msp_message_buffer[],
                      unsigned int *serial_read_bytes, msp_ltm_port_t *db_msp_ltm_port);

/***************************************************************************************************************************
 * Parses MAVLink messages and sends them via the radio link.
 * This function reads data from the serial interface, parses it for complete MAVLink messages, and sends those messages in a
 * buffer. It ensures that only complete messages are sent and that the buffer does not exceed TRANS_BUFF_SIZE. Checks for a
 * serial read timeout. In case timeout is reached all data read from serial so far will be flushed to radio interface The
 * parsing is done semi-transparent as in: parser understands the MavLink frame format but performs no further checks
 *
 * @param tcp_clients Array of connected TCP clients
 * @param up_conns Structure containing all UDP connection data including the sockets
 * @param serial_buffer Buffer that gets filled with data and then sent via radio, shall be >x2 the max payload
 * @param serial_buff_pos Number of bytes already read for the current packet
 **************************************************************************************************************************/
void db_read_serial_parse_mavlink(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *serial_buffer,
                                  unsigned int *serial_buff_pos);

/***************************************************************************************************************************
 * Reads TRANS_RD_BYTES_NUM bytes from serial interface and checks if we already got enough bytes to send them out.
 * Timeout ensures that no data is getting stuck in the buffer. Once serial read timeout is reached, the buffer will be
 * flushed to the radio interface (send what we have)
 *
 * @param tcp_clients Array of connected TCP clients
 * @param udp_connection Structure containing all UDP connection data including the sockets
 * @param serial_buffer Buffer that gets filled with data and then sent via radio
 * @param serial_read_bytes Number of bytes already read for the current packet
 **************************************************************************************************************************/
void db_read_serial_parse_transparent(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t serial_buffer[],
                                      unsigned int *serial_read_bytes);

/***************************************************************************************************************************
 * Parses MAVLink coming from WiFi/ESPNOW - and sends the packet to the serial output.
 *
 * @param tcp_clients Array of connected TCP clients
 * @param up_conns Structure containing all UDP connection data including the sockets
 * @param buffer Buffer containing the raw bytes to be parsed
 * @param bytes_read Number of bytes in the buffer
 * @param origin Origin of the data - serial link or radio link
 **************************************************************************************************************************/
void db_parse_mavlink_from_radio(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *buffer, int bytes_read);

/**
 * We received some MAVLink request via the origin. Decide on which interface to respond with an answer
 * @param buffer Data to send
 * @param length Data length to send
 * @param origin Origin of the MAVLink request
 * @param tcp_clients List of connected TCP client
 * @param udp_conns List of active UDP connections
 */
void db_route_mavlink_response(uint8_t *buffer, uint16_t length, enum DB_MAVLINK_DATA_ORIGIN origin, int *tcp_clients,
                               udp_conn_list_t *udp_conns);

#endif // DB_ESP32_DB_SERIAL_H
