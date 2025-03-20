/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2019 Wolfgang Christl
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

#ifndef DB_ESP32_TCP_SERVER_H
#define DB_ESP32_TCP_SERVER_H
/***************************************************************************************************************************
 * Public Macros
 **************************************************************************************************************************/
#define TCP_BUFF_SIZ 4096

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/
/***************************************************************************************************************************
 * @brief Opens a TCP server socket on the specified port.
 *
 * This function creates and binds a TCP socket to listen for incoming connections.
 *
 * @param port The port number on which the TCP server should listen.
 * @return The socket file descriptor on success, or ESP_FAIL on failure.
 **************************************************************************************************************************/
int open_tcp_server(int port);

/***************************************************************************************************************************
 * @brief Sends data to all connected TCP clients.
 *
 * Iterates through the list of active TCP client sockets and sends the given data.
 *
 * @param tcp_clients Array of active TCP client sockets.
 * @param data Pointer to the data buffer to be sent.
 * @param data_length The length of the data buffer.
 **************************************************************************************************************************/
void db_send_to_all_tcp_clients(const int tcp_clients[], uint8_t data[], uint data_length);

#endif // DB_ESP32_TCP_SERVER_H
