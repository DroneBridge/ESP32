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

#define TCP_BUFF_SIZ 4096

int open_tcp_server(int port);
void db_send_to_all_tcp_clients(const int tcp_clients[], uint8_t data[], uint data_length);

#endif //DB_ESP32_TCP_SERVER_H
