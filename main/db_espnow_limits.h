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

#ifndef DB_ESP32_DB_ESPNOW_LIMITS_H
#define DB_ESP32_DB_ESPNOW_LIMITS_H

/** Maximum number of broadcast AIR peers supported by the ESP-NOW runtime. */
#define DB_ESPNOW_MAX_BROADCAST_PEERS 19U // 19 peers is the limit, we cannot fit more clients into one ESP-NOW internal telemetry packet

/** Number of bytes in an ESP-NOW MAC address. */
#define DB_ESPNOW_MAC_ADDR_LEN 6U

#endif // DB_ESP32_DB_ESPNOW_LIMITS_H
