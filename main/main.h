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

#ifndef DB_ESP32_MAIN_H
#define DB_ESP32_MAIN_H

#ifdef CONFIG_DB_HAS_RF_SWITCH
#define DB_HAS_RF_SWITCH 1
#else
#define DB_HAS_RF_SWITCH 0
#endif

/**
 * @brief Enumeration for radio modes
 */
typedef enum {
  DB_WIFI_MODE_AP         = 1, /**< Wi-Fi access point mode with 802.11b mode enabled */
  DB_WIFI_MODE_STA        = 2, /**< Wi-Fi client mode with 802.11b and LR mode enabled */
  DB_WIFI_MODE_AP_LR      = 3, /**< ESP32 WiFi LR Mode 802.11b */
  DB_WIFI_MODE_ESPNOW_AIR = 4, /**< ESP-NOW mode for broadcasting device */
  DB_WIFI_MODE_ESPNOW_GND = 5, /**< ESP-NOW mode for ground station */
  DB_WIFI_MODE_ESPNOW_END = 6, /**< End of ESP-NOW mode enum */
  DB_BLUETOOTH_MODE_SPP   = 7  /**< Bluetooth SPP (Serial Port Profile) mode */
} E_DB_WIFI_MODE;

/**
 * @brief Enumeration for different serial communication protocols.
 */
enum E_DB_SERIAL_PROTOCOL {
  DB_SERIAL_PROTOCOL_MSPLTM      = 1, /**< MSP/LTM (MultiWii Serial Protocol / Lightweight Telemetry) */
  DB_SERIAL_PROTOCOL_MAVLINK     = 4, /**< MAVLink protocol for UAV communication */
  DB_SERIAL_PROTOCOL_TRANSPARENT = 5  /**< Transparent serial data forwarding */
};

void db_jtag_serial_info_print();
void db_write_settings_to_nvs();
void save_udp_client_to_nvm(struct db_udp_client_t *new_db_udp_client, bool clear_client);
void db_set_wifi_status(uint8_t enable_wifi);

#endif // DB_ESP32_MAIN_H
