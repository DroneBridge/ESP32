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

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/

/**
 * For simple debugging when serial via JTAG is enabled. Printed once control module configured USB serial socket.
 * Write settings to JTAG/USB, so we can debug issues better
 */
void db_jtag_serial_info_print();

/**
 * Write settings to non-volatile memory so they can be loaded on next startup. The UDP clients are saved using a
 * separate function since the "save" operation is triggered by a separate button on the UI.
 */
void db_write_settings_to_nvs();

/**
 * Saves a udp client to the NVM so it can be automatically added on the next boot. No need for the user to manually add it
 * again. Only one UDP client can be saved to the NVM.
 * @param new_db_udp_client The client to add to NVM. Must have IP and port set.
 * @param clear_client Set to true to remove the current client from NVM. In that case the new_db_udp_client param will be
 * ignored.
 */
void save_udp_client_to_nvm(struct db_udp_client_t *new_db_udp_client, bool clear_client);

/**
 * Enables or disables (via reboot) the WiFi if the DB_DISABLE_RADIO_ARMED parameter is set. Not used during boot.
 * Usually called when arm state change of the autopilot is detected. As internal check if the WiFi is already
 * enabled/disabled WiFi must be inited first (done during boot).
 * @param enable_wifi True to enable the WiFi and FALSE to disable it
 */
void db_set_wifi_status(uint8_t enable_wifi);

#endif // DB_ESP32_MAIN_H
