/******************************************************************************
 * @file db_esp32_main.h
 * @brief DroneBridge ESP32 Main Header File
 *
 * This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 * This file declares core utility and system-level functions used in the
 * DroneBridge ESP32 firmware, including JTAG info, NVS write operations,
 * RF switch configuration, and radio status handling.
 *
 * @copyright
 * Copyright 2018 Wolfgang Christl
 * Licensed under the Apache License, Version 2.0 (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#ifndef DB_ESP32_MAIN_H
#define DB_ESP32_MAIN_H

#include "db_esp32_control.h"

/******************************************************************************
 * Configuration Macros
 ******************************************************************************/

/******************************************************************************
 * @brief Enable RF switch support if configured in Kconfig
 ******************************************************************************/
#ifdef CONFIG_DB_HAS_RF_SWITCH
#define DB_HAS_RF_SWITCH 1
#else
#define DB_HAS_RF_SWITCH 0
#endif

/******************************************************************************
 * Public Function Declarations
 ******************************************************************************/

/******************************************************************************
 * For simple debugging when serial via JTAG is enabled. Printed once control
 * module configured USB serial socket. Write settings to JTAG/USB, so we can
 * debug issues better
 ******************************************************************************/
void db_jtag_serial_info_print(void);

/******************************************************************************
 * Write settings to non-volatile memory so they can be loaded on next
 * startup. The UDP clients are saved using a separate function since the
 * "save" operation is triggered by a separate button on the UI.
 ******************************************************************************/
void db_write_settings_to_nvs(void);

/******************************************************************************
 * Saves an udp client to the NVM so it can be automatically added on the next
 * boot. No need for the user to manually add it again. Only one UDP client can
 * be saved to the NVM.
 * @param new_db_udp_client The client to add to NVM. Must have IP and port
 * set.
 * @param clear_client Set to true to remove the current client from NVM. In
 * that case the new_db_udp_client param will be ignored.
 ******************************************************************************/
void db_save_udp_client_to_nvm(struct db_udp_client_t *new_db_udp_client,
                               bool clear_client);

/******************************************************************************
 * Enables or disables (via reboot) the Wi-Fi/BLE if the
 * DB_PARAM_DIS_RADIO_ON_ARM parameter is set. Not used during boot. Usually
 * called when arm state change of the autopilot is detected. As internal check
 * if the Wi-Fi is already enabled/disabled Wi-Fi must be inited first (done
 * during boot).
 *
 * This is handy for modes like BLE & WiFi AP which are used to configure the
 * drone and usually do not act as a long range telemetry link. In these cases
 * interference on the drone can be reduced by disabling the radio.
 *
 * @param enable_wifi True to enable the Wi-Fi and FALSE to disable it
 ******************************************************************************/
void db_set_radio_status(uint8_t enable_wifi);

#endif /* DB_ESP32_MAIN_H */
