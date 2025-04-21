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

void db_jtag_serial_info_print();
void db_write_settings_to_nvs();
void save_udp_client_to_nvm(struct db_udp_client_t *new_db_udp_client, bool clear_client);
void db_set_radio_status(uint8_t enable_wifi);

#endif //DB_ESP32_MAIN_H
