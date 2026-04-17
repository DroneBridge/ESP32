/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2026 Wolfgang Christl
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

#ifndef DB_ESP32_DB_LED_INDICATOR_H
#define DB_ESP32_DB_LED_INDICATOR_H

// Status LED API (used by serial/radio processing paths and timer module).
void db_status_led_init();
void db_status_led_mark_serial_mavlink_rx();
void db_status_led_mark_radio_rx();
void db_status_led_process();

#endif // DB_ESP32_DB_LED_INDICATOR_H
