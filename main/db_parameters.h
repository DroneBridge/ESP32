/*
*   This file is part of DroneBridge:https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2025 Wolfgang Christl
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

#ifndef DB_PARAMETERS_H
#define DB_PARAMETERS_H

#include <string.h>
#include <cJSON.h>
#include <nvs.h>
#include <stdint.h>
#include <driver/gpio.h>
#include "common/common.h"

#define DB_BUILD_VERSION 16
#define DB_MAJOR_VERSION 2
#define DB_MINOR_VERSION 1
#define DB_PATCH_VERSION 0
#define DB_MATURITY_VERSION "RC2"

#define DB_PARAM_TOTAL_NUM          25  // total number of db parameters
#define DB_PARAM_MAV_CNT            17  // Number of MAVLink parameters returned by ESP32 in the PARAM message. Needed by GCS.

#define DB_PARAM_NAME_MAXLEN        16      // max len of a parameter/key stored in the ESP32 NVM
#define DB_PARAM_MAX_MAV_PARAM_NAME_LEN 16  // max len of the field used to store the mav param name (max len 16 by def.)
#define DB_PARAM_VALUE_MAXLEN       64      // max len of a value stored for a key in the ESP32 NVM (string len)
#define MAX_LTM_FRAMES_IN_BUFFER    5


#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_5
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_4
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_6
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_7
#define DB_DEFAULT_UART_BAUD_RATE 115200
#elif CONFIG_DB_OFFICIAL_BOARD_1_X_C6
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_21
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_2
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_22
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_23
#define DB_DEFAULT_UART_BAUD_RATE 115200
#elif CONFIG_DB_GENERIC_BOARD
// initially set pins to 0 to allow the start of the system on all boards. User has to set the correct pins
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_BAUD_RATE 57600
#else
// someone fucked up the config - fallback to generic config
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_BAUD_RATE 57600
#endif

/* ---------- Optional Macros for quick access to param values ---------- */
#define DB_PARAM_WIFI_SSID (char *) db_param_ssid.value.db_param_str.value
#define DB_PARAM_PASS (char *) db_param_pass.value.db_param_str.value
#define DB_PARAM_CHANNEL db_param_channel.value.db_param_u8.value
#define DB_PARAM_RADIO_MODE db_param_radio_mode.value.db_param_u8.value
#define DB_PARAM_STA_IP db_param_wifi_sta_ip.value.db_param_str.value
#define DB_PARAM_STA_GW db_param_wifi_sta_gw.value.db_param_str.value
#define DB_PARAM_STA_IP_NETMASK db_param_wifi_sta_netmask.value.db_param_str.value
#define DB_PARAM_AP_IP (char *) db_param_wifi_ap_ip.value.db_param_str.value
#define DB_PARAM_WIFI_EN_GN db_param_wifi_en_gn.value.db_param_u8.value
#define DB_PARAM_DIS_RADIO_ON_ARM db_param_dis_radio_armed.value.db_param_u8.value
#define DB_PARAM_SERIAL_BAUD db_param_baud.value.db_param_i32.value
#define DB_PARAM_SERIAL_PROTO db_param_proto.value.db_param_u8.value
#define DB_PARAM_SERIAL_PACK_SIZE db_param_serial_pack_size.value.db_param_u16.value
#define DB_PARAM_GPIO_TX db_param_gpio_tx.value.db_param_u8.value
#define DB_PARAM_GPIO_RX db_param_gpio_rx.value.db_param_u8.value
#define DB_PARAM_GPIO_RTS db_param_gpio_rts.value.db_param_u8.value
#define DB_PARAM_GPIO_CTS db_param_gpio_cts.value.db_param_u8.value
#define DB_PARAM_SERIAL_RTS_THRESH db_param_gpio_rts_thresh.value.db_param_u8.value
#define DB_PARAM_EN_EXT_ANT db_param_radio_ant_ext.value.db_param_u8.value

enum E_DB_WIFI_MODE {
  DB_WIFI_MODE_AP         = 1, // Wi-Fi access point mode with 802.11b mode enabled
  DB_WIFI_MODE_STA        = 2, // Wi-Fi client mode with 802.11b and LR mode enabled
  DB_WIFI_MODE_AP_LR      = 3, // ESP32 WiFi LR Mode 802.11b
  DB_WIFI_MODE_ESPNOW_AIR = 4, // ESP-NOW Mode for broadcasting device
  DB_WIFI_MODE_ESPNOW_GND = 5, // ESP-NOW Mode for GND station
  DB_WIFI_MODE_END        = 6, // End of enum
  DB_BLUETOOTH_MODE_SPP   = 7, // Bluetooth SPP mode
};

enum E_DB_SERIAL_PROTOCOL {
    DB_SERIAL_PROTOCOL_MSPLTM = 1,
    DB_SERIAL_PROTOCOL_MAVLINK = 4,
    DB_SERIAL_PROTOCOL_TRANSPARENT = 5
};

typedef struct db_parameter_str_s {
    uint8_t *value;
    uint8_t *default_value;
    uint8_t min_len;    // valid if >= min_len
    uint8_t max_len;    // valid if <= max_len
} db_param_str_t;

typedef struct db_parameter_u8_s {
    uint8_t value;
    uint8_t default_value;
    uint8_t min;
    uint8_t max;
} db_param_u8_t;

typedef struct db_parameter_u16_s {
    uint16_t value;
    uint16_t default_value;
    uint16_t min;
    uint16_t max;
} db_param_u16_t;

typedef struct db_parameter_i32_s {
    int32_t value;
    int32_t default_value;
    int32_t min;
    int32_t max;
} db_param_i32_t;

enum db_param_type_t {
  STRING,
  UINT8,
  UINT16,
  INT32
};

typedef struct db_parameter_s {
    uint8_t db_name[DB_PARAM_NAME_MAXLEN];  // equals the NVS key, JSON object name and web interface input ID
    enum db_param_type_t type;
    struct {    // no string support for mavlink
      uint8_t param_name[DB_PARAM_MAX_MAV_PARAM_NAME_LEN];
      uint16_t param_index;         // strings will be ignored for mavlink set to -1 for strings
      MAV_PARAM_TYPE param_type;    // set to MAV_PARAM_TYPE_ENUM_END for strings
    } mav_t;
    union {
      db_param_str_t db_param_str;
      db_param_u8_t db_param_u8;
      db_param_u16_t db_param_u16;
      db_param_i32_t db_param_i32;
    } value;
} db_parameter_t;

extern uint8_t DB_RADIO_MODE_DESIGNATED;
extern db_parameter_t *db_params[DB_PARAM_TOTAL_NUM];

extern db_parameter_t db_param_ssid;
extern db_parameter_t db_param_pass;
extern db_parameter_t db_param_wifi_ap_ip;
extern db_parameter_t db_param_wifi_sta_ip;
extern db_parameter_t db_param_wifi_sta_gw;
extern db_parameter_t db_param_wifi_sta_netmask;
extern db_parameter_t db_param_udp_client_ip;
extern db_parameter_t db_param_wifi_hostname;
extern db_parameter_t db_param_radio_mode;
extern db_parameter_t db_param_channel;
extern db_parameter_t db_param_wifi_en_gn;
extern db_parameter_t db_param_radio_ant_ext;
extern db_parameter_t db_param_baud;
extern db_parameter_t db_param_gpio_tx;
extern db_parameter_t db_param_gpio_rx;
extern db_parameter_t db_param_gpio_rts;
extern db_parameter_t db_param_gpio_cts;
extern db_parameter_t db_param_gpio_rts_thresh;
extern db_parameter_t db_param_proto;
extern db_parameter_t db_param_serial_pack_size;
extern db_parameter_t db_param_serial_read_timeout;
extern db_parameter_t db_param_ltm_per_packet;
extern db_parameter_t db_param_dis_radio_armed;
extern db_parameter_t db_param_udp_client_port;
extern db_parameter_t db_param_rssi_dbm;

void db_param_init_parameters();
void db_param_set_to_default(db_parameter_t *db_parameter);
void db_param_reset_all();
int db_param_print_values_to_buffer(uint8_t *str_buffer);
void db_param_read_all_params_nvs(const nvs_handle_t *nvs_handle);
void db_param_write_all_params_nvs(const nvs_handle_t *nvs_handle);
void db_param_read_all_params_json(const cJSON *root_obj);
void db_param_write_all_params_json(cJSON *root_obj);
bool db_param_is_valid_assign_str(const char *new_string_value, db_parameter_t *target_param);
bool db_param_is_valid_assign_u8(uint8_t new_u8_value, db_parameter_t *target_param);
bool db_param_is_valid_assign_u16(uint16_t new_u16_value, db_parameter_t *target_param);
bool db_param_is_valid_assign_i32(int32_t new_i32_value, db_parameter_t *target_param);
bool is_valid_ip4(const char *ipaddress);

#endif //DB_PARAMETERS_H
