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

#include "db_parameters.h"

#include <esp_log.h>
#include <esp_wifi_types_generic.h>
#include <string.h>
#include <lwip/sockets.h>

#define TAG "DB_PARAM"

/**
 * Steps to add new parameters.
 * 1. Add new parameter as db_parameter_t below
 * 2. Add new parameter to db_params array
 * 3. Increase DB_PARAM_TOTAL_NUM
 * 4. Increase DB_MAV_PARAM_CNT if parameter is available via MAVLink (non-string parameter)
 * 5. Make parameter available globally by adding extern definition into db_parameters.h
 * 6. Create an optional macro in db_parameters.h to make the parameter value easy accessible
 */

/**
 * Introduced and used for settings change only since changing DB_WIFI_MODE directly while system is live will
 * eventually lead to crashes. Stores the new Wi-Fi mode and will be written to the settings. This means it will
 * become active after reboot.
 */
uint8_t DB_RADIO_MODE_DESIGNATED = DB_WIFI_MODE_AP; // initially assign the same value as DB_RADIO_MODE

/* ---------- String based parameters - not available via MAVLink ---------- */

/**
 * Wi-Fi AP SSID name OR Wi-Fi AP SSID name to connect to in Wi-Fi client mode
 */
db_parameter_t db_param_ssid = {
        .db_name = "ssid",
        .type = STRING,
        .mav_t = {
                .param_name = "SYS_SSID",
                .param_index = -1,
                .param_type = MAV_PARAM_TYPE_ENUM_END, // no string support so far
        },
        .value = {
                .db_param_str = {
                        .value = "DroneBridge for ESP32",
                        .default_value = "DroneBridge for ESP32",
                        .min_len = 1,
                        .max_len = MAX_SSID_LEN,
                }
        }
};

/**
 * Password for ESP-NOW encryption.
 * Password for Wi-Fi connections.
 */
db_parameter_t db_param_pass = {
        .db_name = "wifi_pass",
        .type = STRING,
        .mav_t = {
                .param_name = "SYS_PASS",
                .param_index = -1,
                .param_type = MAV_PARAM_TYPE_ENUM_END, // no string support so far
        },
        .value = {
                .db_param_str = {
                        .value = "dronebridge",
                        .default_value = "dronebridge",
                        .min_len = 7,
                        .max_len = 64
                }
        }
};

/**
 * IPv4 of the Wi-Fi access point when in Wi-Fi AP mode
 */
db_parameter_t db_param_wifi_ap_ip = {
        .db_name = "ap_ip",
        .type = STRING,
        .mav_t = {
                .param_name = "WIFI_AP_IP",
                .param_index = -1,
                .param_type = MAV_PARAM_TYPE_ENUM_END, // no string support so far
        },
        .value = {
                .db_param_str = {
                        .value = "192.168.2.1",
                        .default_value = "192.168.2.1",
                        .min_len = 8,
                        .max_len = IP4ADDR_STRLEN_MAX
                }
        }
};

/**
 * User can specify static IP when in Wi-Fi client mode. If this is empty use auto IP.
 */
db_parameter_t db_param_wifi_sta_ip = {
        .db_name = "ip_sta",
        .type = STRING,
        .mav_t = {
                .param_name = "WIFI_STA_IP",
                .param_index = -1,
                .param_type = MAV_PARAM_TYPE_ENUM_END, // no string support so far
        },
        .value = {
                .db_param_str = {
                        .value = "",
                        .default_value = "",
                        .min_len = 0,   // can be empty
                        .max_len = IP4ADDR_STRLEN_MAX
                }
        }
};

/**
 * If db_param_wifi_sta_ip is set then this must be set to the GW IP
 */
db_parameter_t db_param_wifi_sta_gw = {
        .db_name = "ip_sta_gw",
        .type = STRING,
        .mav_t = {
                .param_name = "WIFI_STA_GW",
                .param_index = -1,
                .param_type = MAV_PARAM_TYPE_ENUM_END, // no string support so far
        },
        .value = {
                .db_param_str = {
                        .value = "",
                        .default_value = "",
                        .min_len = 0,
                        .max_len = IP4ADDR_STRLEN_MAX
                }
        }
};

/**
 * If db_param_wifi_sta_ip is set: Netmask when settings static IP in Wi-Fi client mode
 */
db_parameter_t db_param_wifi_sta_netmask = {
        .db_name = "ip_sta_netmsk",
        .type = STRING,
        .mav_t = {
                .param_name = "WIFI_STA_NETM",
                .param_index = -1,
                .param_type = MAV_PARAM_TYPE_ENUM_END, // no string support so far
        },
        .value = {
                .db_param_str = {
                        .value = "",
                        .default_value = "",
                        .min_len = 0,
                        .max_len = IP4ADDR_STRLEN_MAX
                }
        }
};

/**
 * Users can add custom UDP client targets. This is the IP of the first target added. Only the first one is saved to NVM.
 */
db_parameter_t db_param_udp_client_ip = {
        .db_name = "udp_client_ip",
        .type = STRING,
        .mav_t = {
                .param_name = "WIFI_UDP_IP",
                .param_index = -1,
                .param_type = MAV_PARAM_TYPE_ENUM_END, // no string support so far
        },
        .value = {
                .db_param_str = {
                        .value = "\0",
                        .default_value = "\0",
                        .min_len = 0,
                        .max_len = IP4ADDR_STRLEN_MAX
                }
        }
};

/* ---------- From here with increasing param_index all parameters that are also available via MAVLink ---------- */

/**
 * For reporting build version via MAVLink. Does not change.
 */
db_parameter_t db_param_sw_version = {
        .db_name = "sw_version",
        .type = UINT8,
        .mav_t = {
                .param_name = "SYS_SW_VERSION",
                .param_index = 0,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = DB_BUILD_VERSION,
                        .default_value = DB_BUILD_VERSION,
                        .min = DB_BUILD_VERSION,
                        .max = DB_BUILD_VERSION,
                }
        }
};

/**
 * never change this value while the ESP32 is running, will likely lead to a crash.
 * Assign it during startup when received from storage. Can therefore only be changed via reboot and DB_WIFI_MODE_DESIGNATED
 */
db_parameter_t db_param_radio_mode = {
        .db_name = "esp32_mode",
        .type = UINT8,
        .mav_t = {
                .param_name = "SYS_ESP32_MODE",
                .param_index = 1,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = DB_WIFI_MODE_AP,
                        .default_value = DB_WIFI_MODE_AP,
                        .min = DB_WIFI_MODE_AP,
                        .max = DB_WIFI_MODE_END,
                }
        }
};

/**
 * Radio channel in WiFi AP and ESP-NOW mode
 */
db_parameter_t db_param_channel = {
        .db_name = "wifi_chan",
        .type = UINT8,
        .mav_t = {
                .param_name = "WIFI_AP_CHANNEL",
                .param_index = 2,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = 6,
                        .default_value = 6,
                        .min = 1,
                        .max = 13,
                }
        }
};

/**
 *  Allow the usage of 802.11bgn mode
 *  disabled: only 802.11b support for client mode - set to true: 802.11bgn mode support
 */
db_parameter_t db_param_wifi_en_gn = {
        .db_name = "wifi_en_gn",
        .type = UINT8,
        .mav_t = {
                .param_name = "WIFI_EN_GN",
                .param_index = 3,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = false,
                        .default_value = false,
                        .min = false,
                        .max = true,
                }
        }
};

/**
 * Set to 1 to use external antenna. Set to 0 to enable onboard antenna - board must have antenna switch
 */
db_parameter_t db_param_wifi_ant_ext = {
        .db_name = "ant_use_ext",
        .type = UINT8,
        .mav_t = {
                .param_name = "RADIO_EN_EXT_ANT",
                .param_index = 4,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = false,
                        .default_value = false,
                        .min = false,
                        .max = true,
                }
        }
};

/**
 * UART baud rate
 */
db_parameter_t db_param_baud = {
        .db_name = "baud",
        .type = INT32,
        .mav_t = {
                .param_name = "RADIO_EN_EXT_ANT",
                .param_index = 5,
                .param_type = MAV_PARAM_TYPE_INT32,
        },
        .value = {
                .db_param_i32 = {
                        .value = DB_DEFAULT_UART_BAUD_RATE,
                        .default_value = DB_DEFAULT_UART_BAUD_RATE,
                        .min = 1200,
                        .max = 5000000,
                }
        }
};

/**
 * TX GPIO number of the UART
 */
db_parameter_t db_param_gpio_tx = {
        .db_name = "gpio_tx",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_TX_PIN",
                .param_index = 6,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = DB_DEFAULT_UART_TX_PIN,
                        .default_value = DB_DEFAULT_UART_TX_PIN,
                        .min = 0,
                        .max = SOC_GPIO_IN_RANGE_MAX,
                }
        }
};

/**
 * RX GPIO number of the UART
 */
db_parameter_t db_param_gpio_rx = {
        .db_name = "gpio_rx",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_RX_PIN",
                .param_index = 7,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = DB_DEFAULT_UART_RX_PIN,
                        .default_value = DB_DEFAULT_UART_RX_PIN,
                        .min = 0,
                        .max = SOC_GPIO_IN_RANGE_MAX,
                }
        }
};

/**
 * RTS GPIO number of the UART. Set to same value as CTS GPIO to disable flow control.
 */
db_parameter_t db_param_gpio_rts = {
        .db_name = "gpio_rts",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_RTS_PIN",
                .param_index = 8,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = DB_DEFAULT_UART_RTS_PIN,
                        .default_value = DB_DEFAULT_UART_RTS_PIN,
                        .min = 0,
                        .max = SOC_GPIO_IN_RANGE_MAX,
                }
        }
};

/**
 * CTS GPIO number of the UART. Set to same value as RTS GPIO to disable flow control.
 */
db_parameter_t db_param_gpio_cts = {
        .db_name = "gpio_cts",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_CTS_PIN",
                .param_index = 9,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = DB_DEFAULT_UART_CTS_PIN,
                        .default_value = DB_DEFAULT_UART_CTS_PIN,
                        .min = 0,
                        .max = SOC_GPIO_IN_RANGE_MAX,
                }
        }
};

/**
 * I really don't know. Maybe some sort of timeout. Just leave it at the defaults.
 */
db_parameter_t db_param_gpio_rts_thresh = {
        .db_name = "rts_thresh",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_RTS_THRES",
                .param_index = 10,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = 64,
                        .default_value = 64,
                        .min = 0,
                        .max = 128,
                }
        }
};

/**
 * Sets the parser for the serial port
 */
db_parameter_t db_param_proto = {
        .db_name = "proto",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_TEL_PROTO",
                .param_index = 11,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = DB_SERIAL_PROTOCOL_MAVLINK,
                        .default_value = DB_SERIAL_PROTOCOL_MAVLINK,
                        .min = DB_SERIAL_PROTOCOL_MSPLTM,
                        .max = DB_SERIAL_PROTOCOL_TRANSPARENT,
                }
        }
};

/**
 * Maximum packet size via ESP-NOW or WiFi in transparent or mavlink mode. Caped to 250 bytes-HEADER in ESP-NOW mode.
 */
db_parameter_t db_param_serial_pack_size = {
        .db_name = "trans_pack_size",
        .type = UINT16,
        .mav_t = {
                .param_name = "SERIAL_PACK_SIZE",
                .param_index = 12,
                .param_type = MAV_PARAM_TYPE_UINT16,
        },
        .value = {
                .db_param_u16 = {
                        .value = 128,
                        .default_value = 128,
                        .min = 16,
                        .max = 1023,
                }
        }
};

/**
 * Serial read timeout [ms] for transparent and MAVLink mode, after that the packet will be sent over the air even when
 * the max. packet size was not reached.
 */
db_parameter_t db_param_serial_read_timeout = {
        .db_name = "serial_timeout",
        .type = UINT16,
        .mav_t = {
                .param_name = "SERIAL_T_OUT_MS",
                .param_index = 13,
                .param_type = MAV_PARAM_TYPE_UINT16,
        },
        .value = {
                .db_param_u16 = {
                        .value = 50,
                        .default_value = 50,
                        .min = 1,
                        .max = UINT16_MAX,
                }
        }
};

/**
 * Number of LTM frames per packet
 */
db_parameter_t db_param_ltm_per_packet = {
        .db_name = "ltm_per_packet",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_LTM_PACK",
                .param_index = 14,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = 2,
                        .default_value = 2,
                        .min = 1,
                        .max = MAX_LTM_FRAMES_IN_BUFFER,
                }
        }
};

/**
 * Detects armed state from MAVLink & LTM stream and disables radio when autopilot reports armed state.
 */
db_parameter_t db_param_dis_radio_armed = {
        .db_name = "radio_dis_onarm",
        .type = UINT8,
        .mav_t = {
                .param_name = "RADIO_DIS_ON_ARM",
                .param_index = 15,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = false,
                        .default_value = false,
                        .min = false,
                        .max = true,
                }
        }
};

/**
 * Port for db_param_udp_client_ip
 */
db_parameter_t db_param_udp_client_port = {
        .db_name = "udp_client_port",
        .type = UINT16,
        .mav_t = {
                .param_name = "WIFI_UDP_CPORT",
                .param_index = 16,
                .param_type = MAV_PARAM_TYPE_UINT16,
        },
        .value = {
                .db_param_u16 = {
                        .value = 0,
                        .default_value = 0,
                        .min = 0,
                        .max = UINT16_MAX,
                }
        }
};

/**
 *  Enable the use of the external antenna of the official DB for ESP32 C6 board
 */
db_parameter_t db_param_en_ext_ant = {
        .db_name = "ant_use_ext",
        .type = UINT8,
        .mav_t = {
                .param_name = "RADIO_EN_EXT_ANT",
                .param_index = 17,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = false,
                        .default_value = false,
                        .min = false,
                        .max = true,
                }
        }
};

/**
 *  Format/Unit of the reported RSSI as part of MAVLink RADIO STATUS message.
 *  If set to true (1) the RSSI will be reported as dBm (QGC)
 *  If set to false (0) the RSSI will be calculated as a value from 0 to 100 (MissionPlanner)
 */
db_parameter_t db_param_rssi_dbm = {
        .db_name = "rep_rssi_dbm",
        .type = UINT8,
        .mav_t = {
                .param_name = "RADIO_RSSI_DBM",
                .param_index = 18,
                .param_type = MAV_PARAM_TYPE_UINT8,
        },
        .value = {
                .db_param_u8 = {
                        .value = true,
                        .default_value = true,
                        .min = false,
                        .max = true,
                }
        }
};

/**
 * Array containing all references to the DB parameters
 */
db_parameter_t *db_params[] = {
        &db_param_ssid,
        &db_param_pass,
        &db_param_wifi_ap_ip,
        &db_param_wifi_sta_ip,
        &db_param_wifi_sta_gw,
        &db_param_wifi_sta_netmask,
        &db_param_udp_client_ip,
        &db_param_radio_mode,
        &db_param_sw_version,
        &db_param_channel,
        &db_param_wifi_en_gn,
        &db_param_wifi_ant_ext,
        &db_param_baud,
        &db_param_gpio_tx,
        &db_param_gpio_rx,
        &db_param_gpio_rts,
        &db_param_gpio_cts,
        &db_param_gpio_rts_thresh,
        &db_param_proto,
        &db_param_serial_pack_size,
        &db_param_serial_read_timeout,
        &db_param_ltm_per_packet,
        &db_param_dis_radio_armed,
        &db_param_udp_client_port,
        &db_param_en_ext_ant,
        &db_param_rssi_dbm
};

//db_parameter_t db_param_init_str_param(uint8_t *db_name, uint8_t *mav_param_name, uint8_t *default_value,
//                                       uint8_t max_val_len, uint8_t min_val_len) {
//    db_parameter_t db_str_param = {
//            .db_name = "",
//            .type = STRING,
//            .mav_t = {
//                    .param_name = "",
//                    .param_index = -1,
//                    .param_type = MAV_PARAM_TYPE_ENUM_END, // no string support so far
//            },
//            .value = {
//                    .db_param_str = {
//                            .min_len = min_val_len,
//                            .max_len = max_val_len,
//                    }
//            }
//    };
//    strncpy((char *) db_str_param.db_name, (char *) db_name, DB_PARAM_NAME_MAXLEN);
//    strncpy((char *) db_str_param.mav_t.param_name, (char *) mav_param_name, 16);
//    db_str_param.value.db_param_str.default_value = (uint8_t *) strdup((char *) default_value);
//    db_str_param.value.db_param_str.value = malloc(max_val_len);
//    if (db_str_param.value.db_param_str.value == NULL) {
//        ESP_LOGE(TAG, "Error allocating space for string parameter %s", db_name);
//    } else {
//        // all good
//    }
//    return db_str_param;
//}
//
//void db_param_init_parameters() {
//    // Wi-Fi AP SSID name OR Wi-Fi AP SSID name to connect to in Wi-Fi client mode
//    db_param_init_str_param("ssid", "SYS_SSID", "DroneBridge for ESP32", MAX_SSID_LEN, 1);
//
//}

/**
 * Sets the value of the supplied the parameter to its default value
 * @param db_parameter The parameter to reset to default
 */
void db_param_set_to_default(db_parameter_t *db_parameter) {
    switch (db_parameter->type) {
        case STRING:
            strncpy((char *) db_parameter->value.db_param_str.value,
                    (char *) db_parameter->value.db_param_str.default_value,
                    DB_PARAM_VALUE_MAXLEN);
            break;
        case UINT8:
            db_parameter->value.db_param_u8.value = db_parameter->value.db_param_u8.default_value;
            break;
        case UINT16:
            db_parameter->value.db_param_u16.value = db_parameter->value.db_param_u16.default_value;
            break;
        case INT32:
            db_parameter->value.db_param_i32.value = db_parameter->value.db_param_i32.default_value;
            break;
        default:
            ESP_LOGE(TAG, "db_param_set_to_default() -> db_parameter.type unknown!");
            break;
    }
}

/**
 * Helper function to reset all known parameters to their defaults. Parameter must be part of db_params array
 */
void db_param_reset_all() {
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        db_param_set_to_default(db_params[i]);
    }
}

/**
 * Helper function to convert all parameters with their values to a string buffer for logging etc.
 * @param str_buffer Buffer to write the parameter string - must be long enough ~512 bytes
 */
int db_param_print_values_to_buffer(uint8_t *str_buffer) {
    int str_len = 0; // overall length of the string in the str_buffer
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        uint8_t param_str_buf[128]; // buffer for the string of a single value
        switch (db_params[i]->type) {
            case STRING:
                str_len += sprintf((char *) param_str_buf, "\t%s: %s\n", (char *) db_params[i]->db_name,
                                   (char *) db_params[i]->value.db_param_str.value);
                break;
            case UINT8:
                str_len += sprintf((char *) param_str_buf, "\t%s: %i\n", (char *) db_params[i]->db_name,
                                   db_params[i]->value.db_param_u8.value);
                break;
            case UINT16:
                str_len += sprintf((char *) param_str_buf, "\t%s: %i\n", (char *) db_params[i]->db_name,
                                   db_params[i]->value.db_param_u16.value);
                break;
            case INT32:
                str_len += sprintf((char *) param_str_buf, "\t%s: %li\n", (char *) db_params[i]->db_name,
                                   db_params[i]->value.db_param_i32.value);
                break;
            default:
                ESP_LOGE(TAG, "db_param_print_values_to_buffer() -> db_parameter.type unknown!");
                break;
        }
        strcat((char *) str_buffer,
               (char *) param_str_buf); // add the string of the individual printed param to the big buffer
    }
    return str_len;
}

/**
 * Helper function to read a string from the NVS based on a key. Handles errors accordingly and print result to console
 * @param my_handle nvs_handle to use
 * @param key NVS key as string with max length NVS_KEY_NAME_MAX_SIZE-1
 * @param dst Destination for the read value
 */
void db_read_str_nvs(nvs_handle_t my_handle, char *key, char *dst) {
    if (strlen(key) + 1 > NVS_KEY_NAME_MAX_SIZE)
        ESP_LOGW(TAG, "key %s is longer than %i bytes", key, NVS_KEY_NAME_MAX_SIZE);
    size_t required_size = 0;
    esp_err_t err = nvs_get_str(my_handle, key, NULL, &required_size);
    if (err == ESP_OK) {
        char *read_nvs_val = malloc(required_size);
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_str(my_handle, key, read_nvs_val, &required_size));
        memcpy(dst, read_nvs_val, required_size);
        free(read_nvs_val);
    } else {
        ESP_LOGW(TAG, "Could not read key %s from NVS", key);
    }
}

/**
 * Updates all parameters with their values from the NVM.
 * Parameters must be part of db_params array
 * @param nvs_handle Opened Namespace of the NVM partition. This is the handle.
 */
void db_param_read_all_params_nvs(const nvs_handle_t *nvs_handle) {
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        switch (db_params[i]->type) {
            case STRING:
                db_read_str_nvs(*nvs_handle, (char *) db_params[i]->db_name,
                                (char *) db_params[i]->value.db_param_str.value);
                break;
            case UINT8:
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                        nvs_get_u8(*nvs_handle, (char *) db_params[i]->db_name,
                                   &db_params[i]->value.db_param_u8.value));
                break;
            case UINT16:
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                        nvs_get_u16(*nvs_handle, (char *) db_params[i]->db_name,
                                    &db_params[i]->value.db_param_u16.value));
                break;
            case INT32:
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                        nvs_get_i32(*nvs_handle, (char *) db_params[i]->db_name,
                                    &db_params[i]->value.db_param_i32.value));
                break;
            default:
                ESP_LOGE(TAG, "db_param_read_all_params_to_nvs() -> db_parameter.type unknown!");
                break;
        }
    }
}

/**
 * Writes all parameters with their values to the NVM. Makes an exception for the mode.
 * Here DB_RADIO_MODE_DESIGNATED is saved instead.
 * Parameters must be part of db_params array
 * @param nvs_handle Opened Namespace of the NVM partition. This is the handle.
 */
void db_param_write_all_params_nvs(const nvs_handle_t *nvs_handle) {
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        switch (db_params[i]->type) {
            case STRING:
                ESP_ERROR_CHECK(nvs_set_str(*nvs_handle, (char *) db_params[i]->db_name,
                                            (char *) db_params[i]->value.db_param_str.value));
                break;
            case UINT8:
                if (strcmp((char *) db_params[i]->db_name, (char *) db_param_radio_mode.db_name) == 0) {
                    // This is different. User writes the desired mode into DB_RADIO_MODE_DESIGNATED and does not overwrite the value
                    ESP_ERROR_CHECK(nvs_set_u8(*nvs_handle, (char *) db_params[i]->db_name, DB_RADIO_MODE_DESIGNATED));
                } else {
                    ESP_ERROR_CHECK(
                            nvs_set_u8(*nvs_handle, (char *) db_params[i]->db_name,
                                       db_params[i]->value.db_param_u8.value));
                }
                break;
            case UINT16:
                ESP_ERROR_CHECK(nvs_set_u16(*nvs_handle, (char *) db_params[i]->db_name,
                                            db_params[i]->value.db_param_u16.value));
                break;
            case INT32:
                ESP_ERROR_CHECK(nvs_set_i32(*nvs_handle, (char *) db_params[i]->db_name,
                                            db_params[i]->value.db_param_i32.value));
                break;
            default:
                ESP_LOGE(TAG, "db_param_write_all_params_to_nvs() -> db_parameter.type unknown!");
                break;
        }
    }
}

/**
 * Parses the provided json for all known parameters. Applies all recognized parameter values to local storage.
 * Checks if parameter values are within the valid range. Otherwise, reject by not applying the new value.
 * Does not save them to NVM!
 * @param root_obj JSON that contains a single layer with the parameters to change.
 */
void db_param_read_all_params_json(const cJSON *root_obj) {
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        cJSON *jobject = cJSON_GetObjectItem(root_obj, (char *) db_params[i]->db_name);
        switch (db_params[i]->type) {
            case STRING:
                if (jobject) {
                    db_param_is_valid_assign_str(jobject->valuestring, db_params[i]);
                } else {
                    // do nothing - param was not found in the json
                }
                break;
            case UINT8:
                if (jobject) {
                    db_param_is_valid_assign_u8(jobject->valueint, db_params[i]);
                } else {
                    // do nothing - param was not found in the json
                }
                break;
            case UINT16:
                if (jobject) {
                    db_param_is_valid_assign_u16(jobject->valueint, db_params[i]);
                } else {
                    // do nothing - param was not found in the json
                }
                break;
            case INT32:
                if (jobject) {
                    db_param_is_valid_assign_i32(jobject->valueint, db_params[i]);
                } else {
                    // do nothing - param was not found in the json
                }
                break;
            default:
                ESP_LOGE(TAG, "db_param_write_all_params_to_nvs() -> db_parameter.type unknown!");
                break;
        }
    }
}

/**
 * Writes all known parameters to a supplied cJSON.
 * @param root_obj A cJSON object that will be filled with all known parameters
 */
void db_param_write_all_params_json(cJSON *root_obj) {
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        switch (db_params[i]->type) {
            case STRING:
                cJSON_AddStringToObject(root_obj, (char *) db_params[i]->db_name,
                                        (char *) db_params[i]->value.db_param_str.value);
                break;
            case UINT8:
                cJSON_AddNumberToObject(root_obj, (char *) db_params[i]->db_name,
                                        db_params[i]->value.db_param_u8.value);
                break;
            case UINT16:
                cJSON_AddNumberToObject(root_obj, (char *) db_params[i]->db_name,
                                        db_params[i]->value.db_param_u16.value);
                break;
            case INT32:
                cJSON_AddNumberToObject(root_obj, (char *) db_params[i]->db_name,
                                        db_params[i]->value.db_param_i32.value);
                break;
            default:
                ESP_LOGE(TAG, "db_param_write_all_params_json() -> db_parameter.type unknown!");
                break;
        }
    }
}

/**
 * Checks if string is valid for assignment to the target_param.
 * If valid assigns the new value to the parameter
 * @param new_string_value The new string to be checked and assigned
 * @param target_param The internal db parameter to be assigned with the new value
 * @return true when valid and assigned - else false
 */
bool db_param_is_valid_assign_str(const char *new_string_value, db_parameter_t *target_param) {
    // ToDo: Add IPv4 check for strings via custom validation function in db_parameter_t
    if (strlen(new_string_value) <= target_param->value.db_param_str.max_len &&
        strlen(new_string_value) >= target_param->value.db_param_str.min_len) {
        strncpy((char *) target_param->value.db_param_str.value, new_string_value, DB_PARAM_VALUE_MAXLEN);
        return true;
    }
    // new value is not valid
    ESP_LOGE(TAG, "db_param_is_valid_assign_str(): Invalid string length (%i-%i) for param %s",
             target_param->value.db_param_str.min_len, target_param->value.db_param_str.max_len,
             (char *) target_param->db_name);
    return false;
}

/**
 * Checks if u8 is valid for assignment to the target_param.
 * If valid assigns the new value to the parameter
 * @param new_u8_value The new u8 to be checked and assigned
 * @param target_param The internal db parameter to be assigned with the new value
 * @return true when valid and assigned - else false
 */
bool db_param_is_valid_assign_u8(const uint8_t new_u8_value, db_parameter_t *target_param) {
    if (new_u8_value <= target_param->value.db_param_u8.max && new_u8_value >= target_param->value.db_param_u8.min) {
        if (strcmp((char *) target_param->db_name, (char *) db_param_radio_mode.db_name) == 0) {
            // Special case check: Do not directly change DB_WIFI_MODE since it is not safe and constantly
            // processed by other tasks. Save settings and reboot will assign DB_RADIO_MODE_DESIGNATED to DB_WIFI_MODE
            DB_RADIO_MODE_DESIGNATED = new_u8_value;
        } else {
            target_param->value.db_param_u8.value = new_u8_value; // accept value and assign
        }
        return true;
    }
    // new value is not valid
    ESP_LOGE(
            TAG, "db_param_is_valid_assign_u8(): Value %i is out of valid range (%i-%i) for param %s",
            new_u8_value, target_param->value.db_param_u8.max, target_param->value.db_param_u8.min,
            (char *) target_param->db_name);
    return false;
}

/**
 * Checks if u16 is valid for assignment to the target_param.
 * If valid assigns the new value to the parameter
 * @param new_u16_value The new u16 to be checked and assigned
 * @param target_param The internal db parameter to be assigned with the new value
 * @return true when valid and assigned - else false
 */
bool db_param_is_valid_assign_u16(const uint16_t new_u16_value, db_parameter_t *target_param) {
    if (new_u16_value <= target_param->value.db_param_u16.max &&
        new_u16_value >= target_param->value.db_param_u16.min) {
        target_param->value.db_param_u16.value = new_u16_value; // accept value and assign
        return true;
    }
    // new value is not valid
    ESP_LOGE(
            TAG, "db_param_is_valid_assign_u16(): Value %i is out of valid range (%i-%i) for param %s",
            new_u16_value, target_param->value.db_param_u16.max, target_param->value.db_param_u16.min,
            (char *) target_param->db_name);
    return false;
}

/**
 * Checks if i32 is valid for assignment to the target_param.
 * If valid assigns the new value to the parameter
 * @param new_i32_value The new i32 to be checked and assigned
 * @param target_param The internal db parameter to be assigned with the new value
 * @return true when valid and assigned - else false
 */
bool db_param_is_valid_assign_i32(const int32_t new_i32_value, db_parameter_t *target_param) {
    if (new_i32_value <= target_param->value.db_param_i32.max &&
        new_i32_value >= target_param->value.db_param_i32.min) {
        target_param->value.db_param_i32.value = new_i32_value; // accept value and assign
        return true;
    }
    // new value is not valid
    ESP_LOGE(
            TAG, "db_param_is_valid_assign_i32(): Value %li is out of valid range (%li-%li) for param %s",
            new_i32_value, target_param->value.db_param_i32.max, target_param->value.db_param_i32.min,
            (char *) target_param->db_name);
    return false;
}

/**
 * Checks if the supplied IPv4 string is representing a valid IPv4 address.
 * @param ipaddress IPv4 string
 * @return true if valid IPv4 string was supplied
 */
bool is_valid_ip4(const char *ipaddress) {
    struct sockaddr_in sa;
    const int result = inet_pton(AF_INET, ipaddress, &(sa.sin_addr));
    return result != 0;
}
