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
#include <string.h>

#define TAG "DB_PARAM"

/**
 * Steps to add new parameters.
 * 1. Add new parameter as db_parameter_t below
 * 2. Add new parameter to db_params array below
 * 3. Increase DB_MAV_PARAM_CNT if parameter is available via MAVLink (non-string parameter)
 * 4. Make parameter available globally by adding extern definition into db_parameters.h
 * 5. Create an optional macro in db_parameters.h to make the parameter value easy accessible
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
            .value = '\0',
            .default_value = '\0',
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
        .db_param_u8 = {
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
    &db_param_udp_client_port
};

/**
 * Sets the value of the supplied the parameter to its default value
 * @param db_parameter The parameter to reset to default
 */
void db_param_set_to_default(db_parameter_t *db_parameter) {
    switch (db_parameter->type) {
        case STRING:
            strncpy(db_parameter->value.db_param_str.value, db_parameter->value.db_param_str.default_value, DB_PARAM_VALUE_MAXLEN);
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
    int str_len = 0;    // overall length of the string in the str_buffer
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        uint8_t param_str_buf[128]; // buffer for the string of a single value
        switch (db_params[i]->type) {
            case STRING:
                str_len += sprintf((char *) param_str_buf, "\t%s: %s\n", (char *) db_params[i]->db_name, (char *) db_params[i]->value.db_param_str.value);
            break;
            case UINT8:
                str_len += sprintf((char *) param_str_buf, "\t%s: %i\n", (char *) db_params[i]->db_name, db_params[i]->value.db_param_u8.value);
            break;
            case UINT16:
                str_len += sprintf((char *) param_str_buf, "\t%s: %i\n", (char *) db_params[i]->db_name, db_params[i]->value.db_param_u16.value);
            break;
            case INT32:
                str_len += sprintf((char *) param_str_buf, "\t%s: %i\n", (char *) db_params[i]->db_name, db_params[i]->value.db_param_i32.value);
            break;
            default:
                ESP_LOGE(TAG, "db_param_print_values_to_buffer() -> db_parameter.type unknown!");
            break;
        }
        strcat((char *) str_buffer, param_str_buf); // add the string of the individual printed param to the big buffer
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
        ESP_LOGI(TAG, "\t%s: %s", key, dst);
    } else {
        ESP_LOGW(TAG, "Could not read key %s from NVS", key);
    }
}

/**
 * Updates all parameters with their values from the NVM.
 * Parameters must be part of db_params array
 * @param nvs_handle Opened Namespace of the NVM partition. This is the handle.
 */
void db_param_read_all_params_to_nvs(const nvs_handle_t *nvs_handle) {
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        switch (db_params[i]->type) {
            case STRING:
                db_read_str_nvs(*nvs_handle, db_params[i]->db_name, db_params[i]->value.db_param_str.value);
            break;
            case UINT8:
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(*nvs_handle, db_params[i]->db_name, &db_params[i]->value.db_param_u8.value));
            break;
            case UINT16:
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u16(*nvs_handle, db_params[i]->db_name, &db_params[i]->value.db_param_u16.value));
            break;
            case INT32:
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_i32(*nvs_handle, db_params[i]->db_name, &db_params[i]->value.db_param_i32.value));
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
void db_param_write_all_params_to_nvs(const nvs_handle_t *nvs_handle) {
    for (int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
        switch (db_params[i]->type) {
            case STRING:
                ESP_ERROR_CHECK(nvs_set_str(*nvs_handle, db_params[i]->db_name, db_params[i]->value.db_param_str.value));
            break;
            case UINT8:
                if (strcmp(db_params[i]->db_name, db_param_radio_mode.db_name) == 0) {
                    // This is different. User writes the desired mode into DB_RADIO_MODE_DESIGNATED and does not overwrite the value
                    ESP_ERROR_CHECK(nvs_set_u8(*nvs_handle, db_params[i]->db_name, DB_RADIO_MODE_DESIGNATED));
                } else {
                    ESP_ERROR_CHECK(nvs_set_u8(*nvs_handle, db_params[i]->db_name, db_params[i]->value.db_param_u8.value));
                }
            break;
            case UINT16:
                ESP_ERROR_CHECK(nvs_set_u16(*nvs_handle, db_params[i]->db_name, db_params[i]->value.db_param_u16.value));
            break;
            case INT32:
                ESP_ERROR_CHECK(nvs_set_i32(*nvs_handle, db_params[i]->db_name, db_params[i]->value.db_param_i32.value));
            break;
            default:
                ESP_LOGE(TAG, "db_param_write_all_params_to_nvs() -> db_parameter.type unknown!");
            break;
        }
    }
}
