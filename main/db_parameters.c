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
 * 4. Increase DB_MAV_PARAM_CNT if parameter is available via MAVLink
 * (non-string parameter)
 * 5. Make parameter available globally by adding extern definition into
 * db_parameters.h
 * 6. Create an optional macro in db_parameters.h to make the parameter value
 * easy accessible
 */

/**
 * Introduced and used for settings change only since changing DB_WIFI_MODE
 * directly while system is live will eventually lead to crashes. Stores the
 * new Wi-Fi mode and will be written to the settings. This means it will
 * become active after reboot.
 */
uint8_t DB_RADIO_MODE_DESIGNATED =
  DB_WIFI_MODE_AP; // initially assign the same value as DB_RADIO_MODE

/* ---------- String based parameters - not available via MAVLink ---------- */

db_parameter_t db_param_ssid, db_param_pass, db_param_wifi_ap_ip,
  db_param_wifi_sta_ip, db_param_wifi_sta_gw, db_param_wifi_sta_netmask,
  db_param_udp_client_ip, db_param_wifi_hostname = { 0 };

/* ---------- From here with increasing param_index all parameters that are
 * also available via MAVLink ---------- */

/**
 * never change this value while the ESP32 is running, will likely lead to a
 * crash. Assign it during startup when received from storage. Can therefore
 * only be changed via reboot and DB_WIFI_MODE_DESIGNATED
 */
db_parameter_t db_param_radio_mode = {
        .db_name = "esp32_mode",
        .type = UINT8,
        .mav_t = {
                .param_name = "SYS_ESP32_MODE",
                .param_index = 0,
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
                .param_index = 1,
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
 *  disabled: only 802.11b support for client mode - set to true:
 * 802.11b/g/n/ax mode support. ax only when chip supports it.
 */
db_parameter_t db_param_wifi_en_gn = {
        .db_name = "wifi_en_gn",
        .type = UINT8,
        .mav_t = {
                .param_name = "WIFI_EN_GN",
                .param_index = 2,
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
 * Set to 1 to use external antenna. Set to 0 to enable onboard antenna - board
 * must have antenna switch Official ESP32C6 board supports this option.
 */
db_parameter_t db_param_radio_ant_ext = {
        .db_name = "ant_use_ext",
        .type = UINT8,
        .mav_t = {
                .param_name = "RADIO_EN_EXT_ANT",
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
 * UART baud rate
 */
db_parameter_t db_param_baud = {
        .db_name = "baud",
        .type = INT32,
        .mav_t = {
                .param_name = "SERIAL_BAUD",
                .param_index = 4,
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
                .param_index = 5,
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
                .param_index = 6,
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
 * RTS GPIO number of the UART. Set to same value as CTS GPIO to disable flow
 * control.
 */
db_parameter_t db_param_gpio_rts = {
        .db_name = "gpio_rts",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_RTS_PIN",
                .param_index = 7,
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
 * CTS GPIO number of the UART. Set to same value as RTS GPIO to disable flow
 * control.
 */
db_parameter_t db_param_gpio_cts = {
        .db_name = "gpio_cts",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_CTS_PIN",
                .param_index = 8,
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
 * I really don't know. Maybe some sort of timeout. Just leave it at the
 * defaults.
 */
db_parameter_t db_param_gpio_rts_thresh = {
        .db_name = "rts_thresh",
        .type = UINT8,
        .mav_t = {
                .param_name = "SERIAL_RTS_THRES",
                .param_index = 9,
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
                .param_index = 10,
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
 * Maximum packet size via ESP-NOW or WiFi in transparent or mavlink mode.
 * Caped to 250 bytes-HEADER in ESP-NOW mode.
 */
db_parameter_t db_param_serial_pack_size = {
        .db_name = "trans_pack_size",
        .type = UINT16,
        .mav_t = {
                .param_name = "SERIAL_PACK_SIZE",
                .param_index = 11,
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
 * Serial read timeout [ms] for transparent and MAVLink mode, after that the
 * packet will be sent over the air even when the max. packet size was not
 * reached.
 */
db_parameter_t db_param_serial_read_timeout = {
        .db_name = "serial_timeout",
        .type = UINT16,
        .mav_t = {
                .param_name = "SERIAL_T_OUT_MS",
                .param_index = 12,
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
                .param_index = 13,
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
 * Detects armed state from MAVLink & LTM stream and disables radio when
 * autopilot reports armed state.
 */
db_parameter_t db_param_dis_radio_armed = {
        .db_name = "radio_dis_onarm",
        .type = UINT8,
        .mav_t = {
                .param_name = "RADIO_DIS_ON_ARM",
                .param_index = 14,
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
                .param_index = 15,
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
 *  Format/Unit of the reported RSSI as part of MAVLink RADIO STATUS message.
 *  If set to true (1) the RSSI will be reported as dBm (QGC)
 *  If set to false (0) the RSSI will be calculated as a value from 0 to 100
 * (MissionPlanner)
 */
db_parameter_t db_param_rssi_dbm = {
        .db_name = "rep_rssi_dbm",
        .type = UINT8,
        .mav_t = {
                .param_name = "RADIO_RSSI_DBM",
                .param_index = 16,
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
 * Array containing all references to the DB parameters assigned with
 * db_param_init_parameters()
 */
db_parameter_t *db_params[DB_PARAM_TOTAL_NUM] = { NULL };

/**
 * Leaks memory. Only call during init of the parameters.
 * Inits a string parameter with the supplied values and ranges.
 * @param db_name NVM & web interface name/identifier
 * @param mav_param_name MAVLink parameter name
 * @param default_value Default value
 * @param min_val_len minimum length of the string (used for validation)
 * @param max_val_len maximum length of the string (used for memory allocation
 * and validation)
 * @return The inited string parameter
 */
db_parameter_t
db_param_init_str_param(char *db_name, char *mav_param_name,
                        const char *default_value, uint8_t min_val_len,
                        uint8_t max_val_len)
{
  db_parameter_t db_str_param = {
            .db_name = "",
            .type = STRING,
            .mav_t = {
                    .param_name = "",
                    .param_index = -1,
                    .param_type = MAV_PARAM_TYPE_ENUM_END, // no string support so far
            },
            .value = {
                    .db_param_str = {
                            .min_len = min_val_len,
                            .max_len = max_val_len,
                    }
            }
    };
  strncpy((char *)db_str_param.db_name, db_name, DB_PARAM_NAME_MAXLEN - 1);
  uint mav_param_name_str_len = strlen(mav_param_name);
  if(mav_param_name_str_len < DB_PARAM_MAX_MAV_PARAM_NAME_LEN) {
    // normal case - param name and string terminator fit into allocated memory
    strncpy((char *)db_str_param.mav_t.param_name,
            mav_param_name,
            DB_PARAM_MAX_MAV_PARAM_NAME_LEN - 1);
  }
  else {
    // mav param name is allowed to be 16 chars long - it is valid without a
    // string terminator if len = 16 copy the first 16 chars of string without
    // a string terminator
    memcpy((char *)db_str_param.mav_t.param_name,
           mav_param_name,
           DB_PARAM_MAX_MAV_PARAM_NAME_LEN);
  }
  db_str_param.value.db_param_str.default_value =
    (uint8_t *)strdup(default_value);
  db_str_param.value.db_param_str.value = malloc(max_val_len);
  if(db_str_param.value.db_param_str.value == NULL) {
    ESP_LOGE(TAG,
             "Error allocating %i bytes for string parameter %s",
             max_val_len,
             db_name);
  }
  else {
    // all good - init value to default value
    strncpy((char *)db_str_param.value.db_param_str.value,
            (char *)db_str_param.value.db_param_str.default_value,
            max_val_len);
    // Ensure null termination, especially if default_value is >= max_val_len.
    // For password this might not be necessary
    db_str_param.value.db_param_str.value[max_val_len - 1] = '\0';
    ESP_LOGD(TAG,
             "Initialized value for %s with default '%s'",
             db_name,
             (char *)db_str_param.value.db_param_str.value);
  }
  return db_str_param;
}

/**
 * Initializes the parameters used for storing user settings. All parameters
 * are collected in an array. Leaks memory. Only call once during startup! Add
 * new parameters here!
 */
void
db_param_init_parameters()
{
  // Wi-Fi AP SSID name OR Wi-Fi AP SSID name to connect to in Wi-Fi client
  // mode
  db_param_ssid = db_param_init_str_param(
    "ssid", "SYS_SSID", "DroneBridge for ESP32", 1, MAX_SSID_LEN);
  // Password for Wi-Fi connections & ESP-NOW encryption.
  db_param_pass =
    db_param_init_str_param("wifi_pass", "SYS_PASS", "dronebridge", 7, 64);
  // IPv4 of the Wi-Fi access point when in Wi-Fi AP mode
  db_param_wifi_ap_ip = db_param_init_str_param(
    "ap_ip", "WIFI_AP_IP", "192.168.2.1", 8, IP4ADDR_STRLEN_MAX);
  // User can specify static IP when in Wi-Fi client mode. If this is empty use
  // auto IP.
  db_param_wifi_sta_ip = db_param_init_str_param(
    "ip_sta", "WIFI_STA_IP", "", 0, IP4ADDR_STRLEN_MAX);
  // If db_param_wifi_sta_ip is set then this must be set to the gateway IP
  db_param_wifi_sta_gw = db_param_init_str_param(
    "ip_sta_gw", "WIFI_STA_GW", "", 0, IP4ADDR_STRLEN_MAX);
  // If db_param_wifi_sta_ip is set: Netmask when settings static IP in Wi-Fi
  // client mode
  db_param_wifi_sta_netmask = db_param_init_str_param(
    "ip_sta_netmsk", "WIFI_STA_NETM", "", 0, IP4ADDR_STRLEN_MAX);
  // Users can add custom UDP client targets. This is the IP of the first
  // target added. Only the first one is saved to NVM.
  db_param_udp_client_ip = db_param_init_str_param(
    "udp_client_ip", "WIFI_UDP_IP", "", 0, IP4ADDR_STRLEN_MAX);
  // Specifies the hostname. Used in Wi-Fi ap & client mode.
  db_param_wifi_hostname = db_param_init_str_param(
    "wifi_hostname", "WIFI_HOSTNAME", CONFIG_LWIP_LOCAL_HOSTNAME, 1, 32);

  db_parameter_t *db_params_l[] = { &db_param_ssid,
                                    &db_param_pass,
                                    &db_param_wifi_ap_ip,
                                    &db_param_wifi_sta_ip,
                                    &db_param_wifi_sta_gw,
                                    &db_param_wifi_sta_netmask,
                                    &db_param_udp_client_ip,
                                    &db_param_wifi_hostname,
                                    &db_param_radio_mode,
                                    &db_param_channel,
                                    &db_param_wifi_en_gn,
                                    &db_param_radio_ant_ext,
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
                                    &db_param_rssi_dbm };
  memcpy(db_params, db_params_l, sizeof(db_params_l));
}

/**
 * Sets the value of the supplied the parameter to its default value
 * @param db_parameter The parameter to reset to default
 */
void
db_param_set_to_default(db_parameter_t *db_parameter)
{
  switch(db_parameter->type) {
  case STRING:
    strncpy((char *)db_parameter->value.db_param_str.value,
            (char *)db_parameter->value.db_param_str.default_value,
            db_parameter->value.db_param_str.max_len);
    db_parameter->value.db_param_str
      .value[db_parameter->value.db_param_str.max_len - 1] = '\0';
    break;
  case UINT8:
    db_parameter->value.db_param_u8.value =
      db_parameter->value.db_param_u8.default_value;
    break;
  case UINT16:
    db_parameter->value.db_param_u16.value =
      db_parameter->value.db_param_u16.default_value;
    break;
  case INT32:
    db_parameter->value.db_param_i32.value =
      db_parameter->value.db_param_i32.default_value;
    break;
  default:
    ESP_LOGE(TAG, "db_param_set_to_default() -> db_parameter.type unknown!");
    break;
  }
}

/**
 * Helper function to reset all known parameters to their defaults. Parameter
 * must be part of db_params array
 */
void
db_param_reset_all()
{
  for(int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
    db_param_set_to_default(db_params[i]);
  }
}

/**
 * Helper function to convert all parameters with their values to a string
 * buffer for logging etc.
 * @param str_buffer Buffer to write the parameter string - must be long enough
 * ~512 bytes
 */
int
db_param_print_values_to_buffer(uint8_t *str_buffer)
{
  int str_len = 0; // overall length of the string in the str_buffer
  for(int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
    uint8_t param_str_buf[128]; // buffer for the string of a single value
    switch(db_params[i]->type) {
    case STRING:
      str_len += sprintf((char *)param_str_buf,
                         "\t%s: %s\n",
                         (char *)db_params[i]->db_name,
                         (char *)db_params[i]->value.db_param_str.value);
      break;
    case UINT8:
      str_len += sprintf((char *)param_str_buf,
                         "\t%s: %i\n",
                         (char *)db_params[i]->db_name,
                         db_params[i]->value.db_param_u8.value);
      break;
    case UINT16:
      str_len += sprintf((char *)param_str_buf,
                         "\t%s: %i\n",
                         (char *)db_params[i]->db_name,
                         db_params[i]->value.db_param_u16.value);
      break;
    case INT32:
      str_len += sprintf((char *)param_str_buf,
                         "\t%s: %li\n",
                         (char *)db_params[i]->db_name,
                         db_params[i]->value.db_param_i32.value);
      break;
    default:
      ESP_LOGE(
        TAG,
        "db_param_print_values_to_buffer() -> db_parameter.type unknown!");
      break;
    }
    strcat((char *)str_buffer,
           (char *)param_str_buf); // add the string of the individual printed
                                   // param to the big buffer
  }
  return str_len;
}

/**
 * Helper function to read a string from the NVS based on a key. Handles errors
 * accordingly and print result to console
 * @param my_handle nvs_handle to use
 * @param key NVS key as string with max length NVS_KEY_NAME_MAX_SIZE-1
 * @param dst Destination for the read value
 */
void
db_read_str_nvs(nvs_handle_t my_handle, db_parameter_t *db_param)
{
  size_t required_size = 0;
  // First call to get size
  esp_err_t err =
    nvs_get_str(my_handle, (char *)db_param->db_name, NULL, &required_size);
  if(err == ESP_OK) {
    if(required_size > db_param->value.db_param_str.max_len) {
      ESP_LOGW(TAG,
               "NVS string for %s too long (%d > %d), using default.",
               (char *)db_param->db_name,
               required_size,
               db_param->value.db_param_str.max_len);
      db_param_set_to_default(db_param); // Use default if stored value too big
    }
    else if(required_size == 0) {
      // Handle case where key exists but string is empty (e.g., saved empty
      // password)
      ESP_LOGI(TAG,
               "NVS string for %s is empty, using default.",
               (char *)db_param->db_name);
      db_param_set_to_default(db_param);
    }
    else {
      // Read the actual value
      err = nvs_get_str(my_handle,
                        (char *)db_param->db_name,
                        (char *)db_param->value.db_param_str.value,
                        &required_size);
      if(err != ESP_OK) {
        ESP_LOGE(TAG,
                 "Error (%s) reading string %s from NVS! Using default.",
                 esp_err_to_name(err),
                 (char *)db_param->db_name);
        db_param_set_to_default(db_param);
      }
      else {
        ESP_LOGD(TAG,
                 "Read %s from NVS: '%s'",
                 (char *)db_param->db_name,
                 (char *)db_param->value.db_param_str.value);
      }
    }
  }
  else if(err == ESP_ERR_NVS_NOT_FOUND) {
    // *** THIS IS THE CRITICAL PART ***
    ESP_LOGI(TAG,
             "Parameter %s not found in NVS, using default.",
             (char *)db_param->db_name);
    db_param_set_to_default(db_param); // Apply the default value
  }
  else {
    ESP_LOGE(TAG,
             "Error (%s) checking NVS for %s! Using default.",
             esp_err_to_name(err),
             (char *)db_param->db_name);
    db_param_set_to_default(db_param); // Also use default on other errors
  }
}

/**
 * Updates all parameters with their values from the NVM.
 * Parameters must be part of db_params array
 * Checks validity of read parameter and if not valid assigns default value to
 * it.
 * @param nvs_handle Opened Namespace of the NVM partition. This is the handle.
 */
void
db_param_read_all_params_nvs(const nvs_handle_t *nvs_handle)
{
  for(int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
    switch(db_params[i]->type) {
    case STRING:
      db_read_str_nvs(*nvs_handle, db_params[i]);
      if(!db_param_is_valid_str((char *)db_params[i]->value.db_param_str.value,
                                db_params[i])) {
        // read parameter is not valid - overwrite NVS value with default value
        strncpy((char *)db_params[i]->value.db_param_str.value,
                (char *)db_params[i]->value.db_param_str.default_value,
                db_params[i]->value.db_param_str.max_len);
        ESP_LOGW(
          TAG,
          "Read invalid parameter %s from NVS, setting to default value %s",
          db_params[i]->db_name,
          db_params[i]->value.db_param_str.default_value);
      }
      break;
    case UINT8:
      ESP_ERROR_CHECK_WITHOUT_ABORT(
        nvs_get_u8(*nvs_handle,
                   (char *)db_params[i]->db_name,
                   &db_params[i]->value.db_param_u8.value));
      if(!db_param_is_valid_u8(db_params[i]->value.db_param_u8.value,
                               db_params[i])) {
        // read parameter is not valid - overwrite NVS value with default value
        db_params[i]->value.db_param_u8.value =
          db_params[i]->value.db_param_u8.default_value;
        ESP_LOGW(
          TAG,
          "Read invalid parameter %s from NVS, setting to default value %i",
          db_params[i]->db_name,
          db_params[i]->value.db_param_u8.default_value);
      }
      break;
    case UINT16:
      ESP_ERROR_CHECK_WITHOUT_ABORT(
        nvs_get_u16(*nvs_handle,
                    (char *)db_params[i]->db_name,
                    &db_params[i]->value.db_param_u16.value));
      if(!db_param_is_valid_u16(db_params[i]->value.db_param_u16.value,
                                db_params[i])) {
        // read parameter is not valid - overwrite NVS value with default value
        db_params[i]->value.db_param_u16.value =
          db_params[i]->value.db_param_u16.default_value;
        ESP_LOGW(
          TAG,
          "Read invalid parameter %s from NVS, setting to default value %i",
          db_params[i]->db_name,
          db_params[i]->value.db_param_u16.default_value);
      }
      break;
    case INT32:
      ESP_ERROR_CHECK_WITHOUT_ABORT(
        nvs_get_i32(*nvs_handle,
                    (char *)db_params[i]->db_name,
                    &db_params[i]->value.db_param_i32.value));
      if(!db_param_is_valid_i32(db_params[i]->value.db_param_i32.value,
                                db_params[i])) {
        // read parameter is not valid - overwrite NVS value with default value
        db_params[i]->value.db_param_i32.value =
          db_params[i]->value.db_param_i32.default_value;
        ESP_LOGW(
          TAG,
          "Read invalid parameter %s from NVS, setting to default value %li",
          db_params[i]->db_name,
          db_params[i]->value.db_param_i32.default_value);
      }
      break;
    default:
      ESP_LOGE(
        TAG,
        "db_param_read_all_params_to_nvs() -> db_parameter.type unknown!");
      break;
    }
  }
}

/**
 * Writes all parameters with their values to the NVM. Makes an exception for
 * the mode. Here DB_RADIO_MODE_DESIGNATED is saved instead. Parameters must be
 * part of db_params array
 * @param nvs_handle Opened Namespace of the NVM partition. This is the handle.
 */
void
db_param_write_all_params_nvs(const nvs_handle_t *nvs_handle)
{
  for(int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
    switch(db_params[i]->type) {
    case STRING:
      ESP_ERROR_CHECK(
        nvs_set_str(*nvs_handle,
                    (char *)db_params[i]->db_name,
                    (char *)db_params[i]->value.db_param_str.value));
      break;
    case UINT8:
      if(strcmp((char *)db_params[i]->db_name,
                (char *)db_param_radio_mode.db_name) == 0) {
        // This is different. User writes the desired mode into
        // DB_RADIO_MODE_DESIGNATED and does not overwrite the value
        ESP_ERROR_CHECK(nvs_set_u8(*nvs_handle,
                                   (char *)db_params[i]->db_name,
                                   DB_RADIO_MODE_DESIGNATED));
      }
      else {
        ESP_ERROR_CHECK(nvs_set_u8(*nvs_handle,
                                   (char *)db_params[i]->db_name,
                                   db_params[i]->value.db_param_u8.value));
      }
      break;
    case UINT16:
      ESP_ERROR_CHECK(nvs_set_u16(*nvs_handle,
                                  (char *)db_params[i]->db_name,
                                  db_params[i]->value.db_param_u16.value));
      break;
    case INT32:
      ESP_ERROR_CHECK(nvs_set_i32(*nvs_handle,
                                  (char *)db_params[i]->db_name,
                                  db_params[i]->value.db_param_i32.value));
      break;
    default:
      ESP_LOGE(
        TAG,
        "db_param_write_all_params_to_nvs() -> db_parameter.type unknown!");
      break;
    }
  }
}

/**
 * Parses the provided json for all known parameters. Applies all recognized
 * parameter values to local storage. Checks if parameter values are within the
 * valid range. Otherwise, reject by not applying the new value. Does not save
 * them to NVM!
 * @param root_obj JSON that contains a single layer with the parameters to
 * change.
 */
void
db_param_read_all_params_json(const cJSON *root_obj)
{
  for(int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
    cJSON *jobject =
      cJSON_GetObjectItem(root_obj, (char *)db_params[i]->db_name);
    switch(db_params[i]->type) {
    case STRING:
      if(jobject) {
        if(!cJSON_IsNull(jobject)) {
          db_param_is_valid_assign_str(jobject->valuestring, db_params[i]);
        }
        else {
          // received empty string
          db_params[i]->value.db_param_str.value[0] = '\0';
        }
      }
      else {
        // do nothing - param was not found in the json
      }
      break;
    case UINT8:
      if(jobject) {
        db_param_is_valid_assign_u8(jobject->valueint, db_params[i]);
      }
      else {
        // do nothing - param was not found in the json
      }
      break;
    case UINT16:
      if(jobject) {
        db_param_is_valid_assign_u16(jobject->valueint, db_params[i]);
      }
      else {
        // do nothing - param was not found in the json
      }
      break;
    case INT32:
      if(jobject) {
        db_param_is_valid_assign_i32(jobject->valueint, db_params[i]);
      }
      else {
        // do nothing - param was not found in the json
      }
      break;
    default:
      ESP_LOGE(
        TAG,
        "db_param_write_all_params_to_nvs() -> db_parameter.type unknown!");
      break;
    }
  }
}

/**
 * Writes all known parameters to a supplied cJSON.
 * @param root_obj A cJSON object that will be filled with all known parameters
 */
void
db_param_write_all_params_json(cJSON *root_obj)
{
  for(int i = 0; i < sizeof(db_params) / sizeof(db_params[0]); i++) {
    switch(db_params[i]->type) {
    case STRING:
      cJSON_AddStringToObject(root_obj,
                              (char *)db_params[i]->db_name,
                              (char *)db_params[i]->value.db_param_str.value);
      break;
    case UINT8:
      cJSON_AddNumberToObject(root_obj,
                              (char *)db_params[i]->db_name,
                              db_params[i]->value.db_param_u8.value);
      break;
    case UINT16:
      cJSON_AddNumberToObject(root_obj,
                              (char *)db_params[i]->db_name,
                              db_params[i]->value.db_param_u16.value);
      break;
    case INT32:
      cJSON_AddNumberToObject(root_obj,
                              (char *)db_params[i]->db_name,
                              db_params[i]->value.db_param_i32.value);
      break;
    default:
      ESP_LOGE(
        TAG, "db_param_write_all_params_json() -> db_parameter.type unknown!");
      break;
    }
  }
}

/**
 * Checks if the supplied IPv4 string is representing a valid IPv4 address.
 * @param ipaddress IPv4 string
 * @return true if valid IPv4 string was supplied
 */
bool
is_valid_ip4(const char *ipaddress)
{
  struct sockaddr_in sa;
  const int result = inet_pton(AF_INET, ipaddress, &(sa.sin_addr));
  return result != 0;
}

/**
 * Checks if string is valid for assignment to the target_param.
 * @param new_string_value The string to be set as value
 * @param target_param The target parameter
 * @return true if valid or false if not
 */
bool
db_param_is_valid_str(char *new_string_value, db_parameter_t *target_param)
{
  // ToDo: Add IPv4 check for strings via custom validation function in
  // db_parameter_t
  if(new_string_value != NULL &&
     strlen(new_string_value) <= target_param->value.db_param_str.max_len &&
     strlen(new_string_value) >= target_param->value.db_param_str.min_len) {
    return true;
  }
  else {
    return false;
  }
};

/**
 * Checks if uint8 is valid for assignment to the target_param.
 * @param new_u8_value The u8 to be set as value
 * @param target_param The target parameter
 * @return true if valid or false if not
 */
bool
db_param_is_valid_u8(const uint8_t new_u8_value, db_parameter_t *target_param)
{
  if(new_u8_value <= target_param->value.db_param_u8.max &&
     new_u8_value >= target_param->value.db_param_u8.min) {
    return true;
  }
  else {
    return false;
  }
}

/**
 * Checks if uint16 is valid for assignment to the target_param.
 * @param new_u16_value The u16 to be set as value
 * @param target_param The target parameter
 * @return true if valid or false if not
 */
bool
db_param_is_valid_u16(const uint16_t new_u16_value,
                      db_parameter_t *target_param)
{
  if(new_u16_value <= target_param->value.db_param_u16.max &&
     new_u16_value >= target_param->value.db_param_u16.min) {
    return true;
  }
  else {
    return false;
  }
}

/**
 * Checks if int32 is valid for assignment to the target_param.
 * @param new_i32_value The i32 to be set as value
 * @param target_param The target parameter
 * @return true if valid or false if not
 */
bool
db_param_is_valid_i32(const int32_t new_i32_value,
                      db_parameter_t *target_param)
{
  if(new_i32_value <= target_param->value.db_param_i32.max &&
     new_i32_value >= target_param->value.db_param_i32.min) {
    return true;
  }
  else {
    return false;
  }
}

/**
 * Checks if string is valid for assignment to the target_param. If valid
 * assigns the new value to the parameter
 * @param new_string_value The new string to be checked and assigned
 * @param target_param The internal db parameter to be assigned with the new
 * value
 * @return true when valid and assigned - else false
 */
bool
db_param_is_valid_assign_str(char *new_string_value,
                             db_parameter_t *target_param)
{
  if(db_param_is_valid_str(new_string_value, target_param)) {
    strncpy((char *)target_param->value.db_param_str.value,
            new_string_value,
            DB_PARAM_VALUE_MAXLEN);
    return true;
  }
  else {
    // new value is not valid - do not assign
    ESP_LOGE(TAG,
             "db_param_is_valid_assign_str(): Invalid string length (%i-%i) "
             "or NULL for param %s",
             target_param->value.db_param_str.min_len,
             target_param->value.db_param_str.max_len,
             (char *)target_param->db_name);
    return false;
  }
}

/**
 * Checks if u8 is valid for assignment to the target_param. If valid assigns
 * the new value to the parameter
 * @param new_u8_value The new u8 to be checked and assigned
 * @param target_param The internal db parameter to be assigned with the new
 * value
 * @return true when valid and assigned - else false
 */
bool
db_param_is_valid_assign_u8(const uint8_t new_u8_value,
                            db_parameter_t *target_param)
{
  if(db_param_is_valid_u8(new_u8_value, target_param)) {
    if(strcmp((char *)target_param->db_name,
              (char *)db_param_radio_mode.db_name) == 0) {
      // Special case check: Do not directly change DB_WIFI_MODE since it is
      // not safe and constantly processed by other tasks. Save settings and
      // reboot will assign DB_RADIO_MODE_DESIGNATED to DB_WIFI_MODE
      DB_RADIO_MODE_DESIGNATED = new_u8_value;
    }
    else {
      target_param->value.db_param_u8.value =
        new_u8_value; // accept value and assign
    }
    return true;
  }
  // new value is not valid
  ESP_LOGE(TAG,
           "db_param_is_valid_assign_u8(): Value %i is out of valid range "
           "(%i-%i) for param %s",
           new_u8_value,
           target_param->value.db_param_u8.max,
           target_param->value.db_param_u8.min,
           (char *)target_param->db_name);
  return false;
}

/**
 * Checks if u16 is valid for assignment to the target_param. If valid assigns
 * the new value to the parameter
 * @param new_u16_value The new u16 to be checked and assigned
 * @param target_param The internal db parameter to be assigned with the new
 * value
 * @return true when valid and assigned - else false
 */
bool
db_param_is_valid_assign_u16(const uint16_t new_u16_value,
                             db_parameter_t *target_param)
{
  if(db_param_is_valid_u16(new_u16_value, target_param)) {
    target_param->value.db_param_u16.value =
      new_u16_value; // accept value and assign
    return true;
  }
  // new value is not valid
  ESP_LOGE(TAG,
           "db_param_is_valid_assign_u16(): Value %i is out of valid range "
           "(%i-%i) for param %s",
           new_u16_value,
           target_param->value.db_param_u16.max,
           target_param->value.db_param_u16.min,
           (char *)target_param->db_name);
  return false;
}

/**
 * Checks if i32 is valid for assignment to the target_param. If valid assigns
 * the new value to the parameter
 * @param new_i32_value The new i32 to be checked and assigned
 * @param target_param The internal db parameter to be assigned with the new
 * value
 * @return true when valid and assigned - else false
 */
bool
db_param_is_valid_assign_i32(const int32_t new_i32_value,
                             db_parameter_t *target_param)
{
  if(db_param_is_valid_i32(new_i32_value, target_param)) {
    target_param->value.db_param_i32.value =
      new_i32_value; // accept value and assign
    return true;
  }
  // new value is not valid
  ESP_LOGE(TAG,
           "db_param_is_valid_assign_i32(): Value %li is out of valid range "
           "(%li-%li) for param %s",
           new_i32_value,
           target_param->value.db_param_i32.max,
           target_param->value.db_param_i32.min,
           (char *)target_param->db_name);
  return false;
}
