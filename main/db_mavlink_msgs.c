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
#include <string.h>
#include <esp_log.h>
#include "db_mavlink_msgs.h"
#include "db_serial.h"
#include "globals.h"
#include "main.h"

#define FASTMAVLINK_ROUTER_LINKS_MAX  3
#define FASTMAVLINK_ROUTER_COMPONENTS_MAX  5

#define TAG "DB_MAV_MSGS"

uint16_t DB_MAV_PARAM_CNT = 10;

/**
 * Based on the system architecture and configured wifi mode the ESP32 may have a different role and system id.
 * Returns the best fitting component ID for the specific role.
 * @return component ID for ESP32
 */
uint8_t db_get_mav_comp_id() {
    return MAV_COMP_ID_TELEMETRY_RADIO;
}

/**
 * Return the Mavlink system ID. Set by handle_mavlink_message()
 * @return system ID for ESP32
 */
uint8_t db_get_mav_sys_id() {
    return DB_MAV_SYS_ID;
}

/**
 * Creates a mavlink PARAM_VALUE message inside the provided buffer using the provided parameter
 *
 * @param buff Buffer to write the mavlink message to
 * @param fmav_status fastmavlink status stucture
 * @param param_index Index of the parameter to send
 * @param value The value of the parameter to be sent -> will be converted to IEEE 745
 * @param type The MAV_PARAM_TYPE
 * @param param_id The name of the parameter (ID)
 * @return Length of the mavlink message inside the buffer
 */
uint16_t db_get_mavmsg_param(uint8_t *buff, fmav_status_t *fmav_status, uint16_t param_index, float_int_union *value, uint8_t type, char *param_id) {
    fmav_param_value_t fmav_param_value = {
            .param_value = value->f,
            .param_type = type,
            .param_count = DB_MAV_PARAM_CNT,
            .param_index = param_index};
    if (strlen(param_id)>15) {  // max size of param_id is 16 bytes
        memcpy(fmav_param_value.param_id, param_id, 16);
    } else {
        strcpy(fmav_param_value.param_id, param_id);
    }
    return fmav_msg_param_value_encode_to_frame_buf(buff, db_get_mav_sys_id(), db_get_mav_comp_id(),
                                                    &fmav_param_value, fmav_status);
}

/**
 * Gets the mavlink parameter value as float_int_union based on the parameter ID from the internal variable.
 * Maps MAVLink parameter names to the internal variable names.
 *
 * @param float_int IEEE 754 storage for the retrieved value
 * @param param_id  Parameter name you want the value of
 * @return MAV_TYPE of the parameter. Returns 0 if the parameter was not found
 */
MAV_TYPE db_mav_get_parameter_value(float_int_union *float_int, char *param_id) {
    MAV_TYPE type = 0;
    if (strncmp(param_id, "SYS_SW_VERSION", 16) == 0) {
        float_int->uint8 = DB_BUILD_VERSION;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SYS_ESP32_MODE", 16) == 0) {
        float_int->uint8 = DB_WIFI_MODE;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_PACK_SIZE", 16) == 0) {
        float_int->uint16 = DB_TRANS_BUF_SIZE;
        type = MAV_PARAM_TYPE_UINT16;
    } else if (strncmp(param_id, "SERIAL_BAUD", 16) == 0) {
        float_int->int32 = DB_UART_BAUD_RATE;
        type = MAV_PARAM_TYPE_UINT16;
    } else if (strncmp(param_id, "SERIAL_TX_PIN", 16) == 0) {
        float_int->uint8 = DB_UART_PIN_TX;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_RX_PIN", 16) == 0) {
        float_int->uint8 = DB_UART_PIN_RX;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_RTS_PIN", 16) == 0) {
        float_int->uint8 = DB_UART_PIN_RTS;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_CTS_PIN", 16) == 0) {
        float_int->uint8 = DB_UART_PIN_CTS;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_TEL_PROTO", 16) == 0) {
        float_int->uint8 = DB_SERIAL_PROTOCOL;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "WIFI_AP_CHANNEL", 16) == 0) {
        float_int->uint8 = DB_WIFI_CHANNEL;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_RTS_THRES", 16) == 0) {
        float_int->uint8 = DB_UART_RTS_THRESH;
        type = MAV_PARAM_TYPE_UINT8;
    }
    return type;
}

/**
 * Writes the parameter received via mavlink PARAM_SET to the internal variable and triggers write to NVS.
 * For some parameters to become effective the ESP32 still needs to be rebooted!
 *
 * @param fmav_param_set_payload
 * @return 1 in case of success and 0 in case of failure
 */
bool db_write_mavlink_parameter(fmav_param_set_t *param_set_payload) {
    // BEWARE: ONLY WORKS WITH NUMBERS FOR NOW! - NO SUPPORT FOR STRINGS
    float_int_union float_int;  // used to convert from IEEE 754
    float_int.f = param_set_payload->param_value;
    bool success = false;

    if (strncmp(param_set_payload->param_id, "SYS_ESP32_MODE", 16) == 0) {
        if (float_int.uint8 < 5) {   // check E_DB_WIFI_MODE for allowed modes
            DB_WIFI_MODE = float_int.uint8;
            success = true;
        } else {
            ESP_LOGE(TAG, "Unknown mode %i, not saving as new setting", float_int.uint8);
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_PACK_SIZE", 16) == 0) {
        if (float_int.uint16 > 0 && float_int.uint16 < 1024) {
            DB_TRANS_BUF_SIZE = float_int.uint16;
            success = true;
        } else {
            ESP_LOGE(TAG, "SERIAL_PACK_SIZE must be <1024 bytes");
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_BAUD", 16) == 0) {
        DB_UART_BAUD_RATE = float_int.int32;
        success = true;
    } else if (strncmp(param_set_payload->param_id, "SERIAL_TX_PIN", 16) == 0) {
        if (float_int.uint8 < 64) {
            DB_UART_PIN_TX = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "GPIO number must be <64");
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_RX_PIN", 16) == 0) {
        if (float_int.uint8 < 64) {
            DB_UART_PIN_RX = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "GPIO number must be <64");
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_RTS_PIN", 16) == 0) {
        if (float_int.uint8 < 64) {
            DB_UART_PIN_RTS = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "GPIO number must be <64");
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_CTS_PIN", 16) == 0) {
        if (float_int.uint8 < 64) {
            DB_UART_PIN_CTS = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "GPIO number must be <64");
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_TEL_PROTO", 16) == 0) {
        if (float_int.uint8 == DB_SERIAL_PROTOCOL_MAVLINK
        || float_int.uint8 == DB_SERIAL_PROTOCOL_TRANSPARENT
        || float_int.uint8 == DB_SERIAL_PROTOCOL_MSPLTM) {
            DB_SERIAL_PROTOCOL = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "Unknown serial protocol. Not writing setting.");
        }
    } else if (strncmp(param_set_payload->param_id, "WIFI_AP_CHANNEL", 16) == 0) {
        if (float_int.uint8 >= 1 && float_int.uint8 < 14) {
            DB_WIFI_CHANNEL = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "Wifi Channel must be between 1 and 13");
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_RTS_THRES", 16) == 0) {
        DB_UART_RTS_THRESH = float_int.uint8;
        success = true;
    } else {
        ESP_LOGE(TAG, "Unknown parameter value. Ignoring!");
    }
    return success;
}


