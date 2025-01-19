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

uint16_t DB_MAV_PARAM_CNT = 14; // Number of MAVLink parameters returned by ESP32 in the PARAM message. Needed by GCS.

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
 * Creates and writes Mavlink heartbeat message to supplied buffer
 * @param buff Buffer to write heartbeat to (>280 bytes)
 * @return Length of the message in the buffer
 */
uint16_t db_create_heartbeat(uint8_t *buff, fmav_status_t *fmav_status) {
    return fmav_msg_heartbeat_pack_to_frame_buf(
            buff, db_get_mav_sys_id(), db_get_mav_comp_id(),
            MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE,
            fmav_status);
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
 * @param param_index Index of the parameter. May be -1 if requested parameter shall be found based on param_id
 * @return MAV_TYPE of the parameter. Returns 0 if the parameter was not found
 */
MAV_TYPE db_mav_get_parameter_value(float_int_union *float_int, char *param_id, int16_t param_index) {
    MAV_TYPE type;
    if (strncmp(param_id, "SYS_SW_VERSION", 16) == 0 || param_index == 0) {
        float_int->uint8 = DB_BUILD_VERSION;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SYS_ESP32_MODE", 16) == 0 || param_index == 1) {
        float_int->uint8 = DB_WIFI_MODE;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_PACK_SIZE", 16) == 0 || param_index == 2) {
        float_int->uint16 = DB_TRANS_BUF_SIZE;
        type = MAV_PARAM_TYPE_UINT16;
    } else if (strncmp(param_id, "SERIAL_BAUD", 16) == 0 || param_index == 3) {
        float_int->int32 = DB_UART_BAUD_RATE;
        type = MAV_PARAM_TYPE_UINT16;
    } else if (strncmp(param_id, "SERIAL_TX_PIN", 16) == 0 || param_index == 4) {
        float_int->uint8 = DB_UART_PIN_TX;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_RX_PIN", 16) == 0 || param_index == 5) {
        float_int->uint8 = DB_UART_PIN_RX;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_RTS_PIN", 16) == 0 || param_index == 6) {
        float_int->uint8 = DB_UART_PIN_RTS;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_CTS_PIN", 16) == 0 || param_index == 7) {
        float_int->uint8 = DB_UART_PIN_CTS;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_TEL_PROTO", 16) == 0 || param_index == 8) {
        float_int->uint8 = DB_SERIAL_PROTOCOL;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "WIFI_AP_CHANNEL", 16) == 0 || param_index == 9) {
        float_int->uint8 = DB_WIFI_CHANNEL;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_RTS_THRES", 16) == 0 || param_index == 10) {
        float_int->uint8 = DB_UART_RTS_THRESH;
        type = MAV_PARAM_TYPE_UINT8;
    } else if (strncmp(param_id, "SERIAL_T_OUT_MS", 16) == 0 || param_index == 11) {
        float_int->uint16 = DB_SERIAL_READ_TIMEOUT_MS;
        type = MAV_PARAM_TYPE_UINT16;
    } else if (strncmp(param_id, "WIFI_DIS_ON_ARM", 16) == 0 || param_index == 12) {
        float_int->uint8 = DB_DISABLE_WIFI_ARMED;
        type = MAV_PARAM_TYPE_UINT8;
    } else {
        type = 0;
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
        if (float_int.uint8 < DB_WIFI_MODE_ESPNOW_END) {   // check E_DB_WIFI_MODE for allowed modes
            DB_WIFI_MODE_DESIGNATED = float_int.uint8;  // do not directly change DB_WIFI_MODE since it is not safe and constantly processed by other tasks. Save settings and reboot will assign DB_WIFI_MODE_DESIGNATED to DB_WIFI_MODE
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
    } else if (strncmp(param_set_payload->param_id, "SERIAL_T_OUT_MS", 16) == 0) {
        if (float_int.uint16 > 0) {
            DB_SERIAL_READ_TIMEOUT_MS = float_int.uint16;
            success = true;
        } else {
            ESP_LOGE(TAG, "SERIAL_T_OUT_MS must be >0 MS");
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_BAUD", 16) == 0) {
        DB_UART_BAUD_RATE = float_int.int32;
        success = true;
    } else if (strncmp(param_set_payload->param_id, "SERIAL_TX_PIN", 16) == 0) {
        if (float_int.uint8 <= SOC_GPIO_IN_RANGE_MAX) {
            DB_UART_PIN_TX = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "GPIO number must be <=%i", SOC_GPIO_IN_RANGE_MAX);
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_RX_PIN", 16) == 0) {
        if (float_int.uint8 <= SOC_GPIO_IN_RANGE_MAX) {
            DB_UART_PIN_RX = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "GPIO number must be <=%i", SOC_GPIO_IN_RANGE_MAX);
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_RTS_PIN", 16) == 0) {
        if (float_int.uint8 <= SOC_GPIO_IN_RANGE_MAX) {
            DB_UART_PIN_RTS = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "GPIO number must be =<%i", SOC_GPIO_IN_RANGE_MAX);
        }
    } else if (strncmp(param_set_payload->param_id, "SERIAL_CTS_PIN", 16) == 0) {
        if (float_int.uint8 <= SOC_GPIO_IN_RANGE_MAX) {
            DB_UART_PIN_CTS = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "GPIO number must be <=%i", SOC_GPIO_IN_RANGE_MAX);
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
    } else if (strncmp(param_set_payload->param_id, "WIFI_DIS_ON_ARM", 16) == 0) {
        if (float_int.uint8 == 1 || float_int.uint8 == 0) {
            DB_DISABLE_WIFI_ARMED = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "WIFI_DIS_ON_ARM must be 1 or 0");
        }
    } else if (strncmp(param_set_payload->param_id, "WIFI_EN_EXT_ANT", 16) == 0) {
        if (float_int.uint8 == 1 || float_int.uint8 == 0) {
            DB_EN_EXT_ANT = float_int.uint8;
            success = true;
        } else {
            ESP_LOGW(TAG, "WIFI_EN_EXT_ANT must be 1 or 0");
        }
    } else {
        ESP_LOGE(TAG, "Unknown parameter value. Ignoring!");
    }
    return success;
}

/**
 * Called by db_process_mavlink_command() to process MAV_CMD_REQUEST_MESSAGE
 * @param requested_msg_id MAVLINK MSG_ID that was requested
 * @param buff Supply output buffer for sending the response
 * @param origin Origin of the command as defined by DB_MAVLINK_DATA_ORIGIN
 * @param tcp_clients List of connected tcp clients as sockets
 * @param udp_conns List of connected UDP clients
 * @param the_msg The mavlink message that was parsed (containing the_command)
 * @param status fastmavlink parser status structure
 */
void db_answer_mavlink_cmd_request_message(uint16_t requested_msg_id,
                                           uint8_t *buff, enum DB_MAVLINK_DATA_ORIGIN origin,
                                           int *tcp_clients, udp_conn_list_t *udp_conns, fmav_message_t *the_msg,
                                           fmav_status_t *status) {
    switch (requested_msg_id) {
        case FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION: {
            fmav_command_ack_t a = {.command = MAV_CMD_REQUEST_MESSAGE,
                    .result = MAV_RESULT_ACCEPTED,
                    .target_system = the_msg->sysid,
                    .target_component = the_msg->compid};
            uint16_t len = fmav_msg_command_ack_encode_to_frame_buf(buff, db_get_mav_sys_id(), db_get_mav_comp_id(), &a,
                                                                    status);
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            fmav_autopilot_version_t autopilot_version = {
                    .board_version = 0,
                    .capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE,
                    .flight_sw_version = DB_MAJOR_VERSION,
                    .middleware_sw_version = DB_MINOR_VERSION
            };
            len = fmav_msg_autopilot_version_encode_to_frame_buf(buff, db_get_mav_sys_id(), db_get_mav_comp_id(), &autopilot_version, status);
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
        }
            break;
        default: {
            ESP_LOGW(TAG, "Unsupported MavLink requested message: %i - ignoring", requested_msg_id);
        }
            break;
    }
}

/**
 * Called when a MSG_ID_COMMAND_LONG was received. Handles the command processing.
 * @param the_command The structure of the received command
 * @param the_msg The mavlink message that was parsed (containing the_command)
 * @param status fastmavlink parser status structure
 * @param buff Supply output buffer for sending data
 * @param origin Origin of the command as defined by DB_MAVLINK_DATA_ORIGIN
 * @param tcp_clients List of connected tcp clients as sockets
 * @param udp_conns List of connected UDP clients
 */
void db_process_mavlink_command(fmav_command_long_t *the_command,
                                fmav_message_t *the_msg,
                                fmav_status_t *status,
                                uint8_t *buff,
                                enum DB_MAVLINK_DATA_ORIGIN origin, int *tcp_clients, udp_conn_list_t *udp_conns) {
    switch (the_command->command) {
        case MAV_CMD_REQUEST_MESSAGE: {
            uint16_t req_msg_id = the_command->param1;
            ESP_LOGI(TAG, "\trequest for msg with ID: %i", req_msg_id);
            db_answer_mavlink_cmd_request_message(req_msg_id, buff, origin, tcp_clients, udp_conns,
                                                  the_msg, status);
        }
            break;
        default: {
            fmav_command_ack_t b = {.command = the_command->command,
                    .result = MAV_RESULT_UNSUPPORTED,
                    .target_system = the_msg->sysid,
                    .target_component = the_msg->compid};
            uint16_t len = fmav_msg_command_ack_encode_to_frame_buf(buff, db_get_mav_sys_id(), db_get_mav_comp_id(), &b,
                                                                    status);
            ESP_LOGW(TAG, "Unsupported MavLink command request: %i - ignoring", the_command->command);
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
        }
            break;
    }
}

/**
 * Expects GCS to have system ID 255.
 * Processes Mavlink messages and sends the radio status message and heartbeat to the GCS on every heartbeat received via UART
 *
 * We expect the FC to be connected to serial port when in WiFi-AP or in WiFi-Client Mode.
 * We expect the GCS to be connected to serial port when in AP-LR or ESP-NOW GND mode.
 *
 * @param new_msg Message to process
 * @param tcp_clients List of connected tcp clients
 * @param udp_conns List of connected UDP clients
 * @param fmav_status fastmavlink library parser status - setup once by the parser for a specific link/interface
 * @param origin Indicates from what kind of input/link we received the new message.
 */
void handle_mavlink_message(fmav_message_t *new_msg, int *tcp_clients, udp_conn_list_t *udp_conns,
                            fmav_status_t *fmav_status,
                            enum DB_MAVLINK_DATA_ORIGIN origin) {
    static uint8_t buff[296];   // buffer to handle the response messages - no need to init every time
    switch (new_msg->msgid) {
        case FASTMAVLINK_MSG_ID_HEARTBEAT:
            if (origin == DB_MAVLINK_DATA_ORIGIN_SERIAL) {
                // we only process heartbeats coming from the UART (local device) since we also use it as a trigger to send our heartbeat
                fmav_heartbeat_t payload;
                fmav_msg_heartbeat_decode(&payload, new_msg);
                if (payload.autopilot == MAV_AUTOPILOT_INVALID && payload.type == MAV_TYPE_GCS) {
                    ESP_LOGD(TAG, "Got heartbeat from GCS (sysID: %i)", new_msg->sysid);
                    DB_MAV_SYS_ID = new_msg->sysid;
                    // We must be in either one of these modes: AP LR or ESP-NOW GND
                    if (DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_GND || DB_WIFI_MODE == DB_WIFI_MODE_AP_LR) {
                        // Send heartbeat to GCS: Every ESP32 no matter its role or mode is emitting a heartbeat
                        uint16_t length = db_create_heartbeat(buff, fmav_status);
                        ESP_LOGD(TAG, "Sending back heartbeat via serial link to GCS");
                        // In AP LR mode and in ESP-NOW GND mode the heartbeat has to be emitted via serial directly to the GCS
                        write_to_serial(buff, length);
                    } else {
                        ESP_LOGW(TAG, "We received a heartbeat from GCS while not being in DB_WIFI_MODE_ESPNOW_GND or "
                                      "DB_WIFI_MODE_AP_LR mode. Check your configuration! AIR-Side ESP32 seems to be "
                                      "connected to GCS via UART");
                    }
                } else if (payload.autopilot != MAV_AUTOPILOT_INVALID && new_msg->compid == MAV_COMP_ID_AUTOPILOT1) {
                    ESP_LOGD(TAG, "Got heartbeat from flight controller (sysID: %i)", new_msg->sysid);
                    // This means we are connected to the FC since we only parse mavlink on UART and thus only see the
                    // device we are connected to via UART
                    DB_MAV_SYS_ID = new_msg->sysid;
                    // Check if FC is armed and the WiFi switch based on armed status is configured by the user
                    if (DB_DISABLE_WIFI_ARMED &&
                    (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED ||
                    (payload.system_status > MAV_STATE_STANDBY && payload.system_status != MAV_STATE_POWEROFF))) {
                        // autopilot indicates it is armed
                        db_set_wifi_status(false);
                    } else {
                        // autopilot indicates it is <<not>> armed
                        db_set_wifi_status(true);
                    }
                    // ESP32s that are connected to a flight controller via UART will send RADIO_STATUS messages to the GND
                    if (DB_WIFI_MODE == DB_WIFI_MODE_STA || (DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_AIR)) {
                        fmav_radio_status_t payload_r = {.fixed = 0, .txbuf=0,
                                .noise = db_esp_signal_quality.gnd_noise_floor,
                                .remnoise = db_esp_signal_quality.air_noise_floor,
                                .remrssi = db_esp_signal_quality.air_rssi,
                                .rssi = db_esp_signal_quality.gnd_rssi,
                                .rxerrors = db_esp_signal_quality.gnd_rx_packets_lost};
                        uint16_t len = fmav_msg_radio_status_encode_to_frame_buf(buff, db_get_mav_sys_id(),
                                                                                 db_get_mav_comp_id(), &payload_r,
                                                                                 fmav_status);
                        db_send_to_all_clients(tcp_clients, udp_conns, buff, len);
                    } else if (DB_WIFI_MODE == DB_WIFI_MODE_AP && wifi_sta_list.num > 0) {
                        // We assume ESP32 is not used in DB_WIFI_MODE_AP on the ground but only on the drone side -> We are in WiFi AP mode and connected to the drone
                        // Send each connected client a radio status packet.
                        // ToDo: Only the RSSI of the first client is considered. Easier for UDP since we have a nice list with mac addresses to use for mapping. Harder for TCP -> no MAC addresses available of connected clients
                        fmav_radio_status_t payload_r = {.fixed = 0, .noise = 0, .remnoise = 0, .remrssi=wifi_sta_list.sta[0].rssi, .rssi=-127, .rxerrors=0, .txbuf=0};
                        uint16_t len = fmav_msg_radio_status_encode_to_frame_buf(buff, db_get_mav_sys_id(),
                                                                                 db_get_mav_comp_id(), &payload_r,
                                                                                 fmav_status);
                        db_send_to_all_clients(tcp_clients, udp_conns, buff, len);
                    } else {
                        // In AP LR mode the clients will send the info to the GCS
                    }
                } else {
                    // We do not react to any other heartbeat!
                }
                // ToDo: Check if that is a good idea or push to extra thread
                uint16_t length = db_create_heartbeat(buff, fmav_status);
                // Send heartbeat to GND clients: Every ESP32 no matter its role or mode is emitting a heartbeat
                db_send_to_all_clients(tcp_clients, udp_conns, buff, length);
            } // do not react to heartbeats received via wireless interface - reaction to serial is sufficient
            break;
        case FASTMAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            ESP_LOGI(TAG, "Received PARAM_REQUEST_LIST msg");

            float_int_union float_int;
            float_int.uint8 = DB_BUILD_VERSION;
            uint16_t len = db_get_mavmsg_param(buff, fmav_status, 0, &float_int, MAV_PARAM_TYPE_UINT8, "SYS_SW_VERSION");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_WIFI_MODE;
            len = db_get_mavmsg_param(buff, fmav_status, 1, &float_int, MAV_PARAM_TYPE_UINT8, "SYS_ESP32_MODE");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint16 = DB_TRANS_BUF_SIZE;
            len = db_get_mavmsg_param(buff, fmav_status, 2, &float_int, MAV_PARAM_TYPE_UINT16, "SERIAL_PACK_SIZE");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.int32 = DB_UART_BAUD_RATE;
            len = db_get_mavmsg_param(buff, fmav_status, 3, &float_int, MAV_PARAM_TYPE_INT32, "SERIAL_BAUD");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_UART_PIN_TX;
            len = db_get_mavmsg_param(buff, fmav_status, 4, &float_int, MAV_PARAM_TYPE_UINT8, "SERIAL_TX_PIN");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_UART_PIN_RX;
            len = db_get_mavmsg_param(buff, fmav_status, 5, &float_int, MAV_PARAM_TYPE_UINT8, "SERIAL_RX_PIN");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_UART_PIN_RTS;
            len = db_get_mavmsg_param(buff, fmav_status, 6, &float_int, MAV_PARAM_TYPE_UINT8, "SERIAL_RTS_PIN");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_UART_PIN_CTS;
            len = db_get_mavmsg_param(buff, fmav_status, 7, &float_int, MAV_PARAM_TYPE_UINT8, "SERIAL_CTS_PIN");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_SERIAL_PROTOCOL;
            len = db_get_mavmsg_param(buff, fmav_status, 8, &float_int, MAV_PARAM_TYPE_UINT8, "SERIAL_TEL_PROTO");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_WIFI_CHANNEL;
            len = db_get_mavmsg_param(buff, fmav_status, 9, &float_int, MAV_PARAM_TYPE_UINT8, "WIFI_AP_CHANNEL");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_UART_RTS_THRESH;
            len = db_get_mavmsg_param(buff, fmav_status, 10, &float_int, MAV_PARAM_TYPE_UINT8, "SERIAL_RTS_THRES");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint16 = DB_SERIAL_READ_TIMEOUT_MS;
            len = db_get_mavmsg_param(buff, fmav_status, 11, &float_int, MAV_PARAM_TYPE_UINT16, "SERIAL_T_OUT_MS");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_DISABLE_WIFI_ARMED;
            len = db_get_mavmsg_param(buff, fmav_status, 12, &float_int, MAV_PARAM_TYPE_UINT8, "WIFI_DIS_ON_ARM");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);

            float_int.uint8 = DB_EN_EXT_ANT;
            len = db_get_mavmsg_param(buff, fmav_status, 13, &float_int, MAV_PARAM_TYPE_UINT8, "WIFI_EN_EXT_ANT");
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            fmav_param_request_read_t payload;
            fmav_msg_param_request_read_decode(&payload, new_msg);
            float_int_union float_int;
            ESP_LOGI(TAG, "GCS request reading parameter ID: %s with index %i", payload.param_id, payload.param_index);
            MAV_PARAM_TYPE type = db_mav_get_parameter_value(&float_int, payload.param_id, payload.param_index);
            if (type != 0) {
                uint16_t len = db_get_mavmsg_param(buff, fmav_status, payload.param_index, &float_int, type, payload.param_id);
                db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
            } else {
                // send nothing, unknown parameter
                ESP_LOGW(TAG, "\tParameter is unknown. Not responding!");
            }
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_SET: {
            fmav_param_set_t parame_set_payload;
            fmav_msg_param_set_decode(&parame_set_payload, new_msg);
            ESP_LOGI(TAG, "GCS requested setting parameter %s", parame_set_payload.param_id);
            if (db_write_mavlink_parameter(&parame_set_payload)) {
                // Respond with parameter
                float_int_union float_int;
                MAV_PARAM_TYPE type = db_mav_get_parameter_value(&float_int, parame_set_payload.param_id, -1);
                if (type != 0) {
                    uint16_t len = db_get_mavmsg_param(buff, fmav_status, 0, &float_int, type,
                                                       parame_set_payload.param_id);
                    db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
                    db_write_settings_to_nvs();
                } else {
                    ESP_LOGE(TAG, "Failed to set parameter %s", parame_set_payload.param_id);
                }
            }
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: {
            ESP_LOGI(TAG, "GCS requested reading ext parameters list - not supported for now!");
            // ToDo Respond to PARAM_EXT_REQUEST_LIST
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
            ESP_LOGI(TAG, "GCS requested reading ext parameter - not supported for now!");
            // ToDo Respond to EXT_REQUEST_READ
        }
            break;
        case FASTMAVLINK_MSG_ID_COMMAND_LONG: {
            fmav_command_long_t payload;
            fmav_msg_command_long_decode(&payload, new_msg);
            ESP_LOGI(TAG, "Received command long with ID: %hu", payload.command);
            db_process_mavlink_command(&payload, new_msg, fmav_status, buff, origin, tcp_clients, udp_conns);
        }
            break;
        case FASTMAVLINK_MSG_ID_PING: {
            fmav_ping_t payload;
            fmav_msg_ping_decode(&payload, new_msg);
            payload.target_system = new_msg->sysid;
            payload.target_component = new_msg->compid;
            uint16_t len = fmav_msg_ping_encode_to_frame_buf(buff, db_get_mav_sys_id(), db_get_mav_comp_id(), &payload, fmav_status);
            ESP_LOGD(TAG, "Answering MAVLink ping from System %i, Component %i", new_msg->sysid, new_msg->compid);
            db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
        }
            break;
        case FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
            fmav_request_data_stream_t data_stream_pay;
            fmav_msg_request_data_stream_decode(&data_stream_pay, new_msg);
            ESP_LOGW(TAG, "GCS requested data stream with ID: %i and rate: %i and start_stop: %i - ignoring!",
                     data_stream_pay.req_stream_id, data_stream_pay.req_message_rate, data_stream_pay.start_stop);
        }
            break;
        default: {
            if (new_msg->target_sysid == db_get_mav_sys_id() && new_msg->target_compid == db_get_mav_comp_id()) {
                ESP_LOGW(TAG, "Received unknown MAVLink message ID: %u - ignoring", new_msg->msgid);
            }
            break;
        }
    }
}


