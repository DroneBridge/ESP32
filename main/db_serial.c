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

#include "lwip/sockets.h"
#include "esp_log.h"
#include <esp_wifi.h>
#include <esp_timer.h>
#include <lwip/inet.h>
#include <esp_vfs_dev.h>
#include <esp_task_wdt.h>
#include <string.h>
#include <sys/param.h>
#include <sys/fcntl.h>
#include <sys/cdefs.h>
#include <stdint-gcc.h>
#include <driver/usb_serial_jtag.h>
#include "common/common.h"
#include "db_serial.h"
#include "main.h"
#include "db_esp32_control.h"
#include "tcp_server.h"
#include "db_protocol.h"
#include "msp_ltm_serial.h"
#include "globals.h"
#include "driver/uart.h"

#define FASTMAVLINK_ROUTER_LINKS_MAX  3
#define FASTMAVLINK_ROUTER_COMPONENTS_MAX  5

#include "lib/fastmavlink_router.h"
#include "db_mavlink_msgs.h"

#define TAG "DB_SERIAL"

uint8_t DB_MAV_SYS_ID = 1;

uint32_t uart_byte_count = 0;
uint8_t ltm_frame_buffer[MAX_LTM_FRAMES_IN_BUFFER * LTM_MAX_FRAME_SIZE];
uint ltm_frames_in_buffer = 0;
uint ltm_frames_in_buffer_pnt = 0;

fmav_message_t msg;

/**
 * Opens UART socket.
 * Enables UART flow control if RTS and CTS pins do NOT match.
 * Only open serial socket/UART if PINs are not matching - matching PIN nums mean they still need to be defined by
 * the user no pre-defined pins as of this release since ESP32 boards have wildly different pin configurations
 *
 * 8 data bits, no parity, 1 stop bit
 * @return ESP_ERROR or ESP_OK
 */
esp_err_t open_uart_serial_socket() {
    // only open serial socket/UART if PINs are not matching - matching PIN nums mean they still need to be defined by
    // the user no pre-defined pins as of this release since ESP32 boards have wildly different pin configurations
    if (DB_UART_PIN_RX == DB_UART_PIN_TX) {
        ESP_LOGW(TAG, "Init UART socket aborted. TX GPIO == RX GPIO - Configure first!");
        return ESP_FAIL;
    }
    bool flow_control = DB_UART_PIN_CTS != DB_UART_PIN_RTS;
    ESP_LOGI(TAG, "Flow control enabled: %s", flow_control ? "true" : "false");
    uart_config_t uart_config = {
            .baud_rate = DB_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = flow_control ? UART_HW_FLOWCTRL_CTS_RTS : UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = DB_UART_RTS_THRESH,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, DB_UART_PIN_TX, DB_UART_PIN_RX,
                                 flow_control ? DB_UART_PIN_RTS : UART_PIN_NO_CHANGE,
                                 flow_control ? DB_UART_PIN_CTS : UART_PIN_NO_CHANGE));
    return uart_driver_install(UART_NUM, 1024, 0, 10, NULL, 0);
}

esp_err_t open_jtag_serial_socket() {
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
            .rx_buffer_size = 1024 * 4,
            .tx_buffer_size = 512,
    };
    ESP_LOGI(TAG, "Initializing USB/JTAG serial interface.");
    return usb_serial_jtag_driver_install(&usb_serial_jtag_config);
}

/**
 * Opens a serial socket for communication with a serial source. On the GND this is the GCS and on the air side this is the flight controller.
 * Depending on the configuration it may open a native UART socket or a JTAG based serial interface.
 * The JTAG serial based interface is a special feature of official DroneBridge for ESP32 boards. Uses the onboard USB for serial I/O with GCS. No FTDI required.
 *
 * @return ESP_FAIL on failure
 */
esp_err_t open_serial_socket() {
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
    // open JTAG based serial socket for comms with FC or GCS via FTDI - special feature of official DB for ESP32 boards. Uses the onboard USB for serial I/O with GCS.
    // this is basically the GND-Station mode for the ESP32
    return open_jtag_serial_socket();
#else
    // open UART based serial socket for comms with FC or GCS via FTDI - configured by pins in the web interface
    return open_uart_serial_socket();
#endif
}

/**
 * Writes data from buffer to the opened serial device
 * @param data_buffer Payload to write to UART
 * @param data_length Size of payload to write to UART
 */
void write_to_serial(const uint8_t data_buffer[], const unsigned int data_length) {
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
    // Writes data from buffer to JTAG based serial interface
    int written = usb_serial_jtag_write_bytes(data_buffer, data_length, 20 / portTICK_PERIOD_MS);
    if (written != data_length) {
        ESP_LOGD(TAG, "Wrote only %i of %i bytes to JTAG", written, data_length);
    } else {
        // all good
    }
#else
    // UART based serial socket for comms with FC or GCS via FTDI - configured by pins in the web interface
    // Writes data from buffer to native UART interface
    int written = uart_write_bytes(UART_NUM, data_buffer, data_length);
    if (written != data_length) {
        // This is a debug log since it happens very rarely that not all bytes get written. Save some cpu cycles.
        ESP_LOGD(TAG, "Wrote only %i of %i bytes to UART", written, data_length);
    } else {
        // all good
    }
#endif
}

/**
 * Read data from the open serial interface
 * @param uart_read_buf Pointer to buffer to put the read bytes into
 * @param length Max length to read
 * @return number of read bytes
 */
int db_read_serial(uint8_t *uart_read_buf, uint length) {
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
    return usb_serial_jtag_read_bytes(uart_read_buf, length, 0);
#else
    // UART based serial socket for comms with FC or GCS via FTDI - configured by pins in the web interface
    return uart_read_bytes(UART_NUM, uart_read_buf, length, 0);
#endif
}

/**
 * @brief Parses & sends complete MSP & LTM messages
 */
void db_parse_msp_ltm(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t msp_message_buffer[],
                      unsigned int *serial_read_bytes,
                      msp_ltm_port_t *db_msp_ltm_port) {
    uint8_t serial_bytes[TRANS_RD_BYTES_NUM];
    unsigned int read;
    if ((read = db_read_serial(serial_bytes, TRANS_RD_BYTES_NUM)) > 0) {
        uart_byte_count += read;
        for (unsigned int j = 0; j < read; j++) {
            (*serial_read_bytes)++;
            uint8_t serial_byte = serial_bytes[j];
            if (parse_msp_ltm_byte(db_msp_ltm_port, serial_byte)) {
                msp_message_buffer[(*serial_read_bytes - 1)] = serial_byte;
                if (db_msp_ltm_port->parse_state == MSP_PACKET_RECEIVED) {
                    send_to_all_clients(tcp_clients, udp_connection, msp_message_buffer, *serial_read_bytes);
                    *serial_read_bytes = 0;
                } else if (db_msp_ltm_port->parse_state == LTM_PACKET_RECEIVED) {
                    memcpy(&ltm_frame_buffer[ltm_frames_in_buffer_pnt], db_msp_ltm_port->ltm_frame_buffer,
                           (db_msp_ltm_port->ltm_payload_cnt + 4));
                    ltm_frames_in_buffer_pnt += (db_msp_ltm_port->ltm_payload_cnt + 4);
                    ltm_frames_in_buffer++;
                    if (ltm_frames_in_buffer == DB_LTM_FRAME_NUM_BUFFER &&
                        (DB_LTM_FRAME_NUM_BUFFER <= MAX_LTM_FRAMES_IN_BUFFER)) {
                        send_to_all_clients(tcp_clients, udp_connection, ltm_frame_buffer, *serial_read_bytes);
                        ESP_LOGD(TAG, "Sent %i LTM message(s) to telemetry port!", DB_LTM_FRAME_NUM_BUFFER);
                        ltm_frames_in_buffer = 0;
                        ltm_frames_in_buffer_pnt = 0;
                        *serial_read_bytes = 0;
                    }
                }
            } else { // Leads to crashes of the ESP32 without it!
                *serial_read_bytes = 0;
            }
        }
    }
}

/**
 * Creates and writes Mavlink heartbeat message to supplied buffer
 * @param buff Buffer to write heartbeat to (>280 bytes)
 * @return Length of the message in the buffer
 */
uint16_t db_create_heartbeat(uint8_t *buff, fmav_status_t *fmav_status) {
    return fmav_msg_heartbeat_pack_to_frame_buf(
            buff, db_get_mav_sys_id(), db_get_mav_comp_id(),
            MAV_TYPE_GENERIC_MULTIROTOR, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE,
            fmav_status);
}

/**
 * We received some MAVLink request via the origin. Decide on which interface to respond with an answer
 * @param buffer Data to send
 * @param length Data length to send
 * @param origin Origin of the MAVLink request
 * @param tcp_clients List of connected TCP client
 * @param udp_conns List of active UDP connections
 */
void db_route_mavlink_response(uint8_t *buffer, uint16_t length, enum DB_MAVLINK_DATA_ORIGIN origin, int *tcp_clients,
                               udp_conn_list_t *udp_conns) {
    if (origin == DB_MAVLINK_DATA_ORIGIN_SERIAL) {
        write_to_serial(buffer, length);
    } else if (origin == DB_MAVLINK_DATA_ORIGIN_RADIO) {
        send_to_all_clients(tcp_clients, udp_conns, buffer, length);
    } else {
        ESP_LOGE(TAG, "Unknown msg origin. Do not know on which link to respond!");
    }
}

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
 * @param new_msg Message to process
 * @param tcp_clients List of connected tcp clients
 * @param origin Indicates from what kind of input/link we received the new message.
 * We expect the FC to be connected to serial port when in WiFi-AP or in WiFi-Client Mode.
 * We expect the GCS to be connected to serial port when in AP-LR or ESP-NOW GND mode.
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
                        write_to_serial(buff, length);
                    } else {
                        ESP_LOGW(TAG, "We received a heartbeat from GCS while not being in DB_WIFI_MODE_ESPNOW_GND or "
                                      "DB_WIFI_MODE_AP_LR mode. Check your configuration! AIR-Side ESP32 seems to be "
                                      "connected to GCS via UART");
                    }
                    // In AP LR mode and in ESP-NOW GND mode the heartbeat has to be emitted via UART directly to the GCS
                } else if (payload.autopilot != MAV_AUTOPILOT_INVALID && new_msg->compid == MAV_COMP_ID_AUTOPILOT1) {
                    ESP_LOGD(TAG, "Got heartbeat from flight controller (sysID: %i)", new_msg->sysid);
                    // This means we are connected to the FC since we only parse mavlink on UART and thus only see the
                    // device we are connected to via UART
                    DB_MAV_SYS_ID = new_msg->sysid;
                    // ESP32s that are connected to a flight controller via UART will send RADIO_STATUS messages to the GND
                    if (DB_WIFI_MODE == DB_WIFI_MODE_STA || DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_AIR) {
                        fmav_radio_status_t payload_r = {.fixed = 0, .rxerrors=0, .txbuf=0,
                                .noise = db_esp_signal_quality.gnd_noise_floor,
                                .remnoise = db_esp_signal_quality.air_noise_floor,
                                .remrssi = db_esp_signal_quality.air_rssi,
                                .rssi = db_esp_signal_quality.gnd_rssi};
                        uint16_t len = fmav_msg_radio_status_encode_to_frame_buf(buff, db_get_mav_sys_id(),
                                                                                 db_get_mav_comp_id(), &payload_r,
                                                                                 fmav_status);
                        send_to_all_clients(tcp_clients, udp_conns, buff, len);
                    } else if (DB_WIFI_MODE == DB_WIFI_MODE_AP && wifi_sta_list.num > 0) {
                        // we assume ESP32 is not used in DB_WIFI_MODE_AP on the ground but only on the drone side
                        // ToDo: Only the RSSI of the first client is considered.
                        //  Send each connected client its RSSI back. Easier for UDP since we have a nice list with mac addresses to use for mapping. Harder for TCP -> no macs
                        fmav_radio_status_t payload_r = {.fixed = 0, .noise = 0, .remnoise = 0, .remrssi=wifi_sta_list.sta[0].rssi, .rssi=-127, .rxerrors=0, .txbuf=0};
                        uint16_t len = fmav_msg_radio_status_encode_to_frame_buf(buff, db_get_mav_sys_id(),
                                                                                 db_get_mav_comp_id(), &payload_r,
                                                                                 fmav_status);
                        send_to_all_clients(tcp_clients, udp_conns, buff, len);
                    } else {
                        // In AP LR mode the clients will send the info to the GCS
                    }
                } else {
                    // We do not react to any other heartbeat!
                }
                // ToDo: Check if that is a good idea or push to extra thread
                uint16_t length = db_create_heartbeat(buff, fmav_status);
                // Send heartbeat to GND clients: Every ESP32 no matter its role or mode is emitting a heartbeat
                send_to_all_clients(tcp_clients, udp_conns, buff, length);
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
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            fmav_param_request_read_t payload;
            fmav_msg_param_request_read_decode(&payload, new_msg);
            float_int_union float_int;
            ESP_LOGI(TAG, "GCS request reading parameter: %s", payload.param_id);
            if (payload.param_index == -1) {
                MAV_PARAM_TYPE type = db_mav_get_parameter_value(&float_int, payload.param_id);
                if (type != 0) {
                    uint16_t len = db_get_mavmsg_param(buff, fmav_status, 0, &float_int, type, payload.param_id);
                    db_route_mavlink_response(buff, len, origin, tcp_clients, udp_conns);
                } else {
                    // send nothing, unknown parameter
                    ESP_LOGW(TAG, "\tParameter is unknown. Not responding!");
                }
            } else {

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
                MAV_PARAM_TYPE type = db_mav_get_parameter_value(&float_int, parame_set_payload.param_id);
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
            ESP_LOGI(TAG, "GCS requested reading ext parameters list");
            // ToDo Respond to PARAM_EXT_REQUEST_LIST
        }
            break;
        case FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
            ESP_LOGI(TAG, "GCS requested reading ext parameter");
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
        default: {
            if (new_msg->target_sysid == db_get_mav_sys_id() && new_msg->target_compid == db_get_mav_comp_id()) {
                ESP_LOGW(TAG, "Received unknown MAVLink message ID: %u - ignoring", new_msg->msgid);
            }
            break;
        }
    }
}

/**
 * Parses MAVLink coming from WiFi/ESPNOW - compared to the other MAVLink parsing function we do not split the stream
 * here in packets. We just want to listen and react if a packet was for us.
 *
 * @param tcp_clients Array of connected TCP clients
 * @param up_conns Structure containing all UDP connection data including the sockets
 */
void db_parse_mavlink_from_radio(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *buffer, int bytes_read) {
    static uint8_t mav_parser_rx_buf[296];  // at least 280 bytes which is the max len for a MAVLink v2 packet
    static fmav_status_t fmav_status_radio; // fmav parser status struct for radio/ESPNOW/WiFi parser

    // Parse each byte received
    for (int i = 0; i < bytes_read; ++i) {
        fmav_result_t result = {0};

        if (fmav_parse_and_check_to_frame_buf(&result, mav_parser_rx_buf, &fmav_status_radio, buffer[i])) {
            // Parser detected a full message
            // Decode message and react to it if it was for us
            fmav_frame_buf_to_msg(&msg, &result, mav_parser_rx_buf);
            if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
                if (fmav_msg_is_for_me(db_get_mav_sys_id(), db_get_mav_comp_id(), &msg)) {
                    handle_mavlink_message(&msg, tcp_clients, udp_conns, &fmav_status_radio,
                                           DB_MAVLINK_DATA_ORIGIN_RADIO);
                } else {
                    // message was not for us so ignore it
                }
            } else {
                // message had a parsing error - we cannot decode it so skip
            }
        } else {
            // do nothing since parser had a LENGTH_ERROR, CRC_ERROR or SIGNATURE_ERROR
        }
    }
    // done parsing all received data via radio link
}

/**
 * Parses MAVLink messages and sends them via the radio link.
 * This function reads data from the serial interface, parses it for complete MAVLink messages, and sends those messages in a buffer.
 * It ensures that only complete messages are sent and that the buffer does not exceed TRANS_BUFF_SIZE
 * The parsing is done semi-transparent as in: parser understands the MavLink frame format but performs no further checks
 *
 * @param tcp_clients Array of connected TCP clients
 * @param up_conns Structure containing all UDP connection data including the sockets
 * @param serial_buffer Buffer that gets filled with data and then sent via radio, shall be >x2 the max payload
 * @param serial_buff_pos Number of bytes already read for the current packet
 */
void db_read_serial_parse_mavlink(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *serial_buffer,
                                  unsigned int *serial_buff_pos) {
    static uint mav_msg_counter = 0;
    static uint8_t mav_parser_rx_buf[296];  // at least 280 bytes which is the max len for a MAVLink v2 packet
    static fmav_status_t fmav_status_serial;    // fmav parser status struct for serial parser
    uint8_t uart_read_buf[DB_TRANS_BUF_SIZE];

    // Read bytes from UART
    int bytes_read = db_read_serial(uart_read_buf, DB_TRANS_BUF_SIZE);
    uart_byte_count += bytes_read; // increase total bytes read via UART
    // Parse each byte received
    for (int i = 0; i < bytes_read; ++i) {
        fmav_result_t result = {0};

        if (fmav_parse_and_check_to_frame_buf(&result, mav_parser_rx_buf, &fmav_status_serial, uart_read_buf[i])) {
            ESP_LOGD(TAG, "Parser detected a full message (%i total): result.frame_len %i", mav_msg_counter,
                     result.frame_len);
            mav_msg_counter++;
            // Check if the new message will fit in the buffer
            if (*serial_buff_pos == 0 && result.frame_len > DB_TRANS_BUF_SIZE) {
                // frame_len is bigger than DB_TRANS_BUF_SIZE -> Split into multiple messages since e.g. ESP-NOW can only handle 250 bytes which is less than MAVLink max msg length
                uint16_t sent_bytes = 0;
                uint16_t next_chuck_len = 0;
                do {
                    next_chuck_len = result.frame_len - sent_bytes;
                    if (next_chuck_len > DB_TRANS_BUF_SIZE) {
                        next_chuck_len = DB_TRANS_BUF_SIZE;
                    } else {}
                    send_to_all_clients(tcp_clients, udp_conns, &mav_parser_rx_buf[sent_bytes], next_chuck_len);
                    sent_bytes += next_chuck_len;
                } while (sent_bytes < result.frame_len);
            } else if (*serial_buff_pos + result.frame_len > DB_TRANS_BUF_SIZE) {
                // New message won't fit into the buffer, send buffer first
                send_to_all_clients(tcp_clients, udp_conns, serial_buffer, *serial_buff_pos);
                *serial_buff_pos = 0;
                // copy the new message to the uart send buffer and set buffer position
                memcpy(&serial_buffer[*serial_buff_pos], mav_parser_rx_buf, result.frame_len);
                *serial_buff_pos += result.frame_len;
            } else {
                // copy the new message to the uart send buffer and set buffer position
                memcpy(&serial_buffer[*serial_buff_pos], mav_parser_rx_buf, result.frame_len);
                *serial_buff_pos += result.frame_len;
            }

            // Decode message and react to it if it was for us
            fmav_frame_buf_to_msg(&msg, &result, mav_parser_rx_buf);
            if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
                if (fmav_msg_is_for_me(db_get_mav_sys_id(), db_get_mav_comp_id(), &msg)) {
                    handle_mavlink_message(&msg, tcp_clients, udp_conns, &fmav_status_serial,
                                           DB_MAVLINK_DATA_ORIGIN_SERIAL);
                } else {
                    // message was not for us so ignore it
                }
            } else {
                // message had a parsing error - we cannot decode it so skip
            }
        } else {
            // do nothing since parser had a LENGTH_ERROR, CRC_ERROR or SIGNATURE_ERROR
        }
    }
    // done parsing all received data via UART
}

/**
 * Reads TRANS_RD_BYTES_NUM bytes from serial interface and checks if we already got enough bytes to send them out.
 * Requires a continuous stream of data.
 *
 * @param tcp_clients Array of connected TCP clients
 * @param udp_connection Structure containing all UDP connection data including the sockets
 * @param serial_buffer Buffer that gets filled with data and then sent via radio
 * @param serial_read_bytes Number of bytes already read for the current packet
 */
void db_read_serial_parse_transparent(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t serial_buffer[],
                                      unsigned int *serial_read_bytes) {
    uint16_t read;
    // read from UART directly into TCP & UDP send buffer
    if ((read = db_read_serial(&serial_buffer[*serial_read_bytes], (DB_TRANS_BUF_SIZE - *serial_read_bytes))) > 0) {
        uart_byte_count += read;    // increase total bytes read via UART
        *serial_read_bytes += read; // set new buffer position
    }
    // TODO: Support UART data streams that are not continuous. Use timer to check how long we waited for data already
    if (*serial_read_bytes >= DB_TRANS_BUF_SIZE) {
        send_to_all_clients(tcp_clients, udp_connection, serial_buffer, *serial_read_bytes);
        *serial_read_bytes = 0; // reset buffer position
    }
}