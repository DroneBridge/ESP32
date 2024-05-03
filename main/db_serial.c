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

#define TAG "DB_SERIAL"

uint32_t uart_byte_count = 0;
uint8_t ltm_frame_buffer[MAX_LTM_FRAMES_IN_BUFFER * LTM_MAX_FRAME_SIZE];
uint ltm_frames_in_buffer = 0;
uint ltm_frames_in_buffer_pnt = 0;

fmav_status_t fmav_status;
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
esp_err_t open_serial_socket() {
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

/**
 * Writes data from buffer to UART
 * @param data_buffer Payload to write to UART
 * @param data_length Size of payload to write to UART
 */
void write_to_uart(const uint8_t data_buffer[], const unsigned int data_length) {
    int written = uart_write_bytes(UART_NUM, data_buffer, data_length);
    if (written > 0) {
        ESP_LOGD(TAG, "Wrote %i bytes to UART", written);
    } else if (written != data_length) {
        ESP_LOGW(TAG, "Wrote 0 of %i bytes to UART.", data_length);
    } else {
        ESP_LOGE(TAG, "Error writing to UART %s", esp_err_to_name(errno));
    }

}

/**
 * @brief Parses & sends complete MSP & LTM messages
 */
void db_parse_msp_ltm(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t msp_message_buffer[],
                      unsigned int *serial_read_bytes,
                      msp_ltm_port_t *db_msp_ltm_port) {
    uint8_t serial_bytes[TRANS_RD_BYTES_NUM];
    unsigned int read;
    if ((read = uart_read_bytes(UART_NUM, serial_bytes, TRANS_RD_BYTES_NUM, 0)) > 0) {
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
 * Based on the system architecture and configured wifi mode the ESP32 may have a different role and system id.
 * Returns the best fitting component ID for the specific role.
 * @return component ID for ESP32
 */
uint8_t db_get_mav_comp_id() {
    if (DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_GND) {
        return MAV_COMP_ID_TELEMETRY_RADIO;
    } else if (DB_WIFI_MODE == DB_WIFI_MODE_AP) {
        return MAV_COMP_ID_UDP_BRIDGE;
    } else {
        return MAV_COMP_ID_UART_BRIDGE;
    }
}

/**
 * Based on the system architecture and configured wifi mode the ESP32 may have a different role and system id.
 * Returns the best fitting component ID for the specific role.
 * @return system ID for ESP32
 */
uint8_t db_get_mav_sys_id() {
    if (DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_GND) {
        return 255;
    } else if (DB_WIFI_MODE == DB_WIFI_MODE_AP) {
        return 1;
    } else if (DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_AIR) {
        return 1;
    } else {
        return 255;
    }
}

void handle_mavlink_message(fmav_message_t* new_msg) {
    switch (new_msg->msgid) {
        case FASTMAVLINK_MSG_ID_HEARTBEAT: {
            fmav_heartbeat_t payload;
            fmav_msg_heartbeat_decode(&payload, new_msg);
            if (payload.autopilot == MAV_AUTOPILOT_INVALID && payload.type == MAV_TYPE_GCS) {
                ESP_LOGI(TAG, "Got heartbeat from GCS (sysID: %i)", new_msg->sysid);
            } else if (payload.autopilot != MAV_AUTOPILOT_INVALID && new_msg->compid == MAV_COMP_ID_AUTOPILOT1) {
                ESP_LOGI(TAG, "Got heartbeat from flight controller (sysID: %i)", new_msg->sysid);
                // This means we are connected to the FC since we only parse mavlink on UART and thus only see the
                // device we are connected to via UART
                uint8_t buff[296];
//                payload.type = MAV_TYPE_ONBOARD_CONTROLLER;
//                payload.autopilot = MAV_AUTOPILOT_INVALID;
//                payload.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
//                payload.custom_mode = DB_WIFI_MODE;
//                payload.system_status = MAV_STATE_ACTIVE;
//                uint16_t len = fmav_msg_heartbeat_encode_to_frame_buf(buff, 255, MAV_COMP_ID_TELEMETRY_RADIO, &payload, &fmav_status);
                fmav_radio_status_t payload_r = {.fixed = 0, .noise = 0, .remnoise = 0, .remrssi=10, .rssi=5, .rxerrors=1, .txbuf=200};
                uint16_t lenr = fmav_msg_radio_status_encode_to_frame_buf(buff, 255, MAV_COMP_ID_TELEMETRY_RADIO, &payload_r, &fmav_status);
                int c[CONFIG_LWIP_MAX_ACTIVE_TCP] = {-1};
                send_to_all_clients(c, udp_conn_list, buff, lenr);
            } else {
                // We do not react to any other heartbeat!
            }
        }
        break;
    }
}

/**
 * Parses MAVLink messages and sends them via the radio link.
 * This function reads data from UART, parses it for complete MAVLink messages, and sends those messages in a buffer.
 * It ensures that only complete messages are sent and that the buffer does not exceed TRANS_BUFF_SIZE
 * The parsing is done semi-transparent as in: parser understands the MavLink frame format but performs no further checks
 *
 * @param tcp_clients Array of connected TCP clients
 * @param up_conns Structure containing all UDP connection data including the sockets
 * @param serial_buffer Buffer that gets filled with data and then sent via radio, shall be >x2 the max payload
 * @param serial_buff_pos Number of bytes already read for the current packet
 */
void db_parse_mavlink(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *serial_buffer, unsigned int *serial_buff_pos) {
    static uint mav_msg_counter = 0;
    static uint8_t mav_parser_rx_buf[296];  // at least 280 bytes which is the max len for a MAVLink v2 packet
    uint8_t uart_read_buf[DB_TRANS_BUF_SIZE];

    // Read bytes from UART
    int bytes_read = uart_read_bytes(UART_NUM, uart_read_buf, DB_TRANS_BUF_SIZE, 0);
    uart_byte_count += bytes_read; // increase total bytes read via UART
    // Parse each byte received
    for (int i = 0; i < bytes_read; ++i) {
        fmav_result_t result = {0};

        if (fmav_parse_and_check_to_frame_buf(&result, mav_parser_rx_buf, &fmav_status, uart_read_buf[i])) {
            ESP_LOGD(TAG, "Parser detected a full message (%i total): result.frame_len %i", mav_msg_counter, result.frame_len);
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
                    handle_mavlink_message(&msg);
                } else {
                    // ToDO: For testing and debugging we look at it anyways
                    handle_mavlink_message(&msg);
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
 * Parses MAVLink messages and sends them via the radio link.
 * This function reads data from UART, parses it for complete MAVLink messages, and sends those messages in a buffer.
 * It ensures that only complete messages are sent and that the buffer does not exceed TRANS_BUFF_SIZE
 *
 * @param tcp_clients Array of connected TCP clients
 * @param up_conns Structure containing all UDP connection data including the sockets
 * @param serial_buffer Buffer that gets filled with data and then sent via radio
 * @param serial_buff_pos Number of bytes already read for the current packet
 */
//void db_parse_mavlink(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *serial_buffer, unsigned int *serial_buff_pos) {
//    static int counter = 0;
//    uint8_t temp_buffer[DB_TRANS_BUF_SIZE];
//    mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);
//
//    // Read bytes from UART
//    int bytes_read = uart_read_bytes(UART_NUM, temp_buffer, DB_TRANS_BUF_SIZE, 0);
//    uart_byte_count += bytes_read; // increase total bytes read via UART
//    // Parse each byte received
//    for (int i = 0; i < bytes_read; ++i) {
//        if (mavlink_parse_char(MAVLINK_COMM_0, temp_buffer[i], &mav_msg, &mav_status)) {
//            ESP_LOGI(TAG, "Parser detected a full message: buffer pos: %i, mav_msg.len %i", *serial_buff_pos, mav_msg.len);
//            // Calculate the space needed for the new message
//            // ToDo: Buffer Overflow here: we write to buffer and then check if it is full or not
//            size_t message_length = mavlink_msg_to_send_buffer(serial_buffer + *serial_buff_pos, &mav_msg);
//            ESP_LOGI(TAG, "message_length: %i", message_length);
//
//            // Check if the new message will fit in the buffer
//            if (*serial_buff_pos + message_length > DB_TRANS_BUF_SIZE) {
//                // Send the buffer if the new message won't fit
//                ESP_LOGI(TAG, "Sending %i bytes to all clients", *serial_buff_pos);
//                uint f = *serial_buff_pos;
//                send_to_all_clients(tcp_clients, udp_conns, serial_buffer, f);
//                *serial_buff_pos = 0; // Reset buffer after sending
//                // send heartbeat every 10 mavlink messages via UDP
//                if (counter % 10) {
//                    counter = 0;
//                    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//                    uint16_t len = 0;
//                    create_mav_heartbeat(1,buf, &len);
//                    ESP_LOGI(TAG, "Sending heartbeat with length %i", len);
//                    send_to_all_clients(tcp_clients, udp_conns, buf, len);
//                } else {
//                    counter++;
//                }
//            } else {
//                // message does fit go on in cose
//            }
//
//            // Add the new message to the buffer
//            *serial_buff_pos += message_length;
//
//            // check if we received version 2 and request a switch.
//            if (!(mavlink_get_channel_status(MAVLINK_COMM_0)->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)) {
//                // this will only switch to proto version 2
//                chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
//                ESP_LOGI(TAG, "MAVLink switched to version 2 protocol");
//            }
//        } else {
//            // ToDo: ?
//        }
//    }
//
//    // Send any remaining complete messages in the buffer
//    if (*serial_buff_pos > 0) {
//        send_to_all_clients(tcp_clients, udp_conns, serial_buffer, *serial_buff_pos);
//    } else {
//        // ToDo
//    }
//}

/**
 * Reads TRANS_RD_BYTES_NUM bytes from UART and checks if we already got enough bytes to send them out. Requires a
 * continuos stream of data.
 *
 * @param tcp_clients Array of connected TCP clients
 * @param udp_connection Structure containing all UDP connection data including the sockets
 * @param serial_buffer Buffer that gets filled with data and then sent via radio
 * @param serial_read_bytes Number of bytes already read for the current packet
 */
void db_parse_transparent(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t serial_buffer[],
                          unsigned int *serial_read_bytes) {
    uint16_t read;
    // read from UART directly into TCP & UDP send buffer
    if ((read = uart_read_bytes(UART_NUM, &serial_buffer[*serial_read_bytes], (DB_TRANS_BUF_SIZE - *serial_read_bytes), 0)) > 0) {
        uart_byte_count += read;    // increase total bytes read via UART
        *serial_read_bytes += read; // set new buffer position
        // TODO: Support UART data streams that are not continuous. Use timer to check how long we waited for data already
        // TODO: Move below if out of the surrounding if statement
        if (*serial_read_bytes >= DB_TRANS_BUF_SIZE) {
            send_to_all_clients(tcp_clients, udp_connection, serial_buffer, *serial_read_bytes);
            *serial_read_bytes = 0; // reset buffer position
        }
    }
}