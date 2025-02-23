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
#include "fastmavlink/c_library/common/common.h"
#include "db_serial.h"
#include "main.h"
#include "db_esp32_control.h"
#include "db_protocol.h"
#include "msp_ltm_serial.h"
#include "globals.h"
#include "driver/uart.h"
#include <db_parameters.h>

#define FASTMAVLINK_ROUTER_LINKS_MAX  3
#define FASTMAVLINK_ROUTER_COMPONENTS_MAX  5

#include "fastmavlink/c_library/lib/fastmavlink_router.h"
#include "db_mavlink_msgs.h"

#define TAG "DB_SERIAL"

uint8_t DB_MAV_SYS_ID = 1;
uint32_t serial_total_byte_count = 0;
uint16_t DB_SERIAL_READ_TIMEOUT_MS = DB_SERIAL_READ_TIMEOUT_MS_DEFAULT;

uint8_t ltm_frame_buffer[MAX_LTM_FRAMES_IN_BUFFER * LTM_MAX_FRAME_SIZE];
uint ltm_frames_in_buffer = 0;
uint ltm_frames_in_buffer_pnt = 0;

fmav_message_t msg;

/**
 * Opens UART socket.
 * Enables UART flow control if RTS and CTS pins do NOT match.
 * Only open serial socket/UART if PINs are not matching - matching PIN nums mean they still need to be defined by
 * the user. No pre-defined pins since ESP32 boards have wildly different pin configurations
 *
 * 8 data bits, no parity, 1 stop bit
 * @return ESP_ERROR or ESP_OK
 */
esp_err_t open_uart_serial_socket() {
    // only open serial socket/UART if PINs are not matching - matching PIN nums mean they still need to be defined by
    // the user no pre-defined pins as of this release since ESP32 boards have wildly different pin configurations
    if (DB_PARAM_GPIO_RX == DB_PARAM_GPIO_TX) {
        ESP_LOGW(TAG, "Init UART socket aborted. TX GPIO == RX GPIO - Configure first!");
        return ESP_FAIL;
    }
    if (DB_PARAM_GPIO_TX > SOC_GPIO_IN_RANGE_MAX || DB_PARAM_GPIO_RX > SOC_GPIO_IN_RANGE_MAX || DB_PARAM_GPIO_CTS > SOC_GPIO_IN_RANGE_MAX || DB_PARAM_GPIO_RTS > SOC_GPIO_IN_RANGE_MAX) {
        ESP_LOGW(TAG, "UART GPIO numbers out of range %i. Configure first!", SOC_GPIO_IN_RANGE_MAX);
        return ESP_FAIL;
    }
    bool flow_control = DB_PARAM_GPIO_CTS != DB_PARAM_GPIO_RTS;
    ESP_LOGI(TAG, "Flow control enabled: %s", flow_control ? "true" : "false");
    uart_config_t uart_config = {
            .baud_rate = DB_PARAM_SERIAL_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = flow_control ? UART_HW_FLOWCTRL_CTS_RTS : UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = DB_PARAM_SERIAL_RTS_THRESH,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, DB_PARAM_GPIO_TX, DB_PARAM_GPIO_RX,
                                 flow_control ? DB_PARAM_GPIO_RTS : UART_PIN_NO_CHANGE,
                                 flow_control ? DB_PARAM_GPIO_CTS : UART_PIN_NO_CHANGE));
    return uart_driver_install(UART_NUM, 1024, 0, 10, NULL, 0);
}

/**
 * Configures the onboard USB/JTAG interface for serial communication (instead of an UART). Board must support this interface.
 *
 * @return result of usb_serial_jtag_driver_install()
 */
esp_err_t open_jtag_serial_socket() {
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
            .rx_buffer_size = 256,
            .tx_buffer_size = 256,
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
        ESP_LOGW(TAG, "Wrote only %i of %i bytes to JTAG", written, data_length);
    } else {
        // all good. Wrote all bytes
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
 * Read data from the open serial interface in non-blocking fashion.
 * @param uart_read_buf Pointer to buffer to put the read bytes into
 * @param length Max length to read
 * @return number of read bytes
 */
int db_read_serial(uint8_t *uart_read_buf, uint length) {
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
    return usb_serial_jtag_read_bytes(uart_read_buf, length, 0);
#else
    // UART based serial socket for communication with FC or GCS via FTDI - configured by pins in the web interface
    return uart_read_bytes(UART_NUM, uart_read_buf, length, 0);
#endif
}

/**
 * Check armed state of LTM packet if feature DB_PARAM_DIS_RADIO_ON_ARM is set and we got a status frame.
 * Triggers the enabling or disabling of the Wi-Fi.
 * @param db_msp_ltm_port MSP/LTM parser struct
 */
void db_ltm_check_arm_state_set_wifi(const msp_ltm_port_t *db_msp_ltm_port) {
    if (DB_PARAM_DIS_RADIO_ON_ARM && db_msp_ltm_port->ltm_type == LTM_TYPE_S) {
        if (db_msp_ltm_port->ltm_frame_buffer[2 + LTM_TYPE_S_PAYLOAD_SIZE] & LTM_ARMED_BIT_MASK) {
            // autopilot says it is armed
            db_set_wifi_status(false);  // disable Wi-Fi
        } else {
            // autopilot says it is <<not>> armed
            db_set_wifi_status(true);   // enable Wi-Fi
        }
    } else {
        // nothing to do
    }
}

/**
 * @brief Reads serial interface, parses & sends complete MSP & LTM messages over the air.
 */
void db_parse_msp_ltm(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t msp_message_buffer[],
                      unsigned int *serial_read_bytes,
                      msp_ltm_port_t *db_msp_ltm_port) {
    uint8_t serial_bytes[TRANS_RD_BYTES_NUM];
    unsigned int read;
    if ((read = db_read_serial(serial_bytes, TRANS_RD_BYTES_NUM)) > 0) {
        serial_total_byte_count += read;
        for (unsigned int j = 0; j < read; j++) {
            (*serial_read_bytes)++;
            uint8_t serial_byte = serial_bytes[j];
            if (parse_msp_ltm_byte(db_msp_ltm_port, serial_byte)) {
                msp_message_buffer[(*serial_read_bytes - 1)] = serial_byte;
                if (db_msp_ltm_port->parse_state == MSP_PACKET_RECEIVED) {
                    db_send_to_all_clients(tcp_clients, udp_connection, msp_message_buffer, *serial_read_bytes);
                    *serial_read_bytes = 0;
                } else if (db_msp_ltm_port->parse_state == LTM_PACKET_RECEIVED) {
                    memcpy(&ltm_frame_buffer[ltm_frames_in_buffer_pnt], db_msp_ltm_port->ltm_frame_buffer,
                           (db_msp_ltm_port->ltm_payload_cnt + 4));
                    ltm_frames_in_buffer_pnt += (db_msp_ltm_port->ltm_payload_cnt + 4);
                    ltm_frames_in_buffer++;
                    db_ltm_check_arm_state_set_wifi(db_msp_ltm_port);
                    if (ltm_frames_in_buffer == db_param_ltm_per_packet.value.db_param_u8.value &&
                        (db_param_ltm_per_packet.value.db_param_u8.value <= MAX_LTM_FRAMES_IN_BUFFER)) {
                        db_send_to_all_clients(tcp_clients, udp_connection, ltm_frame_buffer, *serial_read_bytes);
                        ESP_LOGD(TAG, "Sent %i LTM message(s) to telemetry port!", ltm_frames_in_buffer);
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
        db_send_to_all_clients(tcp_clients, udp_conns, buffer, length);
    } else {
        ESP_LOGE(TAG, "Unknown msg origin. Do not know on which link to respond!");
    }
}

/**
 * Parses MAVLink coming from WiFi/ESPNOW - and sends the packet to the serial output.
 *
 * @param tcp_clients Array of connected TCP clients
 * @param up_conns Structure containing all UDP connection data including the sockets
 * @param buffer Buffer containing the raw bytes to be parsed
 * @param bytes_read Number of bytes in the buffer
 * @param origin Origin of the data - serial link or radio link
 */
void db_parse_mavlink_from_radio(int *tcp_clients, udp_conn_list_t *udp_conns, uint8_t *buffer, int bytes_read) {
    static uint8_t mav_parser_rx_buf[296];  // at least 280 bytes which is the max len for a MAVLink v2 packet
    static fmav_status_t fmav_status_radio; // fmav parser status struct for radio/ESPNOW/WiFi parser

    // Parse each byte received
    for (int i = 0; i < bytes_read; ++i) {
        fmav_result_t result = {0};
        if (fmav_parse_and_check_to_frame_buf(&result, mav_parser_rx_buf, &fmav_status_radio, buffer[i])) {
            // Parser detected a full message, write to serial
            write_to_serial(mav_parser_rx_buf, result.frame_len);
            // Decode message and react to it if it was for us
            fmav_frame_buf_to_msg(&msg, &result, mav_parser_rx_buf);
            if (result.res == FASTMAVLINK_PARSE_RESULT_OK) {
                if (fmav_msg_is_for_me(db_get_mav_sys_id(), db_get_mav_comp_id(), &msg)) {
                    handle_mavlink_message(&msg, tcp_clients, udp_conns, &fmav_status_radio, DB_MAVLINK_DATA_ORIGIN_RADIO);
                } else {
                    // message was not for us so ignore it
                }
            } else {
                switch (result.res) {
                    case FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN:
                        ESP_LOGW(TAG, "fastmavlink parser had an error FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN msgID: %lu", result.msgid);
                        break;
                    case FASTMAVLINK_PARSE_RESULT_LENGTH_ERROR:
                        ESP_LOGW(TAG, "fastmavlink parser had an error FASTMAVLINK_PARSE_RESULT_LENGTH_ERROR msgID: %lu", result.msgid);
                        break;
                    case FASTMAVLINK_PARSE_RESULT_CRC_ERROR:
                        ESP_LOGW(TAG, "fastmavlink parser had an error FASTMAVLINK_PARSE_RESULT_CRC_ERROR msgID: %lu", result.msgid);
                        break;
                    case FASTMAVLINK_PARSE_RESULT_SIGNATURE_ERROR:
                        ESP_LOGW(TAG, "fastmavlink parser had an error FASTMAVLINK_PARSE_RESULT_SIGNATURE_ERROR msgID: %lu", result.msgid);
                        break;
                    default:
                        ESP_LOGW(TAG, "fastmavlink parser had an error parsing the message: %i", result.res);
                        break;
                }

            }
        } else {
            // do nothing since parser did not detect a message
        }
    }
    // done parsing all received data via radio link
}

/**
 * Parses MAVLink messages and sends them via the radio link.
 * This function reads data from the serial interface, parses it for complete MAVLink messages, and sends those messages in a buffer.
 * It ensures that only complete messages are sent and that the buffer does not exceed TRANS_BUFF_SIZE.
 * Checks for a serial read timeout. In case timeout is reached all data read from serial so far will be flushed to radio interface
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
    uint8_t uart_read_buf[DB_PARAM_SERIAL_PACK_SIZE];
    // timeout variables
    static TickType_t last_tick = 0;    // time when we received something from the serial interface for the last time
    static TickType_t current_tick = 0;
    current_tick = xTaskGetTickCount(); // get current time

    // Read bytes from serial link (UART or USB/JTAG interface)
    int bytes_read = db_read_serial(uart_read_buf, DB_PARAM_SERIAL_PACK_SIZE);

    if (bytes_read == 0) {
        // did not read anything this cycle -> check serial read timeout
        if (current_tick - last_tick >= pdMS_TO_TICKS(DB_SERIAL_READ_TIMEOUT_MS)) {
            // serial read timeout detected
            last_tick = current_tick;   // reset timeout
            // flush buffer to air interface -> send what we have in the buffer (already parsed)
            if (*serial_buff_pos > 0) {
                db_send_to_all_clients(tcp_clients, udp_conns, serial_buffer, *serial_buff_pos);
                *serial_buff_pos = 0;
            } else {
                // do nothing since buffer is empty anyway
            }
        } else {
            // nothing received but no timeout yet -> do nothing
        }
    } else {
        // have received something -> reset timeout
        last_tick = current_tick;
    }

    serial_total_byte_count += bytes_read; // increase total bytes read via serial interface
    // Parse each byte received
    for (int i = 0; i < bytes_read; ++i) {
        fmav_result_t result = {0};

        if (fmav_parse_and_check_to_frame_buf(&result, mav_parser_rx_buf, &fmav_status_serial, uart_read_buf[i])) {
            ESP_LOGD(TAG, "Parser detected a full message (%i total): result.frame_len %i", mav_msg_counter,
                     result.frame_len);
            mav_msg_counter++;
            // Check if the new message will fit in the buffer
            if (*serial_buff_pos == 0 && result.frame_len > DB_PARAM_SERIAL_PACK_SIZE) {
                // frame_len is bigger than DB_PARAM_SERIAL_PACK_SIZE -> Split into multiple messages since
                // e.g. ESP-NOW can only handle DB_ESPNOW_PAYLOAD_MAXSIZE bytes which is less than MAVLink max msg length
                uint16_t sent_bytes = 0;
                uint16_t next_chunk_len = 0;
                do {
                    next_chunk_len = result.frame_len - sent_bytes;
                    if (next_chunk_len > DB_PARAM_SERIAL_PACK_SIZE) {
                        next_chunk_len = DB_PARAM_SERIAL_PACK_SIZE;
                    } else {}
                    db_send_to_all_clients(tcp_clients, udp_conns, &mav_parser_rx_buf[sent_bytes], next_chunk_len);
                    sent_bytes += next_chunk_len;
                } while (sent_bytes < result.frame_len);
            } else if (*serial_buff_pos + result.frame_len > DB_PARAM_SERIAL_PACK_SIZE) {
                // New message won't fit into the buffer, send buffer first
                db_send_to_all_clients(tcp_clients, udp_conns, serial_buffer, *serial_buff_pos);
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
                    // This will also instantly send a response. That is OK at this position since we buffer and send
                    // out only full packets and this "MAVLink packet injection" into the stream will not mess with the
                    // main MAVLink packet stream.
                    handle_mavlink_message(&msg, tcp_clients, udp_conns, &fmav_status_serial,
                                           DB_MAVLINK_DATA_ORIGIN_SERIAL);
                } else {
                    // message was not for us so ignore it
                }
            } else {
                // message had a parsing error - we cannot decode it so skip
            }
        } else {
            // do nothing since parser had a now new message, LENGTH_ERROR, CRC_ERROR or SIGNATURE_ERROR
        }
    }
    // done parsing all received data via UART
}

/**
 * Reads TRANS_RD_BYTES_NUM bytes from serial interface and checks if we already got enough bytes to send them out.
 * Timeout ensures that no data is getting stuck in the buffer. Once serial read timeout is reached, the buffer will be
 * flushed to the radio interface (send what we have)
 *
 * @param tcp_clients Array of connected TCP clients
 * @param udp_connection Structure containing all UDP connection data including the sockets
 * @param serial_buffer Buffer that gets filled with data and then sent via radio
 * @param serial_read_bytes Number of bytes already read for the current packet
 */
void db_read_serial_parse_transparent(int tcp_clients[], udp_conn_list_t *udp_connection, uint8_t serial_buffer[],
                                      unsigned int *serial_read_bytes) {
    uint16_t read;
    static bool serial_read_timeout_reached = false;
    static TickType_t last_tick = 0;    // time when we received something from the serial interface for the last time
    static TickType_t current_tick = 0;
    current_tick = xTaskGetTickCount(); // get current time
    // read from UART directly into TCP & UDP send buffer
    if ((read = db_read_serial(&serial_buffer[*serial_read_bytes], (DB_PARAM_SERIAL_PACK_SIZE - *serial_read_bytes))) > 0) {
        serial_total_byte_count += read;    // increase total bytes read via UART
        *serial_read_bytes += read; // set new buffer position
        serial_read_timeout_reached = false;    // reset serial read timeout
        last_tick = current_tick;               // reset time for serial read timeout
    } else {
        /* did not read anything this cycle -> check serial read timeout */
        if (current_tick - last_tick >= pdMS_TO_TICKS(DB_SERIAL_READ_TIMEOUT_MS)) {
            serial_read_timeout_reached = true;
            last_tick = current_tick;
        } else {
            // no timeout detected
        }
    }
    // send serial data over the air interface
    if (*serial_read_bytes >= DB_PARAM_SERIAL_PACK_SIZE || (serial_read_timeout_reached && *serial_read_bytes > 0)) {
        db_send_to_all_clients(tcp_clients, udp_connection, serial_buffer, *serial_read_bytes);
        *serial_read_bytes = 0; // reset buffer position
        serial_read_timeout_reached = false;    // reset serial read timeout
    }
}