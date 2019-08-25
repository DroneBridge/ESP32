/*
 *   This file is part of DroneBridge: https://github.com/seeul8er/DroneBridge
 *
 *   Copyright 2018 Wolfgang Christl
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

#include <sys/fcntl.h>
#include <sys/param.h>
#include <string.h>
#include "esp_log.h"
#include "lwip/sockets.h"
#include "driver/uart.h"
#include "globals.h"
#include "msp_ltm_serial.h"
#include "db_protocol.h"
#include "db_esp32_control.h"


uint16_t app_port_proxy = APP_PORT_PROXY;
uint16_t app_port_telem = APP_PORT_TELEMETRY;
struct sockaddr_in client_telem_addr, client_proxy_addr, server_addr;
char const *TAG = "DB_CONTROL";
int udp_socket = -1, serial_socket = -1;
uint8_t msp_message_buffer[UART_BUF_SIZE];
uint8_t ltm_frame_buffer[MAX_LTM_FRAMES_IN_BUFFER*LTM_MAX_FRAME_SIZE];
uint ltm_frames_in_buffer = 0;
uint ltm_frames_in_buffer_pnt = 0;
char udp_buffer[UDP_BUF_SIZE];


int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    if(getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen) == -1) {
        ESP_LOGE(TAG, "getsockopt failed");
        return -1;
    }
    return result;

}

int open_udp_socket() {
    // set up UDP socket remote address to forward proxy traffic (MSP) to
    client_proxy_addr.sin_family = AF_INET;
    client_proxy_addr.sin_addr.s_addr = inet_addr(DEST_IP);
    client_proxy_addr.sin_port = htons(app_port_proxy);
    // set up UDP socket remote address to forward telemetry traffic (MAVLink, LTM) to
    client_telem_addr.sin_family = AF_INET;
    client_telem_addr.sin_addr.s_addr = inet_addr(DEST_IP);
    client_telem_addr.sin_port = htons(app_port_telem);

    // local server port we bind to
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(app_port_proxy);

    ESP_LOGI(TAG, "bound udp server to port: %d", app_port_proxy);
    udp_socket = lwip_socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        int err = get_socket_error_code(udp_socket);
        ESP_LOGE(TAG, "socket error %d %s", err, strerror(err));
        return ESP_FAIL;
    }
    if (lwip_bind(udp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        int err = get_socket_error_code(udp_socket);
        ESP_LOGE(TAG, "socket error %d %s", err, strerror(err));
        close(udp_socket);
        return ESP_FAIL;
    }
    memset(udp_buffer, 0, UDP_BUF_SIZE);
    return 1;
}

int open_serial_socket() {
    uart_config_t uart_config = {
            .baud_rate = DB_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, DB_UART_PIN_TX, DB_UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0));
    if ((serial_socket = open("/dev/uart/2", O_RDWR)) == -1) {
        ESP_LOGE(TAG, "Cannot open UART2");
        close(serial_socket); serial_socket = -1; uart_driver_delete(UART_NUM_2);
    }
    return 1;
}

/**
 * @brief Parses & sends complete MSP & LTM messages
 */
void parse_msp_ltm(){
    uint8_t serial_byte;
    msp_ltm_port_t db_msp_ltm_port;
    bool continue_reading = true;
    uint serial_read_bytes = 0;
    while (continue_reading){
        if (uart_read_bytes(UART_NUM_2, &serial_byte, 1, 200 / portTICK_RATE_MS) > 0) {
            serial_read_bytes++;
            if (parse_msp_ltm_byte(&db_msp_ltm_port, serial_byte)){
                msp_message_buffer[(serial_read_bytes-1)] = serial_byte;
                if (db_msp_ltm_port.parse_state == MSP_PACKET_RECEIVED){
                    continue_reading = false;
                    if (client_connected){
                        lwip_sendto(udp_socket, msp_message_buffer, (size_t) serial_read_bytes, 0,
                                    (struct sockaddr *) &client_proxy_addr, sizeof (client_proxy_addr));
                    }
                } else if (db_msp_ltm_port.parse_state == LTM_PACKET_RECEIVED){
                    memcpy(&ltm_frame_buffer[ltm_frames_in_buffer_pnt], &db_msp_ltm_port.ltm_frame_buffer,
                           (db_msp_ltm_port.ltm_payload_cnt + 4));
                    ltm_frames_in_buffer_pnt += (db_msp_ltm_port.ltm_payload_cnt+4);
                    ltm_frames_in_buffer++;
                    if (ltm_frames_in_buffer == LTM_FRAME_NUM_BUFFER && (LTM_FRAME_NUM_BUFFER<=MAX_LTM_FRAMES_IN_BUFFER)){
                        if (client_connected && MSP_LTM_TO_SAME_PORT){
                            lwip_sendto(udp_socket, ltm_frame_buffer, (size_t) (ltm_frames_in_buffer_pnt), 0,
                                        (struct sockaddr *) &client_proxy_addr, sizeof (client_proxy_addr));
                            ESP_LOGV(TAG, "Sent %i LTM message(s) to proxy port!", LTM_FRAME_NUM_BUFFER);
                        } else if (client_connected) {
                            lwip_sendto(udp_socket, ltm_frame_buffer, (size_t) (ltm_frames_in_buffer_pnt), 0,
                                        (struct sockaddr *) &client_telem_addr, sizeof (client_telem_addr));
                            ESP_LOGV(TAG, "Sent %i LTM message(s) to telemetry port!", LTM_FRAME_NUM_BUFFER);
                        }
                        ltm_frames_in_buffer = 0;
                        ltm_frames_in_buffer_pnt = 0;
                        continue_reading = false;
                    }
                }
            } else {
                continue_reading = false;
            }
        }
    }
}


void parse_transparent(){
    uint8_t serial_buffer[TRANSPARENT_BUF_SIZE];
    bool continue_reading = true;
    uint8_t serial_byte;
    uint serial_read_bytes = 0;
    while(continue_reading){
        if (uart_read_bytes(UART_NUM_2, &serial_byte, 1, 200 / portTICK_RATE_MS) > 0) {
            serial_buffer[serial_read_bytes] = serial_byte;
            serial_read_bytes++;
            if (serial_read_bytes == TRANSPARENT_BUF_SIZE) {
                lwip_sendto(udp_socket, serial_buffer, (size_t) serial_read_bytes, 0,
                            (struct sockaddr *) &client_telem_addr, sizeof (client_telem_addr));
                serial_read_bytes = 0;
                continue_reading = false;
                ESP_LOGV(TAG, "Sent message transparent");
            }
        }
    }
}


void control_module_uart_parser(void *parameter){
    while (true){
        switch (SERIAL_PROTOCOL){
            default:
            case 1:
            case 2:
                parse_msp_ltm();
                break;
            case 3:
            case 4:
            case 5:
                parse_transparent();
                break;
        }
    }
    vTaskDelete(NULL);
}

void control_module_udp_server(void *parameter){
    fd_set fd_socket_set;
    socklen_t proxy_addr_length;
    struct timeval tv;
    int select_return;
    while(true) {
        proxy_addr_length = sizeof(client_proxy_addr);
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        FD_ZERO(&fd_socket_set);
        FD_SET(udp_socket, &fd_socket_set);
        select_return = select(udp_socket + 1, &fd_socket_set, NULL, NULL, &tv);
        if (select_return>0){
            if (FD_ISSET(udp_socket, &fd_socket_set)){
                int recv_length = lwip_recvfrom(udp_socket, udp_buffer, UDP_BUF_SIZE, 0,
                                                  (struct sockaddr *) &client_proxy_addr, &proxy_addr_length);
                if (recv_length > 0)
                    uart_write_bytes(UART_NUM_2, udp_buffer, (size_t) recv_length);
            }
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief DroneBridge control module implementation for a ESP32 device. Bi-directional link between FC and ground. Can
 * handle MSPv1, MSPv2, LTM and MAVLink.
 * MSP & LTM is parsed and sent packet/frame by frame to ground
 * MAVLink is passed through (fully transparent). Can be used with any protocol.
 */
void control_module(){
    bool setup_success = 1;
    xEventGroupWaitBits(wifi_event_group, BIT2, false, true, portMAX_DELAY);

    if (open_udp_socket() == ESP_FAIL) setup_success = 0;
    if (open_serial_socket() == ESP_FAIL) setup_success = 0;
    if (setup_success){
        ESP_LOGI(TAG, "setup complete!");
    }

    xTaskCreate(&control_module_udp_server, "control_udp", 2048, NULL, 5, NULL);
    xTaskCreate(&control_module_uart_parser, "control_uart", 4096, NULL, 5, NULL);
}