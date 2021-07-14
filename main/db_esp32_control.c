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
#include <esp_task_wdt.h>
#include <esp_vfs_dev.h>
#include <esp_wifi.h>
#include <lwip/inet.h>
#include <esp_netif_sta_list.h>
#include "esp_log.h"
#include "lwip/sockets.h"
#include "driver/uart.h"
#include "globals.h"
#include "msp_ltm_serial.h"
#include "db_protocol.h"
#include "tcp_server.h"

#define TAG "DB_CONTROL"
#define TRANS_RD_BYTES_NUM  8   // amount of bytes read form serial port at once when transparent is selected
#define UDP_BUF_SIZE    2048
#define UART_BUF_SIZE   (1024)
#define MAX_UDP_CLIENTS 8

struct db_udp_connection_t {
    int udp_socket;
    struct sockaddr_in udp_clients[MAX_UDP_CLIENTS];
    bool is_broadcast[MAX_UDP_CLIENTS];  // sockaddr_in at index is added because of station that connected
};

uint16_t app_port_proxy = APP_PORT_PROXY;
uint8_t ltm_frame_buffer[MAX_LTM_FRAMES_IN_BUFFER * LTM_MAX_FRAME_SIZE];
uint ltm_frames_in_buffer = 0;
uint ltm_frames_in_buffer_pnt = 0;

int open_serial_socket() {
    int serial_socket;
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
        close(serial_socket);
        uart_driver_delete(UART_NUM_2);
        return ESP_FAIL;
    }
    esp_vfs_dev_uart_use_driver(2);
    return serial_socket;
}

int open_udp_socket() {
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(APP_PORT_PROXY_UDP);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(server_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int udp_socket = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }
    int err = bind(udp_socket, (struct sockaddr *) &server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG, "Opened UDP socket on port %i", APP_PORT_PROXY_UDP);

    return udp_socket;
}

/**
 * Send to all connected TCP & UDP clients
 *
 * @param tcp_clients
 * @param udp_conn
 * @param data
 * @param data_length
 */
void send_to_all_clients(int tcp_clients[], struct db_udp_connection_t *udp_conn, uint8_t data[], uint data_length) {
    send_to_all_tcp_clients(tcp_clients, data, data_length);

    for (int i = 0; i < MAX_UDP_CLIENTS; i++) {  // send to all UDP clients
        if (udp_conn->udp_clients[i].sin_len > 0) {
            int sent = sendto(udp_conn->udp_socket, data, data_length, 0, (struct sockaddr *) &udp_conn->udp_clients[i],
                              sizeof(udp_conn->udp_clients[i]));
            if (sent != data_length) {
                ESP_LOGE(TAG, "UDP - Error sending (%i/%i) because of %d", sent, data_length, errno);
                udp_conn->udp_clients[i].sin_len = 0;
            }
        }
    }
}

void write_to_uart(const char tcp_client_buffer[], const size_t data_length) {
    int written = uart_write_bytes(UART_NUM_2, tcp_client_buffer, data_length);
    if (written > 0)
        ESP_LOGD(TAG, "Wrote %i bytes", written);
    else
        ESP_LOGE(TAG, "Error writing to UART %s", esp_err_to_name(errno));
}

/**
 * @brief Parses & sends complete MSP & LTM messages
 */
void parse_msp_ltm(int tcp_clients[], struct db_udp_connection_t *udp_conn, uint8_t msp_message_buffer[],
                   uint *serial_read_bytes,
                   msp_ltm_port_t *db_msp_ltm_port) {
    uint8_t serial_bytes[TRANS_RD_BYTES_NUM];
    uint read = 0;
    if ((read = uart_read_bytes(UART_NUM_2, serial_bytes, TRANS_RD_BYTES_NUM, 200 / portTICK_RATE_MS)) > 0) {
        for (uint j = 0; j < read; j++) {
            (*serial_read_bytes)++;
            uint8_t serial_byte = serial_bytes[j];
            if (parse_msp_ltm_byte(db_msp_ltm_port, serial_byte)) {
                msp_message_buffer[(*serial_read_bytes - 1)] = serial_byte;
                if (db_msp_ltm_port->parse_state == MSP_PACKET_RECEIVED) {
                    *serial_read_bytes = 0;
                    send_to_all_clients(tcp_clients, udp_conn, msp_message_buffer, *serial_read_bytes);
                } else if (db_msp_ltm_port->parse_state == LTM_PACKET_RECEIVED) {
                    memcpy(&ltm_frame_buffer[ltm_frames_in_buffer_pnt], db_msp_ltm_port->ltm_frame_buffer,
                           (db_msp_ltm_port->ltm_payload_cnt + 4));
                    ltm_frames_in_buffer_pnt += (db_msp_ltm_port->ltm_payload_cnt + 4);
                    ltm_frames_in_buffer++;
                    if (ltm_frames_in_buffer == LTM_FRAME_NUM_BUFFER &&
                        (LTM_FRAME_NUM_BUFFER <= MAX_LTM_FRAMES_IN_BUFFER)) {
                        send_to_all_clients(tcp_clients, udp_conn, ltm_frame_buffer, *serial_read_bytes);
                        ESP_LOGV(TAG, "Sent %i LTM message(s) to telemetry port!", LTM_FRAME_NUM_BUFFER);
                        ltm_frames_in_buffer = 0;
                        ltm_frames_in_buffer_pnt = 0;
                        *serial_read_bytes = 0;
                    }
                }
            }
        }
    }
}


/**
 * Reads one byte from UART and checks if we already got enough bytes to send them out
 *
 * @param tcp_clients Array of connected TCP clients
 * @param serial_read_bytes Number of bytes already read for the current packet
 */
void parse_transparent(int tcp_clients[], struct db_udp_connection_t *udp_conn, uint8_t serial_buffer[],
                       uint *serial_read_bytes) {
    uint8_t serial_bytes[TRANS_RD_BYTES_NUM];
    uint read = 0;
    if ((read = uart_read_bytes(UART_NUM_2, serial_bytes, TRANS_RD_BYTES_NUM, 200 / portTICK_RATE_MS)) > 0) {
        memcpy(&serial_buffer[*serial_read_bytes], serial_bytes, read);
        *serial_read_bytes += read;
        if (*serial_read_bytes >= TRANSPARENT_BUF_SIZE) {
            send_to_all_clients(tcp_clients, udp_conn, serial_buffer, *serial_read_bytes);
            *serial_read_bytes = 0;
        }
    }
}

/**
 * Check for incoming connections on TCP server
 *
 * @param tcp_master_socket
 * @param tcp_clients
 */
void handle_tcp_master(const int tcp_master_socket, int tcp_clients[]) {
    struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
    uint addr_len = sizeof(source_addr);
    int new_tcp_client = accept(tcp_master_socket, (struct sockaddr *) &source_addr, &addr_len);
    if (new_tcp_client > 0) {
        for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
            if (tcp_clients[i] <= 0) {
                tcp_clients[i] = new_tcp_client;
                fcntl(tcp_clients[i], F_SETFL, O_NONBLOCK);
                char addr_str[128];
                inet_ntoa_r(((struct sockaddr_in *) &source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                ESP_LOGI(TAG, "TCP: New client connected: %s", addr_str);
                return;
            }
        }
        ESP_LOGI(TAG, "TCP: Could not accept connection. Too many clients connected.");
    }
}

/**
 * Add a new client to the list of known UDP clients. Check if client is already known
 *
 * @param connections Structure containing all UDP connection information
 * @param new_client_addr Address of new client
 */
void
add_udp_to_known_clients(struct db_udp_connection_t *connections, struct sockaddr_in new_client_addr, bool is_brdcst) {
    for (int i = 0; i < MAX_UDP_CLIENTS; i++) {
        if (connections->udp_clients[i].sin_port == new_client_addr.sin_port && new_client_addr.sin_family == PF_INET &&
            ((struct sockaddr_in *) &connections->udp_clients[i])->sin_addr.s_addr ==
            ((struct sockaddr_in *) &new_client_addr)->sin_addr.s_addr) {
            return;
        } else if (connections->udp_clients[i].sin_len == 0) {
            connections->udp_clients[i] = new_client_addr;
            char addr_str[128];
            if (new_client_addr.sin_family == PF_INET) {
                inet_ntoa_r(((struct sockaddr_in *) &new_client_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
            }
            connections->is_broadcast[i] = is_brdcst;
            if (!is_brdcst)
                ESP_LOGI(TAG, "UDP: New client connected: %s:%i", addr_str, new_client_addr.sin_port);
            return;
        }
    }
}

/**
 * Get all connected clients to local AP and add to UDP connection list. List is updated every second
 *
 * @param pInt
 */
void update_udp_broadcast(int64_t *last_update, struct db_udp_connection_t *connections, const wifi_mode_t *wifi_mode) {
    if (*wifi_mode == WIFI_MODE_AP && (esp_timer_get_time() - *last_update) >= 1000000) {
        *last_update = esp_timer_get_time();
        // clear all entries
        for (int i = 0; i < MAX_UDP_CLIENTS; i++) {
            if (connections->is_broadcast[i]) {
                memset(&connections->udp_clients[i], 0, sizeof(connections->udp_clients[i]));
            }
        }
        // add based on connected stations
        wifi_sta_list_t sta_list;
        esp_netif_sta_list_t netif_sta_list;
        memset(&sta_list, 0, sizeof(sta_list));
        memset(&netif_sta_list, 0, sizeof(esp_netif_sta_list_t));
        ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&sta_list));
        ESP_ERROR_CHECK(esp_netif_get_sta_list(&sta_list, &netif_sta_list));
        for (int i = 0; i < netif_sta_list.num; i++) {
            esp_netif_sta_info_t station = netif_sta_list.sta[i];

            struct sockaddr_in new_client_addr;
            new_client_addr.sin_family = PF_INET;
            new_client_addr.sin_port = htons(APP_PORT_PROXY_UDP);
            new_client_addr.sin_len = 16;
            new_client_addr.sin_addr.s_addr = station.ip.addr;
            ESP_LOGD(TAG, "%i " IPSTR " " MACSTR "", netif_sta_list.num, IP2STR(&station.ip), MAC2STR(station.mac));
            if (station.ip.addr != 0)  // DHCP bug. Assigns 0.0.0.0 to station when directly connected on startup
                add_udp_to_known_clients(connections, new_client_addr, true);
        }
    }
}

void control_module_tcp() {
    int uart_socket = open_serial_socket();
    int tcp_master_socket = open_tcp_server(app_port_proxy);

    struct db_udp_connection_t udp_conn;
    udp_conn.udp_socket = open_udp_socket();
    fcntl(udp_conn.udp_socket, F_SETFL, O_NONBLOCK);
    char udp_buffer[UDP_BUF_SIZE];
    struct sockaddr_in udp_source_addr;
    socklen_t udp_socklen = sizeof(udp_source_addr);
    for (int i = 0; i < MAX_UDP_CLIENTS; i++)
        memset(&(udp_conn.udp_clients[i]), 0, sizeof(struct sockaddr_in));

    int tcp_clients[CONFIG_LWIP_MAX_ACTIVE_TCP];
    for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
        tcp_clients[i] = -1;
    }
    if (tcp_master_socket == ESP_FAIL || uart_socket == ESP_FAIL) {
        ESP_LOGE(TAG, "Can not start control module");
    }
    fcntl(tcp_master_socket, F_SETFL, O_NONBLOCK);
    uint read_transparent = 0;
    uint read_msp_ltm = 0;
    char tcp_client_buffer[TCP_BUFF_SIZ];
    memset(tcp_client_buffer, 0, TCP_BUFF_SIZ);
    uint8_t msp_message_buffer[UART_BUF_SIZE];
    uint8_t serial_buffer[TRANSPARENT_BUF_SIZE];
    msp_ltm_port_t db_msp_ltm_port;

    int64_t last_udp_brdc_update = esp_timer_get_time();  // time since boot for UDP broadcast update
    wifi_mode_t wifi_mode;
    esp_wifi_get_mode(&wifi_mode);

    ESP_LOGI(TAG, "Started control module");
    while (1) {
        handle_tcp_master(tcp_master_socket, tcp_clients);
        for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {  // handle TCP clients
            if (tcp_clients[i] > 0) {
                ssize_t recv_length = recv(tcp_clients[i], tcp_client_buffer, TCP_BUFF_SIZ, 0);
                if (recv_length > 0) {
                    ESP_LOGD(TAG, "TCP: Received %i bytes", recv_length);
                    write_to_uart(tcp_client_buffer, recv_length);
                } else if (recv_length == 0) {
                    shutdown(tcp_clients[i], 0);
                    close(tcp_clients[i]);
                    tcp_clients[i] = -1;
                    ESP_LOGI(TAG, "TCP client disconnected");
                } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    ESP_LOGE(TAG, "Error receiving from TCP client %i (fd: %i): %d", i, tcp_clients[i], errno);
                    shutdown(tcp_clients[i], 0);
                    close(tcp_clients[i]);
                    tcp_clients[i] = -1;
                }
            }
        }
        // handle incoming UDP data
        ssize_t recv_length = recvfrom(udp_conn.udp_socket, udp_buffer, UDP_BUF_SIZE, 0,
                                       (struct sockaddr *) &udp_source_addr, &udp_socklen);
        if (recv_length > 0) {
            ESP_LOGD(TAG, "UDP: Received %i bytes", recv_length);
            write_to_uart(udp_buffer, recv_length);
            add_udp_to_known_clients(&udp_conn, udp_source_addr, false);
        }
        update_udp_broadcast(&last_udp_brdc_update, &udp_conn, &wifi_mode);
        switch (SERIAL_PROTOCOL) {
            case 1:
            case 2:
                parse_msp_ltm(tcp_clients, &udp_conn, msp_message_buffer, &read_msp_ltm, &db_msp_ltm_port);
                break;
            default:
            case 3:
            case 4:
            case 5:
                parse_transparent(tcp_clients, &udp_conn, serial_buffer, &read_transparent);
                break;
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
void control_module() {
    xTaskCreate(&control_module_tcp, "control_tcp", 40960, NULL, 5, NULL);
}