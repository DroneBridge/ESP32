/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2019 Wolfgang Christl
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

#include "esp_event.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

#define TCP_TAG "TCP_SERVER_SETUP"

int open_tcp_server(int port) {
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TCP_TAG, "Unable to create socket: %s", esp_err_to_name(errno));
        return ESP_FAIL;
    }
    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TCP_TAG, "Socket unable to bind: %s", esp_err_to_name(errno));
        return ESP_FAIL;
    }
    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TCP_TAG, "Error occurred during listen: %s", esp_err_to_name(errno));
        return ESP_FAIL;
    }
    ESP_LOGI(TCP_TAG, "Opened TCP server on port %d", port);
    return listen_sock;
}

void db_send_to_all_tcp_clients(const int tcp_clients[], uint8_t data[], uint data_length) {
    for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
        if (tcp_clients[i] > 0) {
            ESP_LOGD(TCP_TAG, "Sending %i bytes", data_length);
            int err = write(tcp_clients[i], data, data_length);
            if (err < 0) {
                ESP_LOGE(TCP_TAG, "Error occurred during sending: %d", errno);
            }
        }
    }

}