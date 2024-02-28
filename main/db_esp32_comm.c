/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
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

#include <string.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "db_protocol.h"
#include "db_comm_protocol.h"
#include "db_comm.h"
#include "tcp_server.h"


#define TCP_COMM_BUF_SIZE 4096
#define TAG "COMM"

uint8_t tcp_comm_buffer[TCP_COMM_BUF_SIZE];
uint8_t comm_resp_buf[TCP_COMM_BUF_SIZE];

void parse_comm_protocol(int client_socket, char *new_json_bytes) {
    cJSON *json_pointer = cJSON_Parse(new_json_bytes);
    int dest = cJSON_GetObjectItem(json_pointer, DB_COMM_KEY_DEST)->valueint;
    if (dest == DB_COMM_DST_GND) {
        int resp_length = 0;

        cJSON *j_type = cJSON_GetObjectItem(json_pointer, DB_COMM_KEY_TYPE);
        char type[strlen(j_type->valuestring)];
        strcpy(type, j_type->valuestring);
        int id = cJSON_GetObjectItem(json_pointer, DB_COMM_KEY_ID)->valueint;

        if (strcmp(type, DB_COMM_TYPE_SYS_IDENT_REQUEST) == 0) {
            ESP_LOGI(TAG, "Generating SYS_IDENT_RESPONSE");
            resp_length = gen_db_comm_sys_ident_json(comm_resp_buf, id, DB_ESP32_FID);
        } else if (strcmp(type, DB_COMM_TYPE_PING_REQUEST) == 0) {
            resp_length = gen_db_comm_ping_resp(comm_resp_buf, id);
        } else {
            resp_length = gen_db_comm_err_resp(comm_resp_buf, id, "Command not supported by DB for ESP32");
        }
        if (resp_length > 0)
            lwip_send(client_socket, comm_resp_buf, resp_length, 0);
    } else {
        ESP_LOGI(TAG, "Message not for us (%i)", DB_COMM_DST_GND);
        int id = cJSON_GetObjectItem(json_pointer, DB_COMM_KEY_ID)->valueint;
        int resp_length = gen_db_comm_err_resp(comm_resp_buf, id, "Command not supported by DB for ESP32");
        lwip_send(client_socket, comm_resp_buf, resp_length, 0);
    }
    cJSON_Delete(json_pointer);
}

/** Implements some basic calls from the DroneBridge Communication Protocol
 *
 */
void communication_module_server(void *parameters) {
    int tcp_master_socket = open_tcp_server(APP_PORT_COMM);
    ESP_LOGI(TAG, "Started communication module");
    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        memset(tcp_comm_buffer, 0, TCP_COMM_BUF_SIZE);
        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint32_t addr_len = sizeof(source_addr);
        int new_tcp_client = accept(tcp_master_socket, (struct sockaddr *) &source_addr, &addr_len);
        if (new_tcp_client < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: %s", esp_err_to_name(errno));
            return;
        }
        ssize_t received_from_client;
        do {
            received_from_client = lwip_recv(new_tcp_client, tcp_comm_buffer, TCP_COMM_BUF_SIZE, 0);
            if (received_from_client > 0) {
                if (crc_ok(tcp_comm_buffer, received_from_client)) {
                    uint8_t json_byte_buf[(received_from_client - 4)];
                    memcpy(json_byte_buf, tcp_comm_buffer, (size_t) (received_from_client - 4));
                    parse_comm_protocol(new_tcp_client, (char *) tcp_comm_buffer);
                } else {
                    ESP_LOGE(TAG, "Bad CRC!");
                    uint8_t rsp_buffer[4096];
                    int resp_length = gen_db_comm_err_resp(rsp_buffer, 9999, "Bad CRC");
                    lwip_send(new_tcp_client, rsp_buffer, resp_length, 0);
                }
            }
        } while (received_from_client > 0);
        ESP_LOGI(TAG, "Client disconnected.");
        lwip_close(new_tcp_client);
    }
}


void communication_module() {
    xTaskCreatePinnedToCore(&communication_module_server, "comm_server", 8192, NULL, 5, NULL, 0);
}