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

#include <esp_log.h>
#include <string.h>
#include <cJSON.h>
#include "lwip/sockets.h"
#include "globals.h"
#include "db_esp32_control.h"
#include "db_protocol.h"
#include "db_comm_protocol.h"
#include "db_comm.h"


#define UDP_COMM_BUF_SIZE 2048

const char *TAGC = "DB_COMM";
struct sockaddr_in client_comm_addr, server_addr;
uint8_t udp_comm_buffer[UDP_COMM_BUF_SIZE];
int comm_udp_socket;
uint16_t app_port_comm = APP_PORT_COMM;
const int accepted_dest = 1;    // only process comm messages that are for groundstation only
uint8_t comm_resp_buf[UDP_COMM_BUF_SIZE];

void parse_comm_protocol(char *new_json_bytes){
    cJSON *json_pointer = cJSON_Parse(new_json_bytes);
    int dest = cJSON_GetObjectItem(json_pointer, DB_COMM_KEY_DEST)->valueint;
    if (dest == accepted_dest){
        int resp_length = 0;

        cJSON *j_type = cJSON_GetObjectItem(json_pointer, DB_COMM_KEY_TYPE);
        char type[strlen(j_type->valuestring)];
        strcpy(type, j_type->valuestring);
        int id = cJSON_GetObjectItem(json_pointer, DB_COMM_KEY_ID)->valueint;

        if (strcmp(type, DB_COMM_TYPE_SYS_IDENT_REQUEST) == 0){
            ESP_LOGI(TAGC, "Generating SYS_IDENT_RESPONSE");
            resp_length = gen_db_comm_sys_ident_json(comm_resp_buf, id, 101);
        } else if (strcmp(type, DB_COMM_TYPE_SETTINGS_CHANGE) == 0){
            // TODO
        } else if (strcmp(type, DB_COMM_TYPE_SETTINGS_REQUEST) == 0){
            // TODO
        } else if (strcmp(type, DB_COMM_TYPE_SETTINGS_RESPONSE) == 0){
            // TODO
        } else if (strcmp(type, DB_COMM_TYPE_SETTINGS_CHANGE) == 0){
            // TODO
        }
        if (resp_length > 0)
            lwip_sendto(comm_udp_socket, comm_resp_buf, (size_t) resp_length, 0,
                        (struct sockaddr *) &client_comm_addr, sizeof (client_comm_addr));
    } else {
        ESP_LOGI(TAGC, "Message not for us (%i)", accepted_dest);
    }
    cJSON_Delete(json_pointer);
}

int open_comm_udp_socket() {
    // set up UDP socket remote address to forward proxy traffic (MSP) to
    client_comm_addr.sin_family = AF_INET;
    client_comm_addr.sin_addr.s_addr = inet_addr(DEST_IP);
    client_comm_addr.sin_port = htons(app_port_comm);

    // local server port we bind to
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(APP_PORT_COMM);

    ESP_LOGI(TAGC, "bound udp server to port: %d", app_port_comm);
    comm_udp_socket = lwip_socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (comm_udp_socket < 0) {
        int err = get_socket_error_code(comm_udp_socket);
        ESP_LOGE(TAGC, "socket error %d %s", err, strerror(err));
        return ESP_FAIL;
    }
    if (lwip_bind(comm_udp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        int err = get_socket_error_code(comm_udp_socket);
        ESP_LOGE(TAGC, "socket error %d %s", err, strerror(err));
        close(comm_udp_socket);
        return ESP_FAIL;
    }
    return 1;
}

void communication_module_server(void *parameters){
    open_comm_udp_socket();
    ESP_LOGI(TAGC, "Started communication module");
    while(true){
        memset(udp_comm_buffer, 0, UDP_COMM_BUF_SIZE);
        int msg_length = lwip_recvfrom(comm_udp_socket, udp_comm_buffer, UDP_COMM_BUF_SIZE, 0,
                                        (struct sockaddr *) &client_comm_addr, (socklen_t *) &client_comm_addr);
        ESP_LOGI(TAGC, "Got UDP packet");
        if (crc_ok(udp_comm_buffer, msg_length)){
            uint8_t json_byte_buf[(msg_length-4)];
            memcpy(json_byte_buf, udp_comm_buffer, (size_t) (msg_length-4));
            parse_comm_protocol((char *) udp_comm_buffer);
        } else {
            ESP_LOGI(TAGC, "Bad CRC!");
        }
    }
}


void communication_module(){
    xTaskCreate(&communication_module_server, "comm_server", 8192, NULL, 5, NULL);
}