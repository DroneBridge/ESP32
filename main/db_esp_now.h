/*
 *   This file is part of DroneBridge:https://github.com/DroneBridge/ESP32
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

#ifndef DB_ESP32_DB_ESP_NOW_H
#define DB_ESP32_DB_ESP_NOW_H

#include <stdint.h>

#define ESPNOW_QUEUE_SIZE   6
#define ESPNOW_MAXDELAY     512

#define DB_ESPNOW_SEND_DELAY    0 //Delay between sending two ESPNOW data, unit: ms

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    DB_ESPNOW_SEND_CB,
    DB_ESPNOW_RECV_CB,
} db_espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} db_espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} db_espnow_event_recv_cb_t;

typedef union {
    db_espnow_event_send_cb_t send_cb;
    db_espnow_event_recv_cb_t recv_cb;
} db_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    db_espnow_event_id_t id;
    db_espnow_event_info_t info;
} db_espnow_event_t;

enum {
    DB_ESPNOW_DATA_BROADCAST,
    DB_ESPNOW_DATA_UNICAST,
    DB_ESPNOW_DATA_MAX,
};

enum {
    DB_ESPNOW_ORIGIN_GND = 0,
    DB_ESPNOW_ORIGIN_AIR = 1,
} db_espnow_data_origin_t;

/* DroneBridge for ESP32 ESP-NOW packet */
typedef struct {
    uint8_t origin;           // type: db_espnow_data_origin_t: 0=packet sent by GCS - 1=packet sent by drone
    uint8_t payload_length;   // length of unencrypted payload without encryption padding if length was not multiple of 16
    uint8_t *payload;         // max size is 240 bytes since it is <250bytes (ESP-NOW) and multiple of 16 (AES)
} __attribute__((packed)) db_espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool broadcast;                       //Send broadcast ESPNOW data.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} db_espnow_send_param_t;

esp_err_t db_espnow_init();
void process_espnow_data(db_espnow_send_param_t *send_param);

#endif //DB_ESP32_DB_ESP_NOW_H
