#include <sys/cdefs.h>
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
#include <esp_now.h>

#define ESPNOW_QUEUE_SIZE   6
#define ESPNOW_MAXDELAY     512
#define DB_ESPNOW_AES_IV_LEN       12   // 96 bit
#define DB_ESPNOW_AES_TAG_LEN      16
#define DB_ESPNOW_AES_KEY_LEN      256  // in bits 128 & 192 are supported by ESP32
#define DB_ESPNOW_PAYLOAD_MAXSIZE  (ESP_NOW_MAX_DATA_LEN-DB_ESPNOW_AES_IV_LEN-DB_ESPNOW_AES_TAG_LEN-6-1) // (origin, packet_type, seq_num) = 6 bytes, payload_length_decrypted = 1 byte

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0)

extern QueueHandle_t db_espnow_send_queue;    // Queue that contains data to be sent via ESP-NOW (filled by control task)
extern QueueHandle_t db_uart_write_queue;    // Queue that contains data to be written to UART (filled by ESP-NOW task)

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
    uint8_t data_len;
    uint8_t *data;
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

/* Element of db_espnow_send_queue or db_uart_write_queue */
typedef struct {
    uint8_t data_len;
    uint8_t *data;
} __attribute__((__packed__)) db_espnow_uart_event_t;

enum {
    DB_ESPNOW_ORIGIN_GND = 0,
    DB_ESPNOW_ORIGIN_AIR = 1,
};

typedef struct {
    uint8_t origin;                         // type: db_espnow_data_origin_t: 0=packet sent by GCS - 1=packet sent by drone
    uint8_t packet_type;                    // FEC packet type or non FEC protected data ToDo: Check if really necessary
    uint32_t seq_num;                       // Sequence number of the packet
    uint8_t aes_iv[DB_ESPNOW_AES_IV_LEN];   // Initialization vector for AES
} __attribute__((__packed__)) db_esp_now_packet_header_t;             // authenticated but NOT encrypted by AES-GCM

typedef struct {
    uint8_t payload_length_decrypted;           // length of unencrypted payload
    uint8_t payload[DB_ESPNOW_PAYLOAD_MAXSIZE]; // actual payload data
} __attribute__((__packed__)) db_esp_now_packet_protected_data_t; // encrypted & authenticated by AES-GCM

/* DroneBridge for ESP32 ESP-NOW packet */
typedef struct {
    db_esp_now_packet_header_t db_esp_now_packet_header;
    uint8_t tag[DB_ESPNOW_AES_TAG_LEN];     // AES-GCM Tag
    db_esp_now_packet_protected_data_t db_esp_now_packet_protected_data;    // shall be last in struct, so we can cut off when payload is smaller
} __attribute__((__packed__)) db_esp_now_packet_t;  // total size must be <=250bytes (ESP-NOW requirement)

void db_espnow_module();

_Noreturn void process_espnow_data();

#endif //DB_ESP32_DB_ESP_NOW_H
