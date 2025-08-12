/******************************************************************************
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
 ******************************************************************************/

#ifndef DB_ESP32_DB_ESP_NOW_H
#define DB_ESP32_DB_ESP_NOW_H

/******************************************************************************
 * System & Standard Library Headers
 ******************************************************************************/
#include <stdint.h>
#include <sys/cdefs.h>

/******************************************************************************
 * ESP-IDF Headers
 ******************************************************************************/
#include <esp_now.h>

/******************************************************************************
 * Macros
 ******************************************************************************/
#define DB_ESPNOW_MAX_BROADCAST_PEERS 19
#define ESPNOW_QUEUE_SIZE 6
#define ESPNOW_MAXDELAY 512
#define DB_ESPNOW_AES_IV_LEN 12 /**< 96-bit IV */
#define DB_ESPNOW_AES_TAG_LEN 16
#define DB_ESPNOW_AES_KEY_LEN 256 /**< Key length in bits */
#define DB_ESPNOW_PAYLOAD_MAXSIZE                                             \
  (ESP_NOW_MAX_DATA_LEN - DB_ESPNOW_AES_IV_LEN - DB_ESPNOW_AES_TAG_LEN - 6 - 1)

/******************************************************************************
 * Enums
 ******************************************************************************/
typedef enum
{
  DB_ESPNOW_SEND_CB,
  DB_ESPNOW_RECV_CB,
} db_espnow_event_id_t;

typedef enum
{
  DB_ESP_NOW_PACKET_TYPE_DATA = 0,
  DB_ESP_NOW_PACKET_TYPE_INTERNAL_TELEMETRY = 1,
} db_espnow_queue_event_type_t;

enum
{
  DB_ESPNOW_ORIGIN_GND = 0,
  DB_ESPNOW_ORIGIN_AIR = 1,
};

/******************************************************************************
 * Struct Definitions
 ******************************************************************************/

/******************************************************************************
 * Stores peer RSSI and sequence number tracking information
 ******************************************************************************/
typedef struct
{
  int8_t gnd_rssi;
  uint16_t last_seq_num;
  uint16_t gnd_rx_lost_packets;
  uint8_t broadcast_peer_mac[ESP_NOW_ETH_ALEN];
} __attribute__((__packed__)) db_esp_now_bpeer_info_t;

/******************************************************************************
 * ESP-NOW internal telemetry message format (sent to AIR)
 ******************************************************************************/
typedef struct
{
  uint8_t size;
  int8_t gnd_noise_floor;
  db_esp_now_bpeer_info_t db_esp_now_bpeer_info[DB_ESPNOW_MAX_BROADCAST_PEERS];
} __attribute__((__packed__)) db_esp_now_clients_list_t;

/******************************************************************************
 * ESP-NOW send callback event
 ******************************************************************************/
typedef struct
{
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];
  esp_now_send_status_t status;
} db_espnow_event_send_cb_t;

/******************************************************************************
 * ESP-NOW receive callback event
 ******************************************************************************/
typedef struct
{
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];
  int8_t rssi;
  uint8_t data_len;
  uint8_t *data;
} db_espnow_event_recv_cb_t;

/******************************************************************************
 * Union of ESP-NOW send/receive event data
 ******************************************************************************/
typedef union
{
  db_espnow_event_send_cb_t send_cb;
  db_espnow_event_recv_cb_t recv_cb;
} db_espnow_event_info_t;

/******************************************************************************
 * Event object passed from callback to ESP-NOW processing task
 ******************************************************************************/
typedef struct
{
  db_espnow_event_id_t id;
  db_espnow_event_info_t info;
} db_espnow_event_t;

/******************************************************************************
 * Queue element for both ESP-NOW and UART transmission queues
 ******************************************************************************/
typedef struct
{
  uint8_t data_len;
  uint8_t *data;
  db_espnow_queue_event_type_t packet_type;
} __attribute__((__packed__)) db_espnow_queue_event_t;

/******************************************************************************
 * ESP-NOW packet header (unencrypted, authenticated)
 ******************************************************************************/
typedef struct
{
  uint8_t origin;
  uint8_t packet_type;
  uint32_t seq_num;
  uint8_t aes_iv[DB_ESPNOW_AES_IV_LEN];
} __attribute__((__packed__)) db_esp_now_packet_header_t;

/******************************************************************************
 * ESP-NOW encrypted payload block (AES-GCM)
 ******************************************************************************/
typedef struct
{
  uint8_t payload_length_decrypted;
  uint8_t payload[DB_ESPNOW_PAYLOAD_MAXSIZE];
} __attribute__((__packed__)) db_esp_now_packet_protected_data_t;

/******************************************************************************
 * Final ESP-NOW packet structure
 * Contains authenticated header, AES-GCM tag, and encrypted payload
 ******************************************************************************/
typedef struct
{
  db_esp_now_packet_header_t db_esp_now_packet_header;
  uint8_t tag[DB_ESPNOW_AES_TAG_LEN];
  db_esp_now_packet_protected_data_t db_esp_now_packet_protected_data;
} __attribute__((__packed__)) db_esp_now_packet_t;

/******************************************************************************
 * Public Variables
 ******************************************************************************/
extern QueueHandle_t db_espnow_send_queue;
extern QueueHandle_t db_uart_write_queue;

/******************************************************************************
 * Public Function Declarations
 ******************************************************************************/

/******************************************************************************
 * @brief Start task that handles ESP-NOW data
 ******************************************************************************/
void db_start_espnow_module();

#endif // DB_ESP32_DB_ESP_NOW_H
