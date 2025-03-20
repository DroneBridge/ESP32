
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

#include <esp_now.h>
#include <stdint.h>
#include <sys/cdefs.h>

/***************************************************************************************************************************
 * Public Macros
 **************************************************************************************************************************/

#define DB_ESPNOW_MAX_BROADCAST_PEERS                                                                                       \
  19 // Number of max. broadcast peers. that we support with internal telemetry. Limit is 255, but this is the max we can fit
     // into one packet 250bytes
#define ESPNOW_QUEUE_SIZE     6   //
#define ESPNOW_MAXDELAY       512 //
#define DB_ESPNOW_AES_IV_LEN  12  // 96 bit
#define DB_ESPNOW_AES_TAG_LEN 16  //
#define DB_ESPNOW_AES_KEY_LEN 256 // in bits 128 & 192 are supported by ESP32
#define DB_ESPNOW_PAYLOAD_MAXSIZE                                                                                           \
  (ESP_NOW_MAX_DATA_LEN - DB_ESPNOW_AES_IV_LEN - DB_ESPNOW_AES_TAG_LEN - 6 -                                                \
   1) // (origin, packet_type, seq_num) = 6 bytes, payload_length_decrypted = 1 byte

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0)

extern QueueHandle_t db_espnow_send_queue; // Queue that contains data to be sent via ESP-NOW
extern QueueHandle_t db_uart_write_queue;  // Queue that contains data to be written to UART (filled by ESP-NOW task)

/***************************************************************************************************************************
 * Public Structure Definitions
 **************************************************************************************************************************/

/***************************************************************************************************************************
 * @enum db_espnow_origin_t
 * @brief Defines the origin of the ESP-NOW packet.
 **************************************************************************************************************************/
enum db_espnow_origin_t {
  DB_ESPNOW_ORIGIN_GND = 0, /**< Packet sent by Ground Control Station (GCS) */
  DB_ESPNOW_ORIGIN_AIR = 1  /**< Packet sent by Drone */
};

/***************************************************************************************************************************
 * @struct db_esp_now_bpeer_info_t
 * @brief Information structure for broadcast peers communicating via ESP-NOW.
 **************************************************************************************************************************/
typedef struct {
  int8_t gnd_rssi;                              /**< RSSI (dBm) of the last received message from the peer. */
  uint16_t last_seq_num;                        /**< Last received sequence number. */
  uint16_t gnd_rx_lost_packets;                 /**< Number of lost packets detected on the ground station. */
  uint8_t broadcast_peer_mac[ESP_NOW_ETH_ALEN]; /**< MAC address of the broadcasting peer. */
} __attribute__((__packed__)) db_esp_now_bpeer_info_t;

/***************************************************************************************************************************
 * @struct db_esp_now_clients_list_t
 * @brief Stores RSSI data of multiple peers communicating via ESP-NOW.
 **************************************************************************************************************************/
typedef struct {
  uint8_t size;           /**< Number of peers in `db_esp_now_bpeer_info`. */
  int8_t gnd_noise_floor; /**< Noise floor measured at the ground station. */
  db_esp_now_bpeer_info_t db_esp_now_bpeer_info[DB_ESPNOW_MAX_BROADCAST_PEERS]; /**< Array of peer information. */
} __attribute__((__packed__)) db_esp_now_clients_list_t;

/***************************************************************************************************************************
 * @enum db_espnow_event_id_t
 * @brief Event types for ESP-NOW communication.
 **************************************************************************************************************************/
typedef enum {
  DB_ESPNOW_SEND_CB, /**< Event triggered when a message is sent. */
  DB_ESPNOW_RECV_CB  /**< Event triggered when a message is received. */
} db_espnow_event_id_t;

/***************************************************************************************************************************
 * @struct db_espnow_event_send_cb_t
 * @brief Information about ESP-NOW send events.
 **************************************************************************************************************************/
typedef struct {
  uint8_t mac_addr[ESP_NOW_ETH_ALEN]; /**< MAC address of the recipient. */
  esp_now_send_status_t status;       /**< Status of the send operation. */
} db_espnow_event_send_cb_t;

/***************************************************************************************************************************
 * @struct db_espnow_event_recv_cb_t
 * @brief Information about ESP-NOW receive events.
 **************************************************************************************************************************/
typedef struct {
  uint8_t mac_addr[ESP_NOW_ETH_ALEN]; /**< MAC address of the sender. */
  int8_t rssi;                        /**< RSSI of the received message. */
  uint8_t data_len;                   /**< Length of received data. */
  uint8_t *data;                      /**< Pointer to received data. */
} db_espnow_event_recv_cb_t;

/***************************************************************************************************************************
 * @union db_espnow_event_info_t
 * @brief Union to hold either send or receive event data.
 **************************************************************************************************************************/
typedef union {
  db_espnow_event_send_cb_t send_cb; /**< Send callback information. */
  db_espnow_event_recv_cb_t recv_cb; /**< Receive callback information. */
} db_espnow_event_info_t;

/***************************************************************************************************************************
 * @struct db_espnow_event_t
 * @brief ESP-NOW event structure.
 **************************************************************************************************************************/
typedef struct {
  db_espnow_event_id_t id;     /**< Event type. */
  db_espnow_event_info_t info; /**< Event details. */
} db_espnow_event_t;

/***************************************************************************************************************************
 * @enum db_espnow_queue_event_type_t
 * @brief Defines packet types in the ESP-NOW queue.
 **************************************************************************************************************************/
typedef enum {
  DB_ESP_NOW_PACKET_TYPE_DATA               = 0, /**< Data packet. */
  DB_ESP_NOW_PACKET_TYPE_INTERNAL_TELEMETRY = 1  /**< Internal telemetry packet. */
} db_espnow_queue_event_type_t;

/***************************************************************************************************************************
 * @struct db_espnow_queue_event_t
 * @brief Structure representing queued ESP-NOW events.
 **************************************************************************************************************************/
typedef struct {
  uint8_t data_len;                         /**< Length of the data. */
  uint8_t *data;                            /**< Pointer to the data. */
  db_espnow_queue_event_type_t packet_type; /**< Type of packet. */
} __attribute__((__packed__)) db_espnow_queue_event_t;

/***************************************************************************************************************************
 * @struct db_esp_now_packet_header_t
 * @brief Header structure for ESP-NOW packets.
 **************************************************************************************************************************/
typedef struct {
  uint8_t origin;                       /**< Packet origin (GCS or Drone). */
  uint8_t packet_type;                  /**< Type of packet. */
  uint32_t seq_num;                     /**< Sequence number of the packet. */
  uint8_t aes_iv[DB_ESPNOW_AES_IV_LEN]; /**< AES Initialization Vector. */
} __attribute__((__packed__)) db_esp_now_packet_header_t;

/***************************************************************************************************************************
 * @struct db_esp_now_packet_protected_data_t
 * @brief Structure for encrypted payload in ESP-NOW packets.
 **************************************************************************************************************************/
typedef struct {
  uint8_t payload_length_decrypted;           /**< Length of the decrypted payload. */
  uint8_t payload[DB_ESPNOW_PAYLOAD_MAXSIZE]; /**< Encrypted payload data. */
} __attribute__((__packed__)) db_esp_now_packet_protected_data_t;

/***************************************************************************************************************************
 * @struct db_esp_now_packet_t
 * @brief Complete ESP-NOW packet structure with encryption.
 **************************************************************************************************************************/
typedef struct {
  db_esp_now_packet_header_t db_esp_now_packet_header;                 /**< Packet header. */
  uint8_t tag[DB_ESPNOW_AES_TAG_LEN];                                  /**< AES-GCM authentication tag. */
  db_esp_now_packet_protected_data_t db_esp_now_packet_protected_data; /**< Encrypted payload. */
} __attribute__((__packed__)) db_esp_now_packet_t;

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/
/***************************************************************************************************************************
 * @brief Start task that handles ESP-NOW data
 **************************************************************************************************************************/
void db_start_espnow_module();

#endif // DB_ESP32_DB_ESP_NOW_H
