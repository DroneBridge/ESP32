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

#ifndef DB_ESP32_DB_ESP32_CONTROL_H
#define DB_ESP32_DB_ESP32_CONTROL_H

#include <lwip/sockets.h>

#define MULTICAST_IPV4_ADDR "232.10.11.12" // "224.0.0.1"
#define MULTICAST_TTL       1
#define MAX_UDP_CLIENTS     8
#define TRANS_RD_BYTES_NUM  8 // amount of bytes read form serial port at once when transparent is selected
#define UDP_BUF_SIZE        2048
#define UART_BUF_SIZE       (1024)

// per client structure of connected devices in softAP mode
struct db_udp_client_t {
  uint8_t mac[6];                /**< MAC address of the connected client */
  struct sockaddr_in udp_client; /**< Socket address (IP & PORT) of the client */
};

typedef struct udp_conn_list_s {
  struct db_udp_client_t db_udp_clients[MAX_UDP_CLIENTS]; /**< Array of UDP clients */
  int size;                                               /**< Number of clients in the list */
  int udp_socket;                                         /**< UDP socket file descriptor */
} udp_conn_list_t;

// Used on the ESP AIR side to keep track and used to fill MAVlink RADIO STATUS msg
typedef struct {
  int8_t air_rssi;              /**< RSSI of received data from AP (station mode) */
  int8_t gnd_rssi;              /**< RSSI reported by the ground station */
  int8_t air_noise_floor;       /**< Noise floor on the air side (ESP-NOW mode) */
  int8_t gnd_noise_floor;       /**< Noise floor reported by the ground station */
  uint16_t gnd_rx_packets_lost; /**< Number of ESP-NOW packets lost by the ground station */
} db_esp_signal_quality_t;

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/

/***************************************************************************************************************************
 * @brief DroneBridge control module implementation for a ESP32 device. Bidirectional link between FC and ground. Can
 * handle MSPv1, MSPv2, LTM and MAVLink.
 * MSP & LTM is parsed and sent packet/frame by frame to ground
 * MAVLink is passed through (fully transparent). Can be used with any protocol.
 **************************************************************************************************************************/
void db_start_control_module();

/***************************************************************************************************************************
 * @brief Init/Create structure containing all UDP connection information
 * @return Structure containing all UDP connection information
 **************************************************************************************************************************/
udp_conn_list_t *udp_client_list_create();

/***************************************************************************************************************************
 *  Destroy structure containing all UDP connection information
 * @param n_udp_conn_list Structure containing all UDP connection information
 **************************************************************************************************************************/
void udp_client_list_destroy(udp_conn_list_t *n_udp_conn_list);

/***************************************************************************************************************************
 * Add a new UDP client to the list of known UDP clients. Checks if client is already known based on IP and port.
 * Added client will receive UDP packets with serial info and will be able to send UDP packets to the serial interface
 * of the ESP32.
 * PORT, MAC & IP should be set inside new_db_udp_client. If MAC is not set then the device cannot be removed later on.
 *
 * @param n_udp_conn_list Structure containing all UDP connection information
 * @param new_db_udp_client New client to add to the UDP list. PORT, MAC & IP must be set. If MAC is not set then the
 *                          device cannot be automatically removed later on. To remove it, the user must clear the entire
 *list.
 * @param save_to_nvm Set to 1 (true) in case you want the UDP client to survive the reboot. Set to 0 (false) if client is
 *temporary for this session. It will then be saved to NVM and added to the udp_conn_list_t on startup. Only one client can
 *be saved to NVM.
 * @return 1 if added - 0 if not
 **************************************************************************************************************************/
bool add_to_known_udp_clients(udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client, bool save_to_nvm);

/***************************************************************************************************************************
 * Main call for sending anything over the air.
 * Send to all connected TCP & UDP clients or broadcast via ESP-NOW depending on the mode (DB_WIFI_MODE) we are currently in.
 * Typically called by a function that read from UART.
 *
 * When in ESP-NOW mode the packets will be split if they are bigger than DB_ESPNOW_PAYLOAD_MAXSIZE.
 *
 * @param tcp_clients Array of socket IDs for the TCP clients
 * @param udp_conn Structure handling the UDP connection
 * @param data payload to send
 * @param data_length Length of payload to send
 **************************************************************************************************************************/
void db_send_to_all_clients(int tcp_clients[], udp_conn_list_t *n_udp_conn_list, uint8_t data[], uint16_t data_length);

/***************************************************************************************************************************
 * Remove a client from the sending list. Client will no longer receive UDP packets. MAC address must be given.
 * Usually called in AP-Mode when a station disconnects. In any other case we will not know since UDP is a connection-less
 * protocol
 *
 * @param n_udp_conn_list Structure containing all UDP connection information
 * @param new_db_udp_client The UDP client to remove based on its MAC address
 * @return true if removed - false if nothing was removed
 **************************************************************************************************************************/
bool remove_from_known_udp_clients(udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client);

#endif // DB_ESP32_DB_ESP32_CONTROL_H
