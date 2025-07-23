/******************************************************************************
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
 ******************************************************************************/

/******************************************************************************
 * Standard C Library Headers
 ******************************************************************************/
#include <stdint-gcc.h>
#include <string.h>
#include <sys/cdefs.h>
#include <sys/fcntl.h>
#include <sys/param.h>
#include <sys/types.h>

/******************************************************************************
 * ESP-IDF APIs
 ******************************************************************************/
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_vfs_dev.h"
#include "esp_wifi.h"

/******************************************************************************
 * ESP-IDF Networking (LWIP) APIs
 ******************************************************************************/
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

/******************************************************************************
 * ESP-IDF Driver APIs
 ******************************************************************************/
#include "driver/uart.h"

/******************************************************************************
 * Project Headers
 ******************************************************************************/
#include "db_esp32_control.h"
#include "db_esp_now.h"
#include "db_parameters.h"
#include "db_protocol.h"
#include "db_serial.h"
#include "globals.h"
#include "main.h"
#include "msp_ltm_serial.h"
#include "tcp_server.h"

#ifdef CONFIG_BT_ENABLED
#include "db_ble.h"
#endif

/******************************************************************************
 * MACROS
 ******************************************************************************/
#define TAG "DB_CONTROL"

/******************************************************************************
 * Private Variables
 ******************************************************************************/
static uint16_t app_port_proxy = APP_PORT_PROXY;

/******************************************************************************
 * Public Variables
 ******************************************************************************/
int8_t num_connected_tcp_clients = 0;

/******************************************************************************
 * Private Function Declaration
 ******************************************************************************/

/******************************************************************************
 * Opens non-blocking UDP socket used for WiFi to UART communication. Does also
 * accept broadcast packets just in case.
 * @return returns socket file descriptor
 ******************************************************************************/
static int open_serial_udp_socket();

/******************************************************************************
 * Opens non-blocking UDP socket used for WiFi to UART communication using WiFi
 * UDP broadcast packets e.g. by Skybrush.
 * @return returns socket file descriptor
 ******************************************************************************/
static int open_serial_udp_broadcast_socket();

/******************************************************************************
 * Opens non-blocking UDP socket used for internal DroneBridge telemetry.
 * Socket shall only be opened when local ESP32 is in client mode. Socket is
 * used to receive internal Wifi telemetry from the ESP32 access point we are
 * connected to.
 * @return returns socket file descriptor
 ******************************************************************************/
static int open_int_telemetry_udp_socket();

/******************************************************************************
 * Sends data to all clients that are part of the udp connection list. No
 * resending of packets in case of failure.
 * @param n_udp_conn_list List of known UDP clients/connections
 * @param data Buffer with the data to send
 * @param data_length Length of the data in the buffer
 ******************************************************************************/
static void send_to_all_udp_clients(udp_conn_list_t *n_udp_conn_list,
                                    const uint8_t *data, uint data_length);

/******************************************************************************
 * Adds a payload to be sent via ESP-NOW to the ESP-NOW queue (where the
 * esp-now task will pick it up, encrypt, package and finally send it over the
 * air)
 *
 * @param data Pointer to the payload buffer
 * @param data_length Length of the payload data. Must not be bigger than
 * DB_ESPNOW_PAYLOAD_MAXSIZE - fails otherwise
 ******************************************************************************/
static void send_to_all_espnow(uint8_t data[], const uint16_t *data_length);

/******************************************************************************
 * Check for incoming connections on TCP server
 *
 * @param tcp_master_socket Main open TCP socket to accept TCP
 * connections/clients
 * @param tcp_clients List of active TCP client connections (socket IDs)
 ******************************************************************************/
static void handle_tcp_master(const int tcp_master_socket, int tcp_clients[]);

/******************************************************************************
 * Reads serial (UART/USB/JTAG) transparently or parsing MAVLink/MSP/LTM
 * protocol. Then sends read data to all connected clients via TCP/UDP or
 * ESP-NOW. Non-Blocking function
 *
 * @param tcp_clients Array of connected TCP client sockets
 * @param transparent_buff_pos Counter variable for total read UART bytes
 * @param msp_ltm_buff_pos Pointer position/data length of destination-buffer
 * for read MSP messages
 * @param msp_message_buffer Destination-buffer for read MSP messages
 * @param serial_buffer Destination-buffer for the serial data
 * @param db_msp_ltm_port Pointer to structure containing MSP/LTM parser
 * information
 ******************************************************************************/
static void read_process_serial_link(int *tcp_clients,
                                     uint *transparent_buff_pos,
                                     uint *msp_ltm_buff_pos,
                                     uint8_t *msp_message_buffer,
                                     uint8_t *serial_buffer,
                                     msp_ltm_port_t *db_msp_ltm_port);

/******************************************************************************
 * Thread that manages all incoming and outgoing ESP-NOW and serial (UART)
 * connections. Called only when ESP-NOW mode is selected
 ******************************************************************************/
_Noreturn static void control_module_esp_now();

/******************************************************************************
 * Sends DroneBridge internal telemetry to tell every connected WiFi station
 * how well we receive their data (rssi). Uses UDP multicast message. Format:
 * [NUM_Entries - (MAC + RSSI) - (MAC + RSSI) - ...] Internal telemetry uses
 * DB_ESP32_INTERNAL_TELEMETRY_PORT port
 *
 * @param sta_list
 ******************************************************************************/
static void send_internal_telemetry_to_stations(int tel_sock,
                                                wifi_sta_list_t *sta_list,
                                                udp_conn_list_t *udp_conns);

/******************************************************************************
 * Receive and process internal telemetry (ESP32 AP to ESP32 Station) sent by
 * ESP32 LR access point. Matches with send_internal_telemetry_to_stations()
 * Sets station_rssi_ap based on the received value
 * Packet format: [NUM_Entries, (MAC + RSSI), (MAC + RSSI), (MAC + RSSI), ...]
 *
 * @param tel_sock Socket listening for internal telemetry
 ******************************************************************************/
static void handle_internal_telemetry(int tel_sock, uint8_t *udp_buffer,
                                      socklen_t *sock_len,
                                      struct sockaddr_in *udp_client);

/******************************************************************************
 * Thread that manages all incoming and outgoing TCP, UDP and serial (UART)
 * connections. Executed when Wi-Fi modes are set - ESP-NOW has its own thread
 ******************************************************************************/
_Noreturn static void control_module_udp_tcp();

#ifdef CONFIG_BT_ENABLED
/******************************************************************************
 * Bluetooth Threads
 ******************************************************************************/
_Noreturn static void control_module_ble();
#endif

/******************************************************************************
 * Private Function Definition
 ******************************************************************************/

static int
open_serial_udp_socket()
{
  struct sockaddr_in server_addr;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(APP_PORT_PROXY_UDP);
  int addr_family = AF_INET;
  int ip_protocol = IPPROTO_IP;
  char addr_str[128];
  inet_ntoa_r(server_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

  int udp_socket = socket(addr_family, SOCK_DGRAM, ip_protocol);
  if(udp_socket < 0) {
    ESP_LOGE(TAG,
             "Unable to create socket for serial communication: errno %d",
             errno);
    return -1;
  }
  int broadcastEnable = 1;
  int err = setsockopt(udp_socket,
                       SOL_SOCKET,
                       SO_BROADCAST,
                       &broadcastEnable,
                       sizeof(broadcastEnable));
  if(err < 0) {
    ESP_LOGE(
      TAG,
      "Socket unable to set udp socket to accept broadcast messages: errno %d",
      errno);
  }
  err = bind(udp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if(err < 0) {
    ESP_LOGE(
      TAG, "Socket unable to bind to %i errno %d", APP_PORT_PROXY_UDP, errno);
  }
  fcntl(udp_socket, F_SETFL, O_NONBLOCK);
  ESP_LOGI(TAG, "Opened UDP socket on port %i", APP_PORT_PROXY_UDP);
  return udp_socket;
}

#ifdef CONFIG_DB_SKYBRUSH_SUPPORT

static int
open_serial_udp_broadcast_socket()
{
  struct sockaddr_in server_addr;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(UDP_BROADCAST_PORT_SKYBRUSH);
  int addr_family = AF_INET;
  int ip_protocol = IPPROTO_IP;
  char addr_str[128];
  inet_ntoa_r(server_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

  int udp_socket = socket(addr_family, SOCK_DGRAM, ip_protocol);
  if(udp_socket < 0) {
    ESP_LOGE(TAG,
             "Unable to create socket for Skybrush communication: errno %d",
             errno);
    return -1;
  }
  int broadcastEnable = 1;
  int err = setsockopt(udp_socket,
                       SOL_SOCKET,
                       SO_BROADCAST,
                       &broadcastEnable,
                       sizeof(broadcastEnable));
  if(err < 0) {
    ESP_LOGE(TAG,
             "Socket unable to set Skybrush udp socket to accept broadcast "
             "messages: errno %d",
             errno);
  }
  err = bind(udp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if(err < 0) {
    ESP_LOGE(TAG,
             "Socket unable to bind Skybrush socket to %i errno %d",
             UDP_BROADCAST_PORT_SKYBRUSH,
             errno);
  }
  fcntl(udp_socket, F_SETFL, O_NONBLOCK);
  ESP_LOGI(TAG,
           "Opened UDP broadcast enabled socket on port %i",
           UDP_BROADCAST_PORT_SKYBRUSH);
  return udp_socket;
}

#endif

static int
open_int_telemetry_udp_socket()
{
  // Create a socket for sending to the multicast address
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if(sock < 0) {
    ESP_LOGE(TAG, "Failed to create socket. Error %d", errno);
    return -1;
  }
  struct sockaddr_in saddr = { 0 };
  // Bind the socket to any address
  saddr.sin_family = PF_INET;
  saddr.sin_port = htons(DB_ESP32_INTERNAL_TELEMETRY_PORT);
  saddr.sin_addr.s_addr = htonl(INADDR_ANY);
  esp_err_t err =
    bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
  if(err < 0) {
    ESP_LOGE(TAG, "Failed to bind socket. Error %d", errno);
    return -1;
  }

  // Set the time-to-live of messages to 1 so they do not go past the local
  // network
  int ttl = MULTICAST_TTL;
  if(setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0) {
    ESP_LOGE(TAG, "Failed to set IP_MULTICAST_TTL. Error %d", errno);
    close(sock);
    return -1;
  }

  fcntl(sock, F_SETFL, O_NONBLOCK);

  if(DB_PARAM_RADIO_MODE == DB_WIFI_MODE_STA) {
    // Configure for listening when in station mode
    struct ip_mreq imreq = { 0 };
    struct in_addr iaddr = { 0 };
    // Configure source interface
    imreq.imr_interface.s_addr = IPADDR_ANY;
    // Configure multicast address to listen to
    err = inet_aton(MULTICAST_IPV4_ADDR, &imreq.imr_multiaddr.s_addr);
    if(err != 1) {
      ESP_LOGE(TAG,
               "Configured IPV4 multicast address '%s' is invalid.",
               MULTICAST_IPV4_ADDR);
      // Errors in the return value have to be negative
      close(sock);
      return -1;
    }
    ESP_LOGI(TAG,
             "Configured internal Multicast address %s",
             inet_ntoa(imreq.imr_multiaddr.s_addr));
    if(!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
      ESP_LOGW(TAG,
               "Configured IPV4 multicast address '%s' is not a valid "
               "multicast address. This will probably not work.",
               MULTICAST_IPV4_ADDR);
    }

    err = setsockopt(
      sock, IPPROTO_IP, IP_MULTICAST_IF, &iaddr, sizeof(struct in_addr));
    if(err < 0) {
      ESP_LOGE(TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
      close(sock);
      return -1;
    }

    err = setsockopt(
      sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imreq, sizeof(struct ip_mreq));
    if(err < 0) {
      ESP_LOGE(TAG, "Failed to set IP_ADD_MEMBERSHIP. Error %d", errno);
      close(sock);
      return -1;
    }
  }
  ESP_LOGI(TAG,
           "Opened internal telemetry socket on port: %i",
           DB_ESP32_INTERNAL_TELEMETRY_PORT);
  return sock;
}

static void
send_to_all_udp_clients(udp_conn_list_t *n_udp_conn_list, const uint8_t *data,
                        uint data_length)
{
  for(int i = 0; i < n_udp_conn_list->size; i++) { // send to all UDP clients
    int sent =
      sendto(n_udp_conn_list->udp_socket,
             data,
             data_length,
             0,
             (struct sockaddr *)&n_udp_conn_list->db_udp_clients[i].udp_client,
             sizeof(n_udp_conn_list->db_udp_clients[i].udp_client));
    if(sent != data_length) {
      int err = errno;
      ESP_LOGE(TAG,
               "UDP - Error sending (%i/%i) to because of %d",
               sent,
               data_length,
               err);
    }
  }
}

static void
send_to_all_espnow(uint8_t data[], const uint16_t *data_length)
{
  db_espnow_queue_event_t evt;
  evt.data = malloc(*data_length);
  memcpy(evt.data, data, *data_length);
  evt.data_len = *data_length;
  evt.packet_type = DB_ESP_NOW_PACKET_TYPE_DATA;
  if(xQueueSend(db_espnow_send_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
    ESP_LOGW(TAG, "Send to db_espnow_send_queue queue fail");
    free(evt.data);
  }
  else {
    // all good
  }
}

static void
handle_tcp_master(const int tcp_master_socket, int tcp_clients[])
{
  struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
  uint32_t addr_len = sizeof(source_addr);
  int new_tcp_client =
    accept(tcp_master_socket, (struct sockaddr *)&source_addr, &addr_len);
  if(new_tcp_client > 0) {
    for(int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
      if(tcp_clients[i] <= 0) {
        tcp_clients[i] = new_tcp_client;
        fcntl(tcp_clients[i], F_SETFL, O_NONBLOCK);
        char addr_str[128];
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr,
                    addr_str,
                    sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "TCP: New client connected: %s", addr_str);
        num_connected_tcp_clients++;
        return;
      }
    }
    ESP_LOGI(TAG,
             "TCP: Could not accept connection. Too many clients connected.");
  }
}

static void
read_process_serial_link(int *tcp_clients, uint *transparent_buff_pos,
                         uint *msp_ltm_buff_pos, uint8_t *msp_message_buffer,
                         uint8_t *serial_buffer,
                         msp_ltm_port_t *db_msp_ltm_port)
{
  switch(DB_PARAM_SERIAL_PROTO) {
  case 0:
  case 2:
  case DB_SERIAL_PROTOCOL_MSPLTM:
    db_parse_msp_ltm(tcp_clients,
                     udp_conn_list,
                     msp_message_buffer,
                     msp_ltm_buff_pos,
                     db_msp_ltm_port);
    break;
  case 3:
  case DB_SERIAL_PROTOCOL_MAVLINK:
    db_read_serial_parse_mavlink(
      tcp_clients, udp_conn_list, msp_message_buffer, transparent_buff_pos);
    break;
  case DB_SERIAL_PROTOCOL_TRANSPARENT:
  default:
    db_read_serial_parse_transparent(
      tcp_clients, udp_conn_list, serial_buffer, transparent_buff_pos);
    break;
  }
}

_Noreturn static void
control_module_esp_now()
{
  ESP_LOGI(TAG, "Starting control module (ESP-NOW)");
  esp_err_t serial_socket = ESP_FAIL;
  // open serial socket for comms with FC or GCS
  serial_socket = open_serial_socket();
  if(serial_socket == ESP_FAIL) {
    ESP_LOGE(TAG, "UART socket not opened. Aborting start of control module.");
    vTaskDelete(NULL);
  }
  else {
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
    db_jtag_serial_info_print();
#endif
  }

  uint transparent_buff_pos = 0;
  uint msp_ltm_buff_pos = 0;
  uint8_t msp_message_buffer[UART_BUF_SIZE];
  uint8_t serial_buffer[DB_PARAM_SERIAL_PACK_SIZE];
  msp_ltm_port_t db_msp_ltm_port;
  db_espnow_queue_event_t db_espnow_uart_evt;
  uint delay_timer_cnt = 0;

  ESP_LOGI(TAG, "Started control module (ESP-NOW)");
  while(1) {
    // read UART (and split into packets & process MAVLink if desired); send to
    // ESP-NOW queue to be processed by esp-now task
    read_process_serial_link(NULL,
                             &transparent_buff_pos,
                             &msp_ltm_buff_pos,
                             msp_message_buffer,
                             serial_buffer,
                             &db_msp_ltm_port);
    // read queue that was filled by esp-now task to check for data that needs
    // to be sent via serial link
    if(db_uart_write_queue != NULL &&
       xQueueReceive(db_uart_write_queue, &db_espnow_uart_evt, 0) == pdTRUE) {
      switch(DB_PARAM_SERIAL_PROTO) {
      case DB_SERIAL_PROTOCOL_MAVLINK:
        // Parse, so we can listen in and react to certain messages - function
        // will send parsed messages to serial link. We can not write to serial
        // first since we might inject packets and do not know when to do so to
        // not "destroy" an existing packet
        db_parse_mavlink_from_radio(
          NULL, NULL, db_espnow_uart_evt.data, db_espnow_uart_evt.data_len);
        break;

      default:
        // No parsing with any other protocol - transparent here - just pass
        // through
        db_write_to_serial(db_espnow_uart_evt.data,
                           db_espnow_uart_evt.data_len);
        break;
      }

      free(db_espnow_uart_evt.data);
    }
    else {
      if(db_uart_write_queue == NULL)
        ESP_LOGE(TAG, "db_uart_write_queue is NULL!");
      // no new data available to be sent via serial link do nothing
    }
    if(delay_timer_cnt == 5000) {
      /* all actions are non-blocking so allow some delay so that the IDLE task
      of FreeRTOS and the watchdog can run read:
      https://esp32developer.com/programming-in-c-c/tasks/tasks-vs-co-routines
      for reference */
      vTaskDelay(10 / portTICK_PERIOD_MS);
      delay_timer_cnt = 0;
    }
    else {
      delay_timer_cnt++;
    }
  }
  vTaskDelete(NULL);
}

static void
send_internal_telemetry_to_stations(int tel_sock, wifi_sta_list_t *sta_list,
                                    udp_conn_list_t *udp_conns)
{
  if(DB_PARAM_RADIO_MODE == DB_WIFI_MODE_AP_LR && udp_conns->size > 0 &&
     sta_list->num > 0) {
    char addr_buf[32] = { 0 };
    struct addrinfo hints = {
      .ai_flags = AI_PASSIVE,
      .ai_socktype = SOCK_DGRAM,
    };
    struct addrinfo *res;

    // Send an IPv4 multicast packet
    hints.ai_family = AF_INET; // For an IPv4 socket
    int err = getaddrinfo(MULTICAST_IPV4_ADDR, NULL, &hints, &res);
    if(err < 0) {
      ESP_LOGE(TAG,
               "getaddrinfo() failed for IPV4 destination address. error: %d",
               err);
      return;
    }
    if(res == 0) {
      ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
      return;
    }
    ((struct sockaddr_in *)res->ai_addr)->sin_port =
      htons(DB_ESP32_INTERNAL_TELEMETRY_PORT);
    inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr,
                addr_buf,
                sizeof(addr_buf) - 1);
    static uint8_t buffer[1280] = { 0 };
    uint16_t buffer_pos =
      1; // we start at 1 since we want to put the count in position 0
    uint8_t already_sent = 0;
    for(int i = 0; i < sta_list->num; i++) {
      if((buffer_pos + 7) < 1280) {
        memcpy(&buffer[buffer_pos], sta_list->sta[i].mac, 6);
        buffer_pos += 6;
        buffer[buffer_pos] = sta_list->sta[i].rssi;
        buffer_pos++;
      }
      else {
        // packet would get too long. Sent this chunk already
        buffer[0] = (uint8_t)
          i; // first byte shall be the number of entries in the packet
        sendto(tel_sock, buffer, buffer_pos, 0, res->ai_addr, res->ai_addrlen);
        already_sent += i;
        buffer_pos = 1;
      }
    }
    buffer[0] = (uint8_t)sta_list->num - already_sent;
    err = sendto(
      tel_sock, buffer, buffer_pos + 1, 0, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if(err < 0) {
      ESP_LOGE(TAG, "Internal telemetry sendto failed. errno: %d", errno);
      return;
    }
  }
  else {
    // in other modes we cannot do that. ESP-NOW uses a different function and
    // way of telling
  }
}

static void
handle_internal_telemetry(int tel_sock, uint8_t *udp_buffer,
                          socklen_t *sock_len, struct sockaddr_in *udp_client)
{
  if(tel_sock > 0) {
    ssize_t recv_length = recvfrom(tel_sock,
                                   udp_buffer,
                                   UDP_BUF_SIZE,
                                   0,
                                   (struct sockaddr *)udp_client,
                                   sock_len);
    if(recv_length > 0) {
      ESP_LOGD(
        TAG, "Got internal telem. frame containing %i entries", udp_buffer[0]);
      for(int i = 1; i < (udp_buffer[0] * 7); i += 7) {
        if(memcmp(LOCAL_MAC_ADDRESS, &udp_buffer[i], ESP_NOW_ETH_ALEN) == 0) {
          // found us in the list (this local ESP32 AIR unit) -> update
          // internal telemetry buffer, so it gets sent with next Mavlink RADIO
          // STATUS in case MAVLink radio status is enabled
          db_esp_signal_quality.gnd_rssi = (int8_t)udp_buffer[i + 6];
          ESP_LOGD(TAG,
                   "AP receives our packets with RSSI: %i",
                   db_esp_signal_quality.gnd_rssi);
          break;
        }
        else {
          // keep on looking for our MAC
        }
      }
    }
    else { /* received nothing - socket is non-blocking */
    }
  }
  else { /* socket failed to init or was never inited */
  }
}

_Noreturn static void
control_module_udp_tcp()
{
  ESP_LOGI(TAG, "Starting control module (Wi-Fi)");
  esp_err_t serial_socket_status = open_serial_socket();
  if(serial_socket_status == ESP_FAIL) {
    ESP_LOGE(
      TAG, "JTAG serial socket not opened. Aborting start of control module.");
    vTaskDelete(NULL);
  }
  else {
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
    db_jtag_serial_info_print();
#endif
  }

  int tcp_master_socket = open_tcp_server(app_port_proxy);
  if(tcp_master_socket == ESP_FAIL) {
    ESP_LOGE(
      TAG,
      "TCP master socket failed to open. Aborting start of control module.");
    vTaskDelete(NULL);
  }
  fcntl(tcp_master_socket, F_SETFL, O_NONBLOCK);
  int tcp_clients[CONFIG_LWIP_MAX_ACTIVE_TCP];
  for(int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
    tcp_clients[i] = -1;
  }

  udp_conn_list->udp_socket = open_serial_udp_socket();
#ifdef CONFIG_DB_SKYBRUSH_SUPPORT
  int udp_broadcast_skybrush_socket = -1;
  if(DB_PARAM_RADIO_MODE == DB_WIFI_MODE_STA) {
    udp_broadcast_skybrush_socket = open_serial_udp_broadcast_socket();
  }
  else {
    // we do only support Skybrush Wi-Fi and the broadcast port when in Wi-Fi
    // client mode
  }
#endif
  int db_internal_telem_udp_sock = -1;
  switch(DB_PARAM_RADIO_MODE) {
  case DB_WIFI_MODE_AP_LR:
  case DB_WIFI_MODE_STA:
    db_internal_telem_udp_sock = open_int_telemetry_udp_socket();
    break;

  default:
    // Other Wi-Fi modes do not need this. Only Wi-Fi stations will receive if
    // connected to LR access point. ESP-NOW uses different sockets/systems
    break;
  }

  uint8_t udp_buffer[UDP_BUF_SIZE];
  struct db_udp_client_t new_db_udp_client = { 0 };
  socklen_t udp_socklen = sizeof(new_db_udp_client.udp_client);

  uint transparent_buff_pos = 0;
  uint msp_ltm_buff_pos = 0;
  uint8_t tcp_client_buffer[TCP_BUFF_SIZ];
  memset(tcp_client_buffer, 0, TCP_BUFF_SIZ);
  uint8_t msp_message_buffer[UART_BUF_SIZE];
  uint8_t serial_buffer[DB_PARAM_SERIAL_PACK_SIZE];
  msp_ltm_port_t db_msp_ltm_port;
  int delay_timer_cnt = 0;

  ESP_LOGI(TAG, "Started control module (Wi-Fi)");
  while(1) {
    // Read incoming wireless data (Wi-Fi)
    // Wi-Fi based modes that use TCP and UDP communication
    handle_tcp_master(tcp_master_socket, tcp_clients);
    for(int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) { // handle TCP clients
      if(tcp_clients[i] > 0) {
        ssize_t recv_length =
          recv(tcp_clients[i], tcp_client_buffer, TCP_BUFF_SIZ, 0);
        if(recv_length > 0) {
          switch(DB_PARAM_SERIAL_PROTO) {
          case DB_SERIAL_PROTOCOL_MAVLINK:
            // Parse, so we can listen in and react to certain messages -
            // function will send parsed messages to serial link. We can not
            // write to serial first since we might inject packets and do not
            // know when to do so to not "destroy" an existing packet
            db_parse_mavlink_from_radio(
              tcp_clients, udp_conn_list, tcp_client_buffer, recv_length);
            break;

          default:
            // No parsing with any other protocol - transparent here
            db_write_to_serial(tcp_client_buffer, recv_length);
            break;
          }
        }
        else if(recv_length == 0) {
          shutdown(tcp_clients[i], 0);
          close(tcp_clients[i]);
          tcp_clients[i] = -1;
          ESP_LOGI(TAG, "TCP client disconnected");
          num_connected_tcp_clients--;
        }
        else if(errno != EAGAIN && errno != EWOULDBLOCK) {
          ESP_LOGE(TAG,
                   "Error receiving from TCP client %i (fd: %i): %d",
                   i,
                   tcp_clients[i],
                   errno);
          shutdown(tcp_clients[i], 0);
          close(tcp_clients[i]);
          num_connected_tcp_clients--;
          tcp_clients[i] = -1;
        }
      }
    }
    // handle incoming UDP data on main port 14550 - Read UDP and forward to
    // UART
    ssize_t recv_length =
      recvfrom(udp_conn_list->udp_socket,
               udp_buffer,
               UDP_BUF_SIZE,
               0,
               (struct sockaddr *)&new_db_udp_client.udp_client,
               &udp_socklen);
    if(recv_length > 0) {
      switch(DB_PARAM_SERIAL_PROTO) {
      case DB_SERIAL_PROTOCOL_MAVLINK:
        // Parse, so we can listen in and react to certain messages - function
        // will send parsed messages to serial link. We can not write to serial
        // first since we might inject packets and do not know when to do so to
        // not "destroy" an existing packet
        db_parse_mavlink_from_radio(
          tcp_clients, udp_conn_list, udp_buffer, recv_length);
        break;

      default:
        // No parsing with any other protocol - transparent here
        db_write_to_serial(udp_buffer, recv_length);
        break;
      }

      // All devices that send us UDP data will be added to the list of UDP
      // receivers Allows to register new app on different port. Used e.g. for
      // UDP conn setup in sta-mode. Devices/Ports added this way cannot be
      // removed in sta-mode since UDP is connectionless, and we cannot
      // determine if the client is still existing. This will blow up the list
      // of connected devices. In AP-Mode the devices can be removed based on
      // the IP/MAC address
      db_add_to_known_udp_clients(udp_conn_list, new_db_udp_client, false);
    }

#ifdef CONFIG_DB_SKYBRUSH_SUPPORT
    if(DB_PARAM_RADIO_MODE == DB_WIFI_MODE_STA &&
       udp_broadcast_skybrush_socket != -1) {
      // This is special support for Skybrush. Skybrush sends some UDP
      // broadcast msgs to 14555 in addition to regular msgs to 14550 We only
      // read and directly forward here. No parsing and no adding to known UDP
      // clients
      recv_length = recvfrom(udp_broadcast_skybrush_socket,
                             udp_buffer,
                             UDP_BUF_SIZE,
                             0,
                             (struct sockaddr *)&new_db_udp_client.udp_client,
                             &udp_socklen);
      if(recv_length > 0) {
        // no parsing - transparent here
        db_write_to_serial(udp_buffer, recv_length);
        // add Skybrush server to known UDP target/distribution list
        db_add_to_known_udp_clients(udp_conn_list, new_db_udp_client, false);
      }
    }
#endif

    if(DB_PARAM_RADIO_MODE == DB_WIFI_MODE_STA) {
      handle_internal_telemetry(db_internal_telem_udp_sock,
                                udp_buffer,
                                &udp_socklen,
                                &new_db_udp_client.udp_client);
    }
    else {
      // internal telemetry only received when in STA mode. Coming from the
      // ESP32 AP. Nothing to do here
    }

    // Second check for incoming UART data and send it to TCP/UDP
    read_process_serial_link(tcp_clients,
                             &transparent_buff_pos,
                             &msp_ltm_buff_pos,
                             msp_message_buffer,
                             serial_buffer,
                             &db_msp_ltm_port);

    if(delay_timer_cnt == 6000) {
      // all actions are non-blocking so allow some delay so that the IDLE task
      // of FreeRTOS and the watchdog can run read:
      // https://esp32developer.com/programming-in-c-c/tasks/tasks-vs-co-routines
      // for reference
      vTaskDelay(10 / portTICK_PERIOD_MS);
      delay_timer_cnt = 0;
      // Use the opportunity to get some regular status information like rssi
      // and send them via internal telemetry
      if(DB_PARAM_RADIO_MODE == DB_WIFI_MODE_STA) {
        // update rssi variable - set to -127 when not available
        if(esp_wifi_sta_get_rssi((int *)&db_esp_signal_quality.air_rssi) !=
           ESP_OK) {
          db_esp_signal_quality.air_rssi = -127;
        }
        else { /* all good */
        }
      }
      else if(!DB_RADIO_IS_OFF &&
              (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_AP ||
               DB_PARAM_RADIO_MODE == DB_WIFI_MODE_AP_LR)) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_ap_get_sta_list(
          &wifi_sta_list)); // update list of connected stations
        send_internal_telemetry_to_stations(
          db_internal_telem_udp_sock, &wifi_sta_list, udp_conn_list);
      }
      else {
        // no way of getting RSSI here. Do nothing
      }
      //            size_t free_dram =
      //            heap_caps_get_free_size(MALLOC_CAP_8BIT); size_t
      //            lowmark_dram =
      //            heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
      //            ESP_LOGI(TAG, "Free heap: %i, low mark: %i", free_dram,
      //            lowmark_dram);
    }
    else {
      delay_timer_cnt++;
    }
  }
  vTaskDelete(NULL);
}

#ifdef CONFIG_BT_ENABLED

_Noreturn static void
control_module_ble()
{
  ESP_LOGI(TAG, "Starting control module (Bluetooth)");

  /* Initialize error code as failed, because UART is not initialized*/
  esp_err_t serial_socket = ESP_FAIL;

  /* open serial socket for comms with FC or GCS */
  serial_socket = open_serial_socket();
  if(serial_socket == ESP_FAIL) {
    ESP_LOGE(TAG, "UART socket not opened. Aborting start of control module.");
    vTaskDelete(NULL);
  }
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
  else {
    db_jtag_serial_info_print();
  }
#endif

  uint8_t msp_message_buffer[UART_BUF_SIZE];
  uint8_t serial_buffer[DB_PARAM_SERIAL_PACK_SIZE];
  msp_ltm_port_t db_msp_ltm_port;
  db_ble_queue_event_t bleData;
  uint transparent_buff_pos = 0;
  uint msp_ltm_buff_pos = 0;
  uint delay_timer_cnt = 0;

  /* Event Loop */
  while(1) {
    /* Read UART and send data to BLE */
    read_process_serial_link(
      NULL,                  // NULL, not using TCP
      &transparent_buff_pos, // transparent buffer position
      &msp_ltm_buff_pos,     // msp buffer position
      msp_message_buffer,    // msp buffer
      serial_buffer,         // serial buffer
      &db_msp_ltm_port       // msp port
    );

    if(db_uart_write_queue_ble != NULL &&
       xQueueReceive(db_uart_write_queue_ble, &bleData, 0) == pdTRUE) {
      if(DB_PARAM_SERIAL_PROTO == DB_SERIAL_PROTOCOL_MAVLINK) {
        // Parse, so we can listen in and react to certain messages - function
        // will send parsed messages to serial link. We can not write to serial
        // first since we might inject packets and do not know when to do so to
        // not "destroy" an existing packet
        db_parse_mavlink_from_radio(
          NULL, NULL, bleData.data, bleData.data_len);
      }
      else {
        // no parsing with any other protocol - transparent here - just pass
        // through
        db_write_to_serial(bleData.data, bleData.data_len);
      }
      free(bleData.data);
    }
    else {
      if(db_uart_write_queue_ble == NULL)
        ESP_LOGE(TAG, "db_uart_write_queue is NULL!");
      // no new data available to be sent via serial link do nothing
    }

    /**Yield to the scheduler if delay_timer_cnt reaches 5000,allowing other
     * tasks to execute and preventing starvation.
     */
    if(delay_timer_cnt == 5000) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      delay_timer_cnt = 0;
    }
    else {
      delay_timer_cnt++;
    }
  }
  vTaskDelete(NULL);
}

#endif

/******************************************************************************
 * Public Function Definition
 ******************************************************************************/

bool
db_add_to_known_udp_clients(udp_conn_list_t *n_udp_conn_list,
                            struct db_udp_client_t new_db_udp_client,
                            bool save_to_nvm)
{
  if(n_udp_conn_list == NULL) { // Check if the list is NULL
    return false;               // Do nothing
  }
  if(n_udp_conn_list->size == MAX_UDP_CLIENTS) { // Check if the list is full
    return false;                                // Do nothing
  }
  for(int i = 0; i < n_udp_conn_list->size; i++) {
    if((n_udp_conn_list->db_udp_clients[i].udp_client.sin_port ==
        new_db_udp_client.udp_client.sin_port) &&
       (n_udp_conn_list->db_udp_clients[i].udp_client.sin_addr.s_addr ==
        new_db_udp_client.udp_client.sin_addr.s_addr)) {
      return false; // client existing - do not add
    }
  }
  n_udp_conn_list->db_udp_clients[n_udp_conn_list->size] =
    new_db_udp_client;     // Copy the element data to the end of the array
  n_udp_conn_list->size++; // Increment the size of the list
  // some logging
  char ip_port_string[INET_ADDRSTRLEN + 10];
  char ip_string[INET_ADDRSTRLEN];
  inet_ntop(AF_INET,
            &(new_db_udp_client.udp_client.sin_addr),
            ip_string,
            INET_ADDRSTRLEN);
  sprintf(ip_port_string,
          "%s:%d",
          ip_string,
          htons(new_db_udp_client.udp_client.sin_port));
  ESP_LOGI(TAG,
           "Added %s to udp client distribution list - save to NVM: %i",
           ip_port_string,
           save_to_nvm);
  // save to memory
  if(save_to_nvm) {
    db_save_udp_client_to_nvm(&new_db_udp_client, false);
  }
  else {
    // do not save to NVM
  }
  return true;
}

udp_conn_list_t *
db_udp_client_list_create()
{
  udp_conn_list_t *n_udp_conn_list =
    malloc(sizeof(udp_conn_list_t)); // Allocate memory for the list
  if(n_udp_conn_list == NULL) {      // Check if the allocation failed
    return NULL;                     // Return NULL to indicate an error
  }
  n_udp_conn_list->size = 0; // Initialize the size to 0
  return n_udp_conn_list;    // Return the pointer to the list
}

void
db_udp_client_list_destroy(udp_conn_list_t *n_udp_conn_list)
{
  if(n_udp_conn_list == NULL) { // Check if the list is NULL
    return;                     // Do nothing
  }
  free(n_udp_conn_list); // Free the list
}

void
db_send_to_all_clients(int tcp_clients[], udp_conn_list_t *n_udp_conn_list,
                       uint8_t data[], uint16_t data_length)
{
  db_ble_queue_event_t bleData;
  switch(DB_PARAM_RADIO_MODE) {
  case DB_WIFI_MODE_ESPNOW_AIR:
  case DB_WIFI_MODE_ESPNOW_GND:
    // ESP-NOW mode
    if(data_length > DB_ESPNOW_PAYLOAD_MAXSIZE) {
      // Data not properly sized, split into multiple packets
      uint16_t sent_bytes = 0;
      uint16_t next_chunk_len = 0;
      do {
        next_chunk_len = data_length - sent_bytes;
        if(next_chunk_len > DB_ESPNOW_PAYLOAD_MAXSIZE) {
          next_chunk_len = DB_ESPNOW_PAYLOAD_MAXSIZE;
        }
        send_to_all_espnow(&data[sent_bytes], &next_chunk_len);
        sent_bytes += next_chunk_len;
      } while(sent_bytes < data_length);
    }
    else {
      // Packet is properly sized - send to ESP-NOW outbound queue
      send_to_all_espnow(data, &data_length);
    }
    break;

  case DB_BLUETOOTH_MODE:
#ifdef CONFIG_BT_ENABLED
    bleData.data = malloc(data_length);
    bleData.data_len = data_length;
    memcpy(bleData.data, data, bleData.data_len);
    if(xQueueSend(db_uart_read_queue_ble, &bleData, portMAX_DELAY) != pdPASS) {
      ESP_LOGE(TAG, "Failed to send BLE data to queue");
      free(bleData.data);
    }
#endif
    break;

  default:
    // Other modes (WiFi Modes using TCP/UDP)
    db_send_to_all_tcp_clients(tcp_clients, data, data_length);
    send_to_all_udp_clients(n_udp_conn_list, data, data_length);
    break;
  }
}

bool
db_remove_from_known_udp_clients(udp_conn_list_t *n_udp_conn_list,
                                 struct db_udp_client_t new_db_udp_client)
{
  if(n_udp_conn_list == NULL) { // Check if the list is NULL
    return false;               // Do nothing
  }
  for(int i = 0; i < n_udp_conn_list->size; i++) { // Loop through the array
    if(memcmp(n_udp_conn_list->db_udp_clients[i].mac,
              new_db_udp_client.mac,
              sizeof(n_udp_conn_list->db_udp_clients[i].mac)) ==
       0) { // Compare the current array element with the element
      // Found a match
      for(int j = i; j < n_udp_conn_list->size - 1;
          j++) { // Loop from the current index to the end of the array
        n_udp_conn_list->db_udp_clients[j] =
          n_udp_conn_list
            ->db_udp_clients[j + 1]; // Shift the array elements to the left
      }
      n_udp_conn_list->size--; // Decrement the size of the list
      return true;             // Exit the function
    }
  }
  // No match found
  return false;
}

void
db_start_control_module()
{
  switch(DB_PARAM_RADIO_MODE) {
  case DB_WIFI_MODE_ESPNOW_GND:
  case DB_WIFI_MODE_ESPNOW_AIR:
    xTaskCreate(
      &control_module_esp_now, /**< Task function for ESP-NOW communication */
      "control_espnow",        /**< Task name (for debugging) */
      40960,                   /**< Stack size (in bytes) */
      NULL,                    /**< Task parameters (unused) */
      5,                       /**< Task priority */
      NULL                     /**< Task handle (unused) */
    );
    break;

  case DB_BLUETOOTH_MODE:
#ifdef CONFIG_BT_ENABLED
    xTaskCreate(&control_module_ble, /**< Task function for Bluetooth BLE
                                        communication */
                "control_bluetooth", /**< Task name (for debugging) */
                40960,               /**< Stack size (in bytes) */
                NULL,                /**< Task parameters (unused) */
                5,                   /**< Task priority */
                NULL                 /**< Task handle (unused) */
    );
#else
    ESP_LOGE(TAG,
             "Bluetooth is not enabled. Aborting start of control module.");
#endif
    break;

  default:
    xTaskCreate(
      &control_module_udp_tcp, /**< Task function for UDP/TCP communication */
      "control_wifi",          /**< Task name (for debugging) */
      46080,                   /**< Stack size (in bytes) */
      NULL,                    /**< Task parameters (unused) */
      5,                       /**< Task priority */
      NULL                     /**< Task handle (unused) */
    );
    break;
  }
}