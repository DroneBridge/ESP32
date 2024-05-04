#include <sys/cdefs.h>
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
#include <sys/cdefs.h>
#include <sys/fcntl.h>
#include <sys/param.h>
#include <string.h>
#include <esp_task_wdt.h>
#include <esp_vfs_dev.h>
#include <lwip/inet.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <stdint-gcc.h>
#include <sys/types.h>
#include "esp_log.h"
#include "lwip/sockets.h"
#include "driver/uart.h"
#include "globals.h"
#include "msp_ltm_serial.h"
#include "db_protocol.h"
#include "tcp_server.h"
#include "db_esp32_control.h"
#include "main.h"
#include "db_serial.h"
#include "db_esp_now.h"

#define TAG "DB_CONTROL"

uint16_t app_port_proxy = APP_PORT_PROXY;

int8_t num_connected_tcp_clients = 0;

/**
 * Opens non-blocking UDP socket used for UART to serial communication. Socket can also accept broadcast messages
 * @return returns socket file descriptor
 */
int db_open_serial_udp_socket() {
    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(APP_PORT_PROXY_UDP);
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    char addr_str[128];
    inet_ntoa_r(server_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int udp_socket = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket for serial communication: errno %d", errno);
        return -1;
    }
    int broadcastEnable=1;
    int err = setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to set socket to accept broadcast messages: errno %d", errno);
    }
    err = bind(udp_socket, (struct sockaddr *) &server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind to %i errno %d", APP_PORT_PROXY_UDP, errno);
    }
    fcntl(udp_socket, F_SETFL, O_NONBLOCK);
    ESP_LOGI(TAG, "Opened UDP socket on port %i", APP_PORT_PROXY_UDP);
    return udp_socket;
}

/**
 * Opens non-blocking UDP socket used for internal DroneBridge telemetry. Socket shall only be opened when local ESP32
 * is in client mode. Socket is used to receive internal Wifi telemetry from the ESP32 access point we are connected to.
 * @return returns socket file descriptor
 */
int db_open_int_telemetry_udp_socket() {
    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(DB_ESP32_INTERNAL_TELEMETRY_PORT);
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    char addr_str[128];
    inet_ntoa_r(server_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int udp_socket = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket for internal telemetry: errno %d", errno);
        return -1;
    }

    int err = bind(udp_socket, (struct sockaddr *) &server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    fcntl(udp_socket, F_SETFL, O_NONBLOCK);
    ESP_LOGI(TAG, "Opened UDP socket on port %i", DB_ESP32_INTERNAL_TELEMETRY_PORT);
    return udp_socket;
}

void send_to_all_udp_clients(udp_conn_list_t *n_udp_conn_list, const uint8_t *data, uint data_length) {
    for (int i = 0; i < n_udp_conn_list->size; i++) {  // send to all UDP clients
        resend:;
        int sent = sendto(n_udp_conn_list->udp_socket, data, data_length, 0,
                          (struct sockaddr *) &n_udp_conn_list->db_udp_clients[i].udp_client,
                          sizeof(n_udp_conn_list->db_udp_clients[i].udp_client));
        if (sent != data_length) {
            ESP_LOGE(TAG, "UDP - Error sending (%i/%i) because of %d - trying again!", sent, data_length, errno);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            goto resend;
        }
    }
}

/**
 * Send to all connected TCP & UDP clients. Typically called by a function that read from UART.
 *
 * @param tcp_clients Array of socket IDs for the TCP clients
 * @param udp_conn Structure handling the UDP connection
 * @param data payload to send
 * @param data_length Length of payload to send
 */
void send_to_all_clients(int tcp_clients[], udp_conn_list_t *n_udp_conn_list, uint8_t data[], uint data_length) {
    if (DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_AIR && DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_GND) {
        send_to_all_tcp_clients(tcp_clients, data, data_length);
        send_to_all_udp_clients(n_udp_conn_list, data, data_length);
    } else {
        // ESP-NOW mode
        db_espnow_uart_event_t evt;
        evt.data = malloc(data_length);
        memcpy(evt.data, data, data_length);
        evt.data_len = data_length;
        if (xQueueSend(db_espnow_send_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
            ESP_LOGW(TAG, "Send to db_espnow_send_queue queue fail");
            free(evt.data);
        } else {
            // all good
        }
    }
}

/**
 * Check for incoming connections on TCP server
 *
 * @param tcp_master_socket Main open TCP socket to accept TCP connections/clients
 * @param tcp_clients List of active TCP client connections (socket IDs)
 */
void handle_tcp_master(const int tcp_master_socket, int tcp_clients[]) {
    struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
    uint32_t addr_len = sizeof(source_addr);
    int new_tcp_client = accept(tcp_master_socket, (struct sockaddr *) &source_addr, &addr_len);
    if (new_tcp_client > 0) {
        for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
            if (tcp_clients[i] <= 0) {
                tcp_clients[i] = new_tcp_client;
                fcntl(tcp_clients[i], F_SETFL, O_NONBLOCK);
                char addr_str[128];
                inet_ntoa_r(((struct sockaddr_in *) &source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                ESP_LOGI(TAG, "TCP: New client connected: %s", addr_str);
                num_connected_tcp_clients++;
                return;
            }
        }
        ESP_LOGI(TAG, "TCP: Could not accept connection. Too many clients connected.");
    }
}

/**
 *  Init/Create structure containing all UDP connection information
 * @return Structure containing all UDP connection information
 */
udp_conn_list_t *udp_client_list_create() {
    udp_conn_list_t *n_udp_conn_list = malloc(sizeof(udp_conn_list_t)); // Allocate memory for the list
    if (n_udp_conn_list == NULL) { // Check if the allocation failed
        return NULL; // Return NULL to indicate an error
    }
    n_udp_conn_list->size = 0; // Initialize the size to 0
    return n_udp_conn_list; // Return the pointer to the list
}

/**
 *  Destroy structure containing all UDP connection information
 * @param n_udp_conn_list Structure containing all UDP connection information
 */
void udp_client_list_destroy(udp_conn_list_t *n_udp_conn_list) {
    if (n_udp_conn_list == NULL) { // Check if the list is NULL
        return; // Do nothing
    }
    free(n_udp_conn_list); // Free the list
}

/**
 * Add a new UDP client to the list of known UDP clients. Checks if client is already known based on IP and port.
 * Added client will receive UDP packets with serial info and will be able to send UDP packets to the serial interface
 * of the ESP32.
 * PORT, MAC & IP should be set inside new_db_udp_client. If MAC is not set then the device cannot be removed later on.
 *
 * @param n_udp_conn_list Structure containing all UDP connection information
 * @param new_db_udp_client New client to add to the UDP list. PORT, MAC & IP must be set. If MAC is not set then the
 *                          device cannot be removed later on.
 * @return 1 if added - 0 if not
 */
bool add_to_known_udp_clients(udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client) {
    if (n_udp_conn_list == NULL) { // Check if the list is NULL
        return false; // Do nothing
    }
    if (n_udp_conn_list->size == MAX_UDP_CLIENTS) { // Check if the list is full
        return false; // Do nothing
    }
    for (int i = 0; i < n_udp_conn_list->size; i++) {
        if ((n_udp_conn_list->db_udp_clients[i].udp_client.sin_port == new_db_udp_client.udp_client.sin_port) &&
            (n_udp_conn_list->db_udp_clients[i].udp_client.sin_addr.s_addr ==
             new_db_udp_client.udp_client.sin_addr.s_addr)) {
            return false; // client existing - do not add
        }
    }
    n_udp_conn_list->db_udp_clients[n_udp_conn_list->size] = new_db_udp_client; // Copy the element data to the end of the array
    n_udp_conn_list->size++; // Increment the size of the list
    return true;
}

/**
 * Remove a client from the sending list. Client will no longer receive UDP packets. MAC address must be given.
 * Usually called in AP-Mode when a station disconnects. In any other case we will not know since UDP is a connection-less
 * protocol
 *
 * @param n_udp_conn_list Structure containing all UDP connection information
 * @param new_db_udp_client The UDP client to remove based on its MAC address
 * @return true if removed - false if nothing was removed
 */
bool remove_from_known_udp_clients(udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client) {
    if (n_udp_conn_list == NULL) { // Check if the list is NULL
        return false; // Do nothing
    }
    for (int i = 0; i < n_udp_conn_list->size; i++) { // Loop through the array
        if (memcmp(n_udp_conn_list->db_udp_clients[i].mac, new_db_udp_client.mac,
                   sizeof(n_udp_conn_list->db_udp_clients[i].mac)) ==
            0) { // Compare the current array element with the element
            // Found a match
            for (int j = i; j < n_udp_conn_list->size - 1; j++) { // Loop from the current index to the end of the array
                n_udp_conn_list->db_udp_clients[j] = n_udp_conn_list->db_udp_clients[j +
                                                                                     1]; // Shift the array elements to the left
            }
            n_udp_conn_list->size--; // Decrement the size of the list
            return true; // Exit the function
        }
    }
    // No match found
    return false;
}

/**
 * Reads UART transparently or parsing MSP/LTM protocol.
 * Then sends read data to all connected clients via TCP/UDP or ESP-NOW.
 * Non-Blocking function
 *
 * @param tcp_clients Array of connected TCP client sockets
 * @param transparent_buff_pos Counter variable for total read UART bytes
 * @param msp_ltm_buff_pos Pointer position/data length of destination-buffer for read MSP messages
 * @param msp_message_buffer Destination-buffer for read MSP messages
 * @param serial_buffer Destination-buffer for the serial data
 * @param db_msp_ltm_port Pointer to structure containing MSP/LTM parser information
 */
void read_process_uart(int *tcp_clients, uint *transparent_buff_pos, uint *msp_ltm_buff_pos, uint8_t *msp_message_buffer,
                       uint8_t *serial_buffer, msp_ltm_port_t *db_msp_ltm_port) {
    // ToDo: When to ESP32 for DroneBridge are used as sender and receiver - only one should be parsing the protocol
    switch (DB_SERIAL_PROTOCOL) {
        case 1:
        case 2:
            db_parse_msp_ltm(tcp_clients, udp_conn_list, msp_message_buffer, msp_ltm_buff_pos, db_msp_ltm_port);
            break;
        case 3:
        case 4:
            db_parse_mavlink(tcp_clients, udp_conn_list, msp_message_buffer, transparent_buff_pos);
            break;
        case 5:
        default:
            db_parse_transparent(tcp_clients, udp_conn_list, serial_buffer, transparent_buff_pos);
            break;
    }
}

/**
 * Thread that manages all incoming and outgoing ESP-NOW and serial (UART) connections.
 * Called only when ESP-NOW mode is selected
 */
_Noreturn void control_module_esp_now(){
    ESP_LOGI(TAG, "Starting control module (ESP-NOW)");
    // only open serial socket/UART if PINs are not matching - matching PIN nums mean they still need to be defined by
    // the user no pre-defined pins as of this release since ESP32 boards have wildly different pin configurations
    int uart_socket = open_serial_socket();
    if (uart_socket == ESP_FAIL) {
        ESP_LOGE(TAG, "UART socket not opened. Aborting start of control module.");
        vTaskDelete(NULL);
    }

    uint transparent_buff_pos = 0;
    uint msp_ltm_buff_pos = 0;
    uint8_t msp_message_buffer[UART_BUF_SIZE];
    uint8_t serial_buffer[DB_TRANS_BUF_SIZE];
    msp_ltm_port_t db_msp_ltm_port;
    db_espnow_uart_event_t db_espnow_uart_evt;
    uint delay_timer_cnt = 0;

    ESP_LOGI(TAG, "Started control module (ESP-NOW)");
    while (1) {
        read_process_uart(NULL, &transparent_buff_pos, &msp_ltm_buff_pos, msp_message_buffer, serial_buffer,
                          &db_msp_ltm_port);
        if (db_uart_write_queue != NULL && xQueueReceive(db_uart_write_queue, &db_espnow_uart_evt, 0) == pdTRUE) {
            write_to_uart(db_espnow_uart_evt.data, db_espnow_uart_evt.data_len);
            free(db_espnow_uart_evt.data);
        } else {
            if (db_uart_write_queue == NULL) ESP_LOGE(TAG, "db_uart_write_queue is NULL!");
            // no new data available do nothing
        }
        if (delay_timer_cnt == 5000) {
            /* all actions are non-blocking so allow some delay so that the IDLE task of FreeRTOS and the watchdog can run
            read: https://esp32developer.com/programming-in-c-c/tasks/tasks-vs-co-routines for reference */
            vTaskDelay(10 / portTICK_PERIOD_MS);
            delay_timer_cnt = 0;
        } else {
            delay_timer_cnt++;
        }
    }
    vTaskDelete(NULL);
}

/**
 * Sends DroneBridge internal telemetry to tell every connected WiFi station how well we receive their data (rssi).
 * Uses UDP connection list to map IP to MAC address of connected stations. Uses wifi_sta_list to get a mac to rssi mapping
 * Internal telemetry uses DB_ESP32_INTERNAL_TELEMETRY_PORT port
 *
 * @param sta_list
 * @param udp_conns
 */
void db_tell_stations_their_rssi(wifi_sta_list_t *sta_list, udp_conn_list_t *udp_conns) {
    if (DB_WIFI_MODE == DB_WIFI_MODE_AP_LR && udp_conns->size > 0 && sta_list->num > 0) {
        // determine udp client and its IP based on MAC
        for (int i = 0; i < sta_list->num; i++) {
            for (int v = 0; v < udp_conns->size; v++) {
                if (sta_list->sta[i].mac[0] == udp_conns->db_udp_clients[v].mac[0] &&
                        sta_list->sta[i].mac[1] == udp_conns->db_udp_clients[v].mac[1] &&
                        sta_list->sta[i].mac[2] == udp_conns->db_udp_clients[v].mac[2] &&
                        sta_list->sta[i].mac[3] == udp_conns->db_udp_clients[v].mac[3] &&
                        sta_list->sta[i].mac[4] == udp_conns->db_udp_clients[v].mac[4] &&
                        sta_list->sta[i].mac[5] == udp_conns->db_udp_clients[v].mac[5]) {
                    // found a match between macs of station of wifi_sta_list and udp_conn_list
                    // create target UDP based on udp_conn_list entry, overwriting target port
                    struct sockaddr_in telem_udp_client;
                    memcpy(&telem_udp_client, &udp_conns->db_udp_clients[v].udp_client, sizeof(telem_udp_client));
                    telem_udp_client.sin_port = htons(DB_ESP32_INTERNAL_TELEMETRY_PORT);
                    // send that IP the rssi value we are seeing when that IP sends us something. Send to DB_ESP32_INTERNAL_TELEMETRY_PORT
                    int sent = sendto(udp_conns->udp_socket, &sta_list->sta[i].rssi, 1, 0,
                                      (struct sockaddr *) &telem_udp_client,
                                      sizeof(telem_udp_client));
                    if (sent < 0) {
                        // we really do not care about errors,
                        ESP_LOGD(TAG, "Failed sending internal telemetry (%i)", sent);
                    }
                } else {
                    // no match, keep looking
                }
            }
        }
    } else {
        // in other modes we cannot do that. ESP-NOW uses a different function and way of telling
    }
}

/**
 * Receive and process internal telemetry (ESP32 to ESP32) sent by ESP32 LR access point.
 * Matches with db_tell_stations_their_rssi()
 * Sets station_rssi_ap based on the received value
 *
 * @param tel_sock Socket listening for internal telemetry
 */
void handle_internal_telemetry(int tel_sock, uint8_t *udp_buffer, socklen_t *sock_len, struct sockaddr_in *udp_client) {
    if (tel_sock > 0) {
        ssize_t recv_length = recvfrom(tel_sock, udp_buffer, UDP_BUF_SIZE, 0,
                                       (struct sockaddr *) udp_client, sock_len);
        if (recv_length > 0) {
            station_rssi_ap = (uint8_t) udp_buffer[0];
            ESP_LOGD(TAG, "Received %i bytes int. telem - AP receives our packets with RSSI: %i", recv_length, station_rssi_ap);
        }
    }
}

/**
 * Thread that manages all incoming and outgoing TCP, UDP and serial (UART) connections.
 * Called when Wi-Fi modes are selected
 */
_Noreturn void control_module_udp_tcp() {
    ESP_LOGI(TAG, "Starting control module (Wi-Fi)");
    int uart_socket = open_serial_socket();
    if (uart_socket == ESP_FAIL) {
        ESP_LOGE(TAG, "UART socket not opened. Aborting start of control module.");
        vTaskDelete(NULL);
    }

    int tcp_master_socket = open_tcp_server(app_port_proxy);
    if (tcp_master_socket == ESP_FAIL) {
        ESP_LOGE(TAG, "TCP master socket failed to open. Aborting start of control module.");
        vTaskDelete(NULL);
    }
    fcntl(tcp_master_socket, F_SETFL, O_NONBLOCK);
    int tcp_clients[CONFIG_LWIP_MAX_ACTIVE_TCP];
    for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
        tcp_clients[i] = -1;
    }

    udp_conn_list->udp_socket = db_open_serial_udp_socket();
    int db_internal_telem_udp_sock = -1;
    if (DB_WIFI_MODE == DB_WIFI_MODE_STA) {
        db_internal_telem_udp_sock = db_open_int_telemetry_udp_socket();
    } else {
        // other WiFi modes do not need this. Only WiFi stations will receive if connected to LR access point.
        // ESP-NOW uses different sockets/systems
    }
    uint8_t udp_buffer[UDP_BUF_SIZE];
    struct db_udp_client_t new_db_udp_client = {0};
    socklen_t udp_socklen = sizeof(new_db_udp_client.udp_client);

    uint transparent_buff_pos = 0;
    uint msp_ltm_buff_pos = 0;
    uint8_t tcp_client_buffer[TCP_BUFF_SIZ];
    memset(tcp_client_buffer, 0, TCP_BUFF_SIZ);
    uint8_t msp_message_buffer[UART_BUF_SIZE];
    uint8_t serial_buffer[DB_TRANS_BUF_SIZE];
    msp_ltm_port_t db_msp_ltm_port;
    int delay_timer_cnt = 0;

    ESP_LOGI(TAG, "Started control module (Wi-Fi)");
    while (1) {
        // Read incoming wireless data (Wi-Fi)
        // Wi-Fi based modes that use TCP and UDP communication
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
                    num_connected_tcp_clients--;
                } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    ESP_LOGE(TAG, "Error receiving from TCP client %i (fd: %i): %d", i, tcp_clients[i], errno);
                    shutdown(tcp_clients[i], 0);
                    close(tcp_clients[i]);
                    num_connected_tcp_clients--;
                    tcp_clients[i] = -1;
                }
            }
        }
        // handle incoming UDP data - Read UDP and forward to UART
        // all devices that send us UDP data will be added to the list of MAVLink UDP receivers
        ssize_t recv_length = recvfrom(udp_conn_list->udp_socket, udp_buffer, UDP_BUF_SIZE, 0,
                                       (struct sockaddr *) &new_db_udp_client.udp_client, &udp_socklen);
        if (recv_length > 0) {
            ESP_LOGD(TAG, "UDP: Received %i bytes", recv_length);
            write_to_uart(udp_buffer, recv_length);
            // Allows to register new app on different port. Used e.g. for UDP conn setup in sta-mode.
            // Devices/Ports added this way cannot be removed in sta-mode since UDP is connectionless, and we cannot
            // determine if the client is still existing. This will blow up the list connected devices.
            // In AP-Mode the devices can be removed based on the IP/MAC address
            add_to_known_udp_clients(udp_conn_list, new_db_udp_client);
        }

        if (DB_WIFI_MODE == DB_WIFI_MODE_STA) {
            handle_internal_telemetry(db_internal_telem_udp_sock, udp_buffer, &udp_socklen, &new_db_udp_client.udp_client);
        }

        // Second check for incoming UART data and send it to TCP/UDP
        read_process_uart(tcp_clients, &transparent_buff_pos, &msp_ltm_buff_pos, msp_message_buffer, serial_buffer,
                          &db_msp_ltm_port);

        if (delay_timer_cnt == 9000) {
            // all actions are non-blocking so allow some delay so that the IDLE task of FreeRTOS and the watchdog can run
            // read: https://esp32developer.com/programming-in-c-c/tasks/tasks-vs-co-routines for reference
            vTaskDelay(10 / portTICK_PERIOD_MS);
            delay_timer_cnt = 0;
            // Use the opportunity to get some regular status information like rssi
            if (DB_WIFI_MODE == DB_WIFI_MODE_STA) {
                // update rssi variable - set to 0 when not available
                if (esp_wifi_sta_get_rssi(&station_rssi) != ESP_OK) {
                    station_rssi = 0;
                }
            } else if (DB_WIFI_MODE == DB_WIFI_MODE_AP || DB_WIFI_MODE == DB_WIFI_MODE_AP_LR) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_ap_get_sta_list(&wifi_sta_list));
                db_tell_stations_their_rssi(&wifi_sta_list, udp_conn_list);
            } else {
                // no way of getting RSSI here. Do nothing
            }
        } else {
            delay_timer_cnt++;
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
    if (DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_GND && DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_AIR) {
        xTaskCreate(&control_module_udp_tcp, "control_wifi", 40960, NULL, 5, NULL);
    } else {
        xTaskCreate(&control_module_esp_now, "control_espnow", 40960, NULL, 5, NULL);
    }
}
