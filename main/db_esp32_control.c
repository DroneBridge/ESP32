#include "db_esp32_control.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_vfs_dev.h>
#include <esp_wifi.h>
#include <string.h>
#include <sys/cdefs.h>
#include <sys/fcntl.h>
#include <sys/param.h>

#include "db_protocol.h"
#include "globals.h"
#include "main.h"
#include "mavlink_parser.h"
#include "msp_ltm_serial.h"
#include "tcp_server.h"
#include "udp_server.h"

#define UART_NUM UART_NUM_1

#define TAG "DB_CONTROL"

#define MAX_TASK_ITERATIONS 2000    // every MAX_TASK_ITERATIONS delay is added to allow IDLE task to run

uint16_t app_port_proxy = APP_PORT_PROXY;
uint8_t ltm_frame_buffer[MAX_LTM_FRAMES_IN_BUFFER * LTM_MAX_FRAME_SIZE];
uint ltm_frames_in_buffer = 0;
uint ltm_frames_in_buffer_pnt = 0;

uint32_t uart_byte_count = 0;
int8_t num_connected_tcp_clients = 0;

esp_err_t open_serial_socket() {
    uart_config_t uart_config = {
        .baud_rate = DB_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, DB_UART_PIN_TX, DB_UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    return uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0);
}

/**
 * Send to all connected TCP & UDP clients
 *
 * @param tcp_clients Array of socket IDs for the TCP clients
 * @param udp_conn Structure handling the UDP connection
 * @param data payload to send
 * @param data_length Length of payload to send
 */
void send_to_all_clients(int tcp_clients[], struct udp_conn_list_t *n_udp_conn_list, uint8_t data[], uint data_length) {
    if (tcp_clients[0] > 0) {
        send_to_all_tcp_clients(tcp_clients, data, data_length);
    }
    if (udp_conn_list->size > 0) {
        send_to_all_udp_clients(n_udp_conn_list, data, data_length);
    }
}

/**
 * Writes data from buffer to UART
 * @param data_buffer Payload to write to UART
 * @param data_length Size of payload to write to UART
 */
void write_to_uart(const char data_buffer[], const size_t data_length) {
    int written = uart_write_bytes(UART_NUM, data_buffer, data_length);
    if (written > 0)
        ESP_LOGD(TAG, "Wrote %i bytes", written);
    else
        ESP_LOGE(TAG, "Error writing to UART %s", esp_err_to_name(errno));
}

/**
 * @brief Parses & sends complete MSP & LTM messages
 */
void parse_msp_ltm(int tcp_clients[], struct udp_conn_list_t *udp_connection, uint8_t msp_message_buffer[],
                   uint *serial_read_bytes,
                   msp_ltm_port_t *db_msp_ltm_port) {
    uint8_t serial_bytes[TRANS_RD_BYTES_NUM];
    uint read;
    if ((read = uart_read_bytes(UART_NUM, serial_bytes, TRANS_RD_BYTES_NUM, 0)) > 0) {
        uart_byte_count += read;
        for (uint j = 0; j < read; j++) {
            (*serial_read_bytes)++;
            uint8_t serial_byte = serial_bytes[j];
            if (parse_msp_ltm_byte(db_msp_ltm_port, serial_byte)) {
                msp_message_buffer[(*serial_read_bytes - 1)] = serial_byte;
                if (db_msp_ltm_port->parse_state == MSP_PACKET_RECEIVED) {
                    *serial_read_bytes = 0;
                    send_to_all_clients(tcp_clients, udp_connection, msp_message_buffer, *serial_read_bytes);
                } else if (db_msp_ltm_port->parse_state == LTM_PACKET_RECEIVED) {
                    memcpy(&ltm_frame_buffer[ltm_frames_in_buffer_pnt], db_msp_ltm_port->ltm_frame_buffer,
                           (db_msp_ltm_port->ltm_payload_cnt + 4));
                    ltm_frames_in_buffer_pnt += (db_msp_ltm_port->ltm_payload_cnt + 4);
                    ltm_frames_in_buffer++;
                    if (ltm_frames_in_buffer == LTM_FRAME_NUM_BUFFER &&
                        (LTM_FRAME_NUM_BUFFER <= MAX_LTM_FRAMES_IN_BUFFER)) {
                        send_to_all_clients(tcp_clients, udp_connection, ltm_frame_buffer, *serial_read_bytes);
                        ESP_LOGV(TAG, "Sent %i LTM message(s) to telemetry port!", LTM_FRAME_NUM_BUFFER);
                        ltm_frames_in_buffer = 0;
                        ltm_frames_in_buffer_pnt = 0;
                        *serial_read_bytes = 0;
                    }
                }
            }
        }
    }
}

/**
 * Reads TRANS_RD_BYTES_NUM bytes from UART and checks if we already got enough bytes to send them out
 *
 * @param tcp_clients Array of connected TCP clients
 * @param udp_connection Structure containing all UDP connection data including the sockets
 * @param serial_buffer Buffer that gets filled with data and then sent via TCP and UDP
 * @param serial_read_bytes Number of bytes already read for the current packet
 */
void parse_transparent(int tcp_clients[], struct udp_conn_list_t *udp_connection, uint8_t serial_buffer[],
                       uint *serial_read_bytes) {
    uint16_t read;
    // read from UART directly into TCP & UDP send buffer
    if ((read = uart_read_bytes(UART_NUM, &serial_buffer[*serial_read_bytes], (TRANSPARENT_BUF_SIZE - *serial_read_bytes), 0)) > 0) {
        xStreamBufferSend(xStreamBufferMavlinkSerial, &(serial_buffer[*serial_read_bytes]), read, 0);
        uart_byte_count += read;  // increase total bytes read via UART
        *serial_read_bytes += read;
        if (*serial_read_bytes >= TRANSPARENT_BUF_SIZE) {
            send_to_all_clients(tcp_clients, udp_connection, serial_buffer, *serial_read_bytes);
            *serial_read_bytes = 0;  // reset buffer position
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
    struct sockaddr_in6 source_addr;  // Large enough for both IPv4 or IPv6
    uint32_t addr_len = sizeof(source_addr);
    int new_tcp_client = accept(tcp_master_socket, (struct sockaddr *)&source_addr, &addr_len);
    if (new_tcp_client > 0) {
        for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
            if (tcp_clients[i] <= 0) {
                tcp_clients[i] = new_tcp_client;
                fcntl(tcp_clients[i], F_SETFL, O_NONBLOCK);
                char addr_str[128];
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                ESP_LOGI(TAG, "TCP: New client connected: %s", addr_str);
                num_connected_tcp_clients++;
                return;
            }
        }
        ESP_LOGI(TAG, "TCP: Could not accept connection. Too many clients connected.");
    }
}

/**
 * Thread that manages all incoming and outgoing TCP, UDP and serial (UART) connections
 */
void control_module_udp_tcp() {
    ESP_LOGI(TAG, "Starting control module");
    // only open serial socket/UART if PINs are not matching - matching PIN nums mean they still need to be defined by
    // the user no pre-defined pins as of this release since ESP32 boards have wildly different pin configurations
    int uart_socket = ESP_FAIL;
    if (DB_UART_PIN_RX != DB_UART_PIN_TX) {
        uart_socket = open_serial_socket();
    } else {
        // do no long continue setting up the system and kill task
        ESP_LOGW(TAG, "Init of control module aborted. TX GPIO == RX GPIO - Configure first!");
        vTaskDelete(NULL);
    }
    int tcp_master_socket = open_tcp_server(app_port_proxy);

    udp_conn_list->udp_socket = open_udp_socket();
    char udp_buffer[UDP_BUF_SIZE];
    struct db_udp_client_t new_db_udp_client = {0};
    socklen_t udp_socklen = sizeof(new_db_udp_client.udp_client);

    int tcp_clients[CONFIG_LWIP_MAX_ACTIVE_TCP];
    for (int i = 0; i < CONFIG_LWIP_MAX_ACTIVE_TCP; i++) {
        tcp_clients[i] = -1;
    }
    if (tcp_master_socket == ESP_FAIL || uart_socket == ESP_FAIL) {
        ESP_LOGE(TAG, "Can not start control module: tcp socket: %i UART socket: %i", tcp_master_socket, uart_socket);
    }
    fcntl(tcp_master_socket, F_SETFL, O_NONBLOCK);
    uint read_transparent = 0;
    uint read_msp_ltm = 0;
    char tcp_client_buffer[TCP_BUFF_SIZ];
    memset(tcp_client_buffer, 0, TCP_BUFF_SIZ);
    uint8_t msp_message_buffer[UART_BUF_SIZE];
    uint8_t serial_buffer[TRANSPARENT_BUF_SIZE];
    msp_ltm_port_t db_msp_ltm_port;
    int delay_timer_cnt = 0;

    ESP_LOGI(TAG, "Started control module");
    while (1) {
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
        // handle incoming UDP data - Read UDP and forward to flight controller
        // all devices that send us UDP data will be added to the list of MAVLink UDP receivers
        ssize_t recv_length = recvfrom(udp_conn_list->udp_socket, udp_buffer, UDP_BUF_SIZE, 0,
                                       (struct sockaddr *)&new_db_udp_client.udp_client, &udp_socklen);
        if (recv_length > 0) {
            ESP_LOGD(TAG, "UDP: Received %i bytes", recv_length);
            write_to_uart(udp_buffer, recv_length);
            // Allows to register new app on different port. Used e.g. for UDP conn setup in sta-mode.
            // Devices/Ports added this way cannot be removed in sta-mode since UDP is connectionless, and we cannot
            // determine if the client is still existing. This will blow up the list connected devices.
            // In AP-Mode the devices can be removed based on the IP/MAC address
            add_to_known_udp_clients(udp_conn_list, new_db_udp_client);
        }
        switch (SERIAL_PROTOCOL) {
            case 1:
            case 2:
                parse_msp_ltm(tcp_clients, udp_conn_list, msp_message_buffer, &read_msp_ltm, &db_msp_ltm_port);
                break;
            default:
            case 3:
            case 4:
            case 5:
                parse_transparent(tcp_clients, udp_conn_list, serial_buffer, &read_transparent);
                // if (tcp_clients[0] > 0) {
                //         delay_timer_cnt += 500; // increase delay timer counter to allow IDLE task to run because of TCP taking too long to send
                //     }
                // }               
                break;
        }
        if (delay_timer_cnt >= MAX_TASK_ITERATIONS) {
            // all actions are non-blocking so allow some delay so that the IDLE task of FreeRTOS and the watchdog can run
            // read: https://esp32developer.com/programming-in-c-c/tasks/tasks-vs-co-routines for reference
            vTaskDelay(10 / portTICK_PERIOD_MS);
            delay_timer_cnt = 0;
            if (DB_NETIF_MODE == DB_WIFI_MODE_STA) {
                // update rssi variable - set to 0 when not available
                if (esp_wifi_sta_get_rssi(&station_rssi) != ESP_OK) {
                    station_rssi = 0;
                }
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
    xTaskCreatePinnedToCore(&control_module_udp_tcp, "control_tcp", 40960, NULL, 5, NULL, 0);
}
