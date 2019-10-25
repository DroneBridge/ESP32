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

#include <sys/socket.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <string.h>
#include <nvs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "globals.h"
#include <math.h>
#include <driver/gpio.h>

#define LISTENQ 2
#define REQUEST_BUF_SIZE 1024
#define WEBSITE_RESPONSE_BUFFER_SIZE 3072
#define TAG "TCP_SERVER"

const char *save_response = "HTTP/1.1 200 OK\r\n"
                            "Server: DroneBridgeESP32\r\n"
                            "Content-type: text/html, text, plain\r\n"
                            "\r\n"
                            "<!DOCTYPE html>\n"
                            "<html>\n"
                            "<head>\n"
                            "  <style>\n"
                            "  .mytext {\n"
                            "    font-family: Verdana, Geneva, sans-serif;\n"
                            "    color: #FFFFF;\n"
                            "  }\n"
                            "  h1 {\n"
                            "    font-family: Verdana, Geneva, sans-serif;\n"
                            "    color: #FF7900;\n"
                            "  }\n"
                            "  </style>\n"
                            "<title>DB for ESP32 Settings</title>\n"
                            "<meta name=\"viewport\" content=\"width=device-width, user-scalable=no\" />\n"
                            "</head>\n"
                            "<body>\n"
                            "  <h1>DroneBridge for ESP32</h1>\n"
                            "  <p class=\"mytext\">Saved settings!</p>\n"
                            "  <a class=\"mytext\" href=\"/\">Back to settings</a>\n"
                            "</body>\n"
                            "</html>\n"
                            "";

const char *bad_gateway = "HTTP/1.1 502 Bad Gateway \r\n"
                          "Server: DroneBridgeESP32 \r\n"
                          "Content-Type: text/html \r\n"
                          "\r\n";

/**
 * @brief Check if we got a simple request or new settings
 * @param request_buffer
 * @param length
 * @return 0 if GET-Request, 1 if new settings, -1 if none
 */
int http_request_type(uint8_t *request_buffer, uint length) {
    uint8_t http_get_header[] = {'G', 'E', 'T', ' ', '/', ' ', 'H', 'T', 'T', 'P'};
    uint8_t http_get_header_settings[] = {'G', 'E', 'T', ' ', '/', 's', 'e', 't', 't', 'i', 'n', 'g', 's'};
    uint8_t http_get_header_other_data[] = {'G', 'E', 'T', ' ', '/'};
    if (memcmp(request_buffer, http_get_header, sizeof(http_get_header)) == 0) return 0;
    if (memcmp(request_buffer, http_get_header_settings, sizeof(http_get_header_settings)) == 0) return 1;
    if (memcmp(request_buffer, http_get_header_other_data, sizeof(http_get_header_other_data)) == 0) return 2;
    return -1;
}


void write_settings_to_nvs() {
    ESP_LOGI(TAG, "Saving to NVS");
    nvs_handle my_handle;
    ESP_ERROR_CHECK(nvs_open("settings", NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ssid", (char *) DEFAULT_SSID));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "wifi_pass", (char *) DEFAULT_PWD));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "wifi_chan", DEFAULT_CHANNEL));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "baud", DB_UART_BAUD_RATE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_tx", DB_UART_PIN_TX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_rx", DB_UART_PIN_RX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "proto", SERIAL_PROTOCOL));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "trans_pack_size", TRANSPARENT_BUF_SIZE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "ltm_per_packet", LTM_FRAME_NUM_BUFFER));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    nvs_close(my_handle);
}


void parse_save_get_parameters(char *request_buffer, uint length) {
    ESP_LOGI(TAG, "Parsing new settings:");
    char *ptr;
    char delimiter[] = "?=& ";
    ptr = strtok(request_buffer, delimiter);
    while (ptr != NULL) {
        if (strcmp(ptr, "ssid") == 0) {
            ptr = strtok(NULL, delimiter);
            if (strlen(ptr) >= 1) {
                strcpy((char *) DEFAULT_SSID, ptr);
                for(size_t i = 0; i <= strlen((char *) DEFAULT_SSID); i++)
                    if(DEFAULT_SSID[i] == '+'){ DEFAULT_SSID[i] = ' '; }  // replace + with space
                ESP_LOGI(TAG, "New ssid: %s", DEFAULT_SSID);
            }
        } else if (strcmp(ptr, "wifi_pass") == 0) {
            ptr = strtok(NULL, delimiter);
            if (strlen(ptr) >= 8) {
                strcpy((char *) DEFAULT_PWD, ptr);
                ESP_LOGI(TAG, "New password: %s", DEFAULT_PWD);
            }
        } else if (strcmp(ptr, "wifi_chan") == 0) {
            ptr = strtok(NULL, delimiter);
            if (atoi(ptr) <= 13) DEFAULT_CHANNEL = atoi(ptr);
            ESP_LOGI(TAG, "New wifi channel: %i", DEFAULT_CHANNEL);
        } else if (strcmp(ptr, "baud") == 0) {
            ptr = strtok(NULL, delimiter);
            if (atoi(ptr) > 2399)
                DB_UART_BAUD_RATE = atoi(ptr);
            ESP_LOGI(TAG, "New baud: %i", DB_UART_BAUD_RATE);
        } else if (strcmp(ptr, "gpio_tx") == 0) {
            ptr = strtok(NULL, delimiter);
            if (atoi(ptr) <= GPIO_NUM_MAX) DB_UART_PIN_TX = atoi(ptr);
            ESP_LOGI(TAG, "New gpio_tx: %i", DB_UART_PIN_TX);
        } else if (strcmp(ptr, "gpio_rx") == 0) {
            ptr = strtok(NULL, delimiter);
            if (atoi(ptr) <= GPIO_NUM_MAX) DB_UART_PIN_RX = atoi(ptr);
            ESP_LOGI(TAG, "New gpio_rx: %i", DB_UART_PIN_RX);
        } else if (strcmp(ptr, "proto") == 0) {
            ptr = strtok(NULL, delimiter);
            if (strcmp(ptr, "msp_ltm") == 0) {
                SERIAL_PROTOCOL = 2;
            } else {
                SERIAL_PROTOCOL = 4;
            }
            ESP_LOGI(TAG, "New proto: %i", SERIAL_PROTOCOL);
        } else if (strcmp(ptr, "trans_pack_size") == 0) {
            ptr = strtok(NULL, delimiter);
            TRANSPARENT_BUF_SIZE = atoi(ptr);
            ESP_LOGI(TAG, "New trans_pack_size: %i", TRANSPARENT_BUF_SIZE);
        } else if (strcmp(ptr, "ltm_per_packet") == 0) {
            ptr = strtok(NULL, delimiter);
            LTM_FRAME_NUM_BUFFER = atoi(ptr);
            ESP_LOGI(TAG, "New ltm_per_packet: %i", LTM_FRAME_NUM_BUFFER);
        } else {
            ptr = strtok(NULL, delimiter);
        }
    }
    write_settings_to_nvs();
}


char *create_response(char *website_response) {
    char baud_selection1[9] = "";
    char baud_selection2[9] = "";
    char baud_selection3[9] = "";
    char baud_selection4[9] = "";
    char baud_selection5[9] = "";
    char baud_selection6[9] = "";
    char baud_selection7[9] = "";
    char uart_serial_selection1[9] = "";
    char uart_serial_selection2[9] = "";
    char trans_pack_size_selection1[9] = "";
    char trans_pack_size_selection2[9] = "";
    char trans_pack_size_selection3[9] = "";
    char trans_pack_size_selection4[9] = "";
    char trans_pack_size_selection5[9] = "";
    char ltm_size_selection1[9] = "";
    char ltm_size_selection2[9] = "";
    char ltm_size_selection3[9] = "";
    char ltm_size_selection4[9] = "", ltm_size_selection5[9] = "";

    switch (SERIAL_PROTOCOL) {
        default:
        case 1:
        case 2:
            strcpy(uart_serial_selection1, "selected");
            break;
        case 3:
        case 4:
        case 5:
            strcpy(uart_serial_selection2, "selected");
            break;
    }
    switch (TRANSPARENT_BUF_SIZE) {
        case 16:
            strcpy(trans_pack_size_selection1, "selected");
            break;
        case 32:
            strcpy(trans_pack_size_selection2, "selected");
            break;
        default:
        case 64:
            strcpy(trans_pack_size_selection3, "selected");
            break;
        case 128:
            strcpy(trans_pack_size_selection4, "selected");
            break;
        case 256:
            strcpy(trans_pack_size_selection5, "selected");
            break;
    }
    switch (LTM_FRAME_NUM_BUFFER) {
        default:
        case 1:
            strcpy(ltm_size_selection1, "selected");
            break;
        case 2:
            strcpy(ltm_size_selection2, "selected");
            break;
        case 3:
            strcpy(ltm_size_selection3, "selected");
            break;
        case 4:
            strcpy(ltm_size_selection4, "selected");
            break;
        case 5:
            strcpy(ltm_size_selection5, "selected");
            break;
    }
    switch (DB_UART_BAUD_RATE) {
        default:
        case 115200:
            strcpy(baud_selection1, "selected");
            break;
        case 57600:
            strcpy(baud_selection2, "selected");
            break;
        case 38400:
            strcpy(baud_selection3, "selected");
            break;
        case 19200:
            strcpy(baud_selection4, "selected");
            break;
        case 9600:
            strcpy(baud_selection5, "selected");
            break;
        case 4800:
            strcpy(baud_selection6, "selected");
            break;
        case 2400:
            strcpy(baud_selection7, "selected");
            break;
    }
    char build_version[16];
    sprintf(build_version, "v%.2f", floorf(BUILDVERSION) / 100);
    sprintf(website_response, "HTTP/1.1 200 OK\r\n"
                              "Server: DroneBridgeESP32\r\n"
                              "Content-type: text/html, text, plain\r\n"
                              "\r\n"
                              "<!DOCTYPE html><html><head><title>DB for ESP32 Settings</title>"
                              "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">"
                              "<style> "
                              "h1 { font-family: Verdana, Geneva, sans-serif; color: #FF7900; } "
                              "table.DroneBridge { font-family: Verdana, Geneva, sans-serif; background-color: #145396; text-align: right; border-collapse: collapse; } "
                              "table.DroneBridge td, table.DroneBridge th { padding: 6px 0px; } "
                              "table.DroneBridge tbody td { font-size: 1rm; font-weight: bold; color: #FFFFFF; padding: 0.5em; } "
                              "table.DroneBridge td:nth-child(even) { background: #FF7900; }"
                              "@media only screen and (max-width:768px) { .DroneBridge {width:100%%; }}"
                              ".foot{color: #145396;font-family: Verdana, Geneva, sans-serif;font-size: 0.8em;}"
                              "</style>"
                              "</head>\n"
                              "<body><h1>DroneBridge for ESP32</h1>"
                              "<form action=\"/settings.html\" id=\"settings_form\" method=\"get\" target=\"_blank\">"
                              "<table class=\"DroneBridge\"><tbody>"
                              "<tr><td>Wifi SSID</td>"
                              "<td><input type=\"text\" name=\"ssid\" value=\"%s\"></td></tr>"
                              "<tr><td>Wifi password</td>"
                              "<td><input type=\"text\" name=\"wifi_pass\" value=\"%s\"></td></tr>"
                              "<tr><td>Wifi channel</td>"
                              "<td><input type=\"number\" name=\"wifi_chan\" min=\"0\" max=\"13\" value=\"%i\"></td></tr>"
                              "<tr><td>UART baud rate</td><td>"
                              "<select name=\"baud\" form=\"settings_form\">"
                              "<option %s value=\"115200\">115200</option>"
                              "<option %s value=\"57600\">57600</option>"
                              "<option %s value=\"38400\">38400</option>"
                              "<option %s value=\"19200\">19200</option>"
                              "<option %s value=\"9600\">9600</option>"
                              "<option %s value=\"4800\">4800</option>"
                              "<option %s value=\"2400\">2400</option>"
                              "</select>"
                              "</td></tr><tr><td>GPIO TX pin number</td>"
                              "<td><input type=\"text\" name=\"gpio_tx\" value=\"%i\"></td></tr>"
                              "<tr><td>GPIO RX pin number</td><td>"
                              "<input type=\"text\" name=\"gpio_rx\" value=\"%i\">"
                              "</td></tr><tr><td>UART serial protocol</td><td>"
                              "<select name=\"proto\" form=\"settings_form\">"
                              "<option %s value=\"msp_ltm\">MSP/LTM</option>"
                              "<option %s value=\"trans\">Transparent/MAVLink</option>"
                              "</select>"
                              "</td></tr><tr><td>Transparent packet size</td><td>"
                              "<select name=\"trans_pack_size\" form=\"settings_form\">"
                              "<option %s value=\"16\">16</option><option %s value=\"32\">32</option>"
                              "<option %s value=\"64\">64</option><option %s value=\"128\">128</option>"
                              "<option %s value=\"256\">256</option>"
                              "</select>"
                              "</td></tr><tr><td>LTM frames per packet</td><td>"
                              "<select name=\"ltm_per_packet\" form=\"settings_form\">"
                              "<option %s value=\"1\">1</option>"
                              "<option %s value=\"2\">2</option>"
                              "<option %s value=\"3\">3</option>"
                              "<option %s value=\"4\">4</option>"
                              "<option %s value=\"5\">5</option>"
                              "</select>"

                              "</td></tr><tr><td></td><td>"
                              "</td></tr></tbody></table><p></p>"

                              "<input target= \"_top\" type=\"submit\" value=\"Save\">"
                              "</form>"
                              "<p class=\"foot\">%s</p>\n"
                              "<p class=\"foot\">&copy; Wolfgang Christl 2018 - Apache 2.0 License</p>"
                              "</body></html>\n"
                              "", DEFAULT_SSID, DEFAULT_PWD, DEFAULT_CHANNEL, baud_selection1, baud_selection2, baud_selection3, baud_selection4,
            baud_selection5, baud_selection6, baud_selection7, DB_UART_PIN_TX, DB_UART_PIN_RX, uart_serial_selection1,
            uart_serial_selection2, trans_pack_size_selection1, trans_pack_size_selection2, trans_pack_size_selection3,
            trans_pack_size_selection4, trans_pack_size_selection5, ltm_size_selection1, ltm_size_selection2,
            ltm_size_selection3, ltm_size_selection4, ltm_size_selection5, build_version);
    return website_response;
}

void http_settings_server(void *parameter) {
    ESP_LOGI(TAG, "http_settings_server task started");
    struct sockaddr_in tcpServerAddr;
    tcpServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    tcpServerAddr.sin_family = AF_INET;
    tcpServerAddr.sin_port = htons(80);
    int tcp_socket, r;
    char recv_buf[64];
    static struct sockaddr_in remote_addr;
    static unsigned int socklen;
    socklen = sizeof(remote_addr);
    xEventGroupWaitBits(wifi_event_group, BIT2, false, true, portMAX_DELAY);
    while (1) {
        tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (tcp_socket < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        if (bind(tcp_socket, (struct sockaddr *) &tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
            ESP_LOGE(TAG, "... socket bind failed errno=%d", errno);
            close(tcp_socket);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        if (listen(tcp_socket, LISTENQ) != 0) {
            ESP_LOGE(TAG, "... socket listen failed errno=%d", errno);
            close(tcp_socket);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        uint8_t *request_buffer = malloc(REQUEST_BUF_SIZE * sizeof(uint8_t));
        // char *website_response = malloc(WEBSITE_RESPONSE_BUFFER_SIZE*sizeof(char));
        char website_response[WEBSITE_RESPONSE_BUFFER_SIZE];
        while (1) {
            int client_socket = accept(tcp_socket, (struct sockaddr *) &remote_addr, &socklen);
            fcntl(client_socket, F_SETFL, O_NONBLOCK);
            uint rec_length = 0;
            do {
                bzero(recv_buf, sizeof(recv_buf));
                r = recv(client_socket, recv_buf, sizeof(recv_buf) - 1, 0);
                if (r > 0) {
                    if (REQUEST_BUF_SIZE >= (rec_length + r)) {
                        memcpy(&request_buffer[rec_length], recv_buf, (size_t) r);
                        rec_length += r;
                    } else {
                        ESP_LOGE(TAG, "Request bigger than buffer");
                    }
                }
            } while (r > 0);
            // prints the requests for debugging
//            ESP_LOGI(TAG2,"New connection request,Request data:");
//            for(int i = 0; i < rec_length; i++) {
//                putchar(request_buffer[i]);
//            }
            int http_req = http_request_type(request_buffer, rec_length);
            if (http_req == 0) {
                char *response = create_response(website_response);
                if (write(client_socket, response, strlen(response)) < 0) {
                    ESP_LOGE(TAG, "... Send failed");
                    close(tcp_socket);
                    vTaskDelay(4000 / portTICK_PERIOD_MS);
                    continue;
                }
            } else if (http_req == 1) {
                parse_save_get_parameters((char *) request_buffer, rec_length);
                if (write(client_socket, save_response, strlen(save_response)) < 0) {
                    ESP_LOGE(TAG, "... Send failed");
                    close(tcp_socket);
                    vTaskDelay(4000 / portTICK_PERIOD_MS);
                    continue;
                }
            } else if (http_req == 2) {
                if (write(client_socket, bad_gateway, strlen(bad_gateway)) < 0) {
                    ESP_LOGE(TAG, "... Send failed");
                    close(tcp_socket);
                    vTaskDelay(4000 / portTICK_PERIOD_MS);
                    continue;
                }
            }
            close(client_socket);
        }
        //free(website_response);
        free(request_buffer);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "...tcp_client task closed\n");
    vTaskDelete(NULL);
}


/**
 * @brief Starts a TCP server that serves the page to change settings & handles the changes
 */
void start_tcp_server() {
    xTaskCreate(&http_settings_server, "http_settings_server", 10240, NULL, 5, NULL);
}
