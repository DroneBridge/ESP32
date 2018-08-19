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

#include <stdio.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <driver/uart.h>
#include <esp_wifi_types.h>
#include <mdns.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "db_esp32_settings.h"
#include "db_esp32_control.h"
#include "globals.h"
#include "tcp_server.h"
#include "db_esp32_comm.h"

EventGroupHandle_t wifi_event_group;
static const char *TAG = "DB_ESP32";
#define DEFAULT_SSID "DroneBridge ESP32"

volatile bool client_connected = false;
volatile int client_connected_num = 0;
char DEST_IP[15] = "192.168.2.2";
uint8_t DEFAULT_PWD[64] = "dronebridge";
uint8_t DEFAULT_CHANNEL = 6;
uint8_t SERIAL_PROTOCOL = 2;  // 1,2=MSP, 3,4,5=MAVLink/transparent
uint8_t DB_UART_PIN_TX = GPIO_NUM_17;
uint8_t DB_UART_PIN_RX = GPIO_NUM_16;
uint32_t DB_UART_BAUD_RATE = 115200;
uint16_t TRANSPARENT_BUF_SIZE = 64;
uint8_t LTM_FRAME_NUM_BUFFER = 1;
uint8_t MSP_LTM_TO_SAME_PORT = 0;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_AP_START:
            ESP_LOGI(TAG, "Wifi AP started!");
            xEventGroupSetBits(wifi_event_group, BIT2);
            break;
        case SYSTEM_EVENT_AP_STOP:
            ESP_LOGI(TAG, "Wifi AP stopped!");
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "Client connected - station:"MACSTR", AID=%d",
                     MAC2STR(event->event_info.sta_connected.mac), event->event_info.sta_connected.aid);
            client_connected_num++;
            if (client_connected_num>0) client_connected = true;
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "Client disconnected - station:"MACSTR", AID=%d",
                     MAC2STR(event->event_info.sta_disconnected.mac), event->event_info.sta_disconnected.aid);
            client_connected_num--;
            if (client_connected_num<1) client_connected = false;
            break;
        default:
            break;
    }
    return ESP_OK;
}

void start_mdns_service()
{
    xEventGroupWaitBits(wifi_event_group, BIT2, false, true, portMAX_DELAY);
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return;
    }
    mdns_hostname_set("dronebridge");
    mdns_instance_name_set("DroneBridge for ESP32");

    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    mdns_service_add(NULL, "_db_telem", "_udp", 1604, NULL, 0);
    mdns_service_add(NULL, "_db_proxy", "_udp", 1607, NULL, 0);
    mdns_service_instance_name_set("_http", "DroneBridge telemetry downlink", "DroneBridge bi-directional link");
}


void init_wifi(){
    wifi_event_group = xEventGroupCreate();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t ap_config = {
            .ap = {
                    .ssid = DEFAULT_SSID,
                    .ssid_len = 0,
                    .authmode = WIFI_AUTH_WPA_PSK,
                    .channel = DEFAULT_CHANNEL,
                    .ssid_hidden = 0,
                    .beacon_interval = 150,
                    .max_connection = 2
            },
    };
    xthal_memcpy(ap_config.ap.password, DEFAULT_PWD, 64);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_AP, WIFI_PROTOCOL_11B));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    wifi_country_t wifi_country = {.cc = "CN", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_AP, "esp32"));
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
    tcpip_adapter_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192,168,2,1);
    IP4_ADDR(&ip_info.gw, 192,168,2,1);
    IP4_ADDR(&ip_info.netmask, 255,255,255,0);
    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info));
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
}


void read_settings_nvs(){
    nvs_handle my_handle;
    if (nvs_open("settings", NVS_READONLY, &my_handle) == ESP_ERR_NVS_NOT_FOUND){
        // First start
        nvs_close(my_handle);
        write_settings_to_nvs();
    } else {
        ESP_LOGI(TAG, "Reading settings from NVS");
        size_t required_size = 0;
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "wifi_pass", NULL, &required_size));
        char* wifi_pass = malloc(required_size);
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "wifi_pass", wifi_pass, &required_size));
        memcpy(DEFAULT_PWD, wifi_pass, required_size);
        ESP_ERROR_CHECK(nvs_get_u32(my_handle, "baud", &DB_UART_BAUD_RATE));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "gpio_tx", &DB_UART_PIN_TX));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "gpio_rx", &DB_UART_PIN_RX));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "proto", &SERIAL_PROTOCOL));
        ESP_ERROR_CHECK(nvs_get_u16(my_handle, "trans_pack_size", &TRANSPARENT_BUF_SIZE));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "ltm_per_packet", &LTM_FRAME_NUM_BUFFER));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "msp_ltm_same", &MSP_LTM_TO_SAME_PORT));
        nvs_close(my_handle);
        free(wifi_pass);
    }
}


void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    read_settings_nvs();
    esp_log_level_set("*", ESP_LOG_WARN);
    init_wifi();
    start_mdns_service();
    control_module();
    start_tcp_server();
    communication_module();
}