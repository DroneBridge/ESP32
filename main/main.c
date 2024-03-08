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

#include <stdio.h>
#include <nvs_flash.h>
#include <esp_wifi_types.h>
#include <string.h>
#include <driver/gpio.h>
#include <lwip/apps/netbiosns.h>

#include "freertos/event_groups.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_log.h"
#include "esp_event.h"
#include "db_esp32_control.h"
#include "http_server.h"
#include "db_esp32_comm.h"
#include "db_protocol.h"
#include "esp_vfs_semihost.h"
#include "esp_spiffs.h"
#include "http_server.h"
#include "main.h"
#include "mdns.h"

#define NVS_NAMESPACE "settings"

static const char *TAG = "DB_ESP32";

uint8_t DB_WIFI_MODE = DB_WIFI_MODE_AP; // 1=Wifi AP mode, 2=Wifi client mode, 3=ESP-NOW LR Mode
uint8_t DEFAULT_SSID[32] = "DroneBridge ESP32";
uint8_t DEFAULT_PWD[64] = "dronebridge";
char DEFAULT_AP_IP[32] = "192.168.2.1";
char CURRENT_CLIENT_IP[32] = "192.168.2.1";
uint8_t DEFAULT_CHANNEL = 6;
uint8_t SERIAL_PROTOCOL = 4;  // 1=MSP, 4=MAVLink/transparent

// initially set pins to 0 to allow the start of the system on all boards. User has to set the correct pins
uint8_t DB_UART_PIN_TX = GPIO_NUM_0;
uint8_t DB_UART_PIN_RX = GPIO_NUM_0;
uint8_t DB_UART_PIN_RTS = GPIO_NUM_0;
uint8_t DB_UART_PIN_CTS = GPIO_NUM_0;
uint8_t DB_UART_RTS_THRESH = 64;

int32_t DB_UART_BAUD_RATE = 57600;
uint16_t TRANSPARENT_BUF_SIZE = 64;
uint8_t LTM_FRAME_NUM_BUFFER = 2;
uint8_t MSP_LTM_SAMEPORT = 0;
int station_rssi = 0;

struct udp_conn_list_t *udp_conn_list;

// Wifi client mode vars
int WIFI_ESP_MAXIMUM_RETRY = 25;          // max number of retries to connect to the ap before enabling temp. ap mode
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

esp_netif_t *esp_default_netif;

/**
 * Devices get added based on IP (check if IP & PORT are already listed) and removed from the UDP broadcast connection
 * based on MAC address
 *
 * @param arg
 * @param event_base
 * @param event_id
 * @param event_data
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    // Wifi access point mode events
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG, "WIFI_EVENT - Client connected - station:"MACSTR", AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG, "WIFI_EVENT - Client disconnected - station:"MACSTR", AID=%d", MAC2STR(event->mac), event->aid);
        struct db_udp_client_t db_udp_client;
        memcpy(db_udp_client.mac, event->mac, sizeof(db_udp_client.mac));
        remove_from_known_udp_clients(udp_conn_list, db_udp_client);
    } else if (event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "WIFI_EVENT - AP started! (SSID: %s PASS: %s)", DEFAULT_SSID, DEFAULT_PWD);
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        ESP_LOGI(TAG, "WIFI_EVENT - AP stopped!");
    } else if(event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED){
        ip_event_ap_staipassigned_t* event = (ip_event_ap_staipassigned_t*) event_data;
        ESP_LOGI(TAG, "WIFI_EVENT - New station IP:" IPSTR, IP2STR(&event->ip));
        ESP_LOGI(TAG, "WIFI_EVENT - MAC: " MACSTR, MAC2STR(event->mac));
        struct db_udp_client_t db_udp_client;
        db_udp_client.udp_client.sin_family = PF_INET;
        db_udp_client.udp_client.sin_port = htons(APP_PORT_PROXY_UDP);
        db_udp_client.udp_client.sin_len = 16;
        db_udp_client.udp_client.sin_addr.s_addr = event->ip.addr;
        memcpy(db_udp_client.mac, event->mac, sizeof(db_udp_client.mac));
        add_to_known_udp_clients(udp_conn_list, db_udp_client);
    }
    // Wifi client mode events
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT - Wifi Started");
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WIFI_EVENT - Lost connection to access point");
        if (s_retry_num < WIFI_ESP_MAXIMUM_RETRY) {
            ESP_ERROR_CHECK(esp_wifi_connect());
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP (%i/%i)", s_retry_num, WIFI_ESP_MAXIMUM_RETRY);
        } else {
            ESP_LOGI(TAG,"Connecting to the AP failed");
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WIFI_EVENT - Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        sprintf(CURRENT_CLIENT_IP, IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void start_mdns_service() {
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return;
    }
    ESP_ERROR_CHECK(mdns_hostname_set("dronebridge"));
    ESP_ERROR_CHECK(mdns_instance_name_set("DroneBridge for ESP32"));

    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_db_proxy", "_tcp", APP_PORT_PROXY, NULL, 0));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_db_comm", "_tcp", APP_PORT_COMM, NULL, 0));
    ESP_ERROR_CHECK(mdns_service_instance_name_set("_http", "_tcp", "DroneBridge for ESP32"));
    ESP_LOGI(TAG, "MDNS Service started!");
}

#if CONFIG_WEB_DEPLOY_SEMIHOST
esp_err_t init_fs(void) {
    esp_err_t ret = esp_vfs_semihost_register(CONFIG_WEB_MOUNT_POINT, CONFIG_HOST_PATH_TO_MOUNT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register semihost driver (%s)!", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    return ESP_OK;
}
#endif


#if CONFIG_WEB_DEPLOY_SF

esp_err_t init_fs(void) {
    esp_vfs_spiffs_conf_t conf = {
            .base_path = CONFIG_WEB_MOUNT_POINT,
            .partition_label = NULL,
            .max_files = 5,
            .format_if_mount_failed = false
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Filesystem init finished! Partition size: total: %d bytes, used: %d bytes (%i%%)", total, used, (used*100)/total);
    }
    return ret;
}

#endif

/**
 * Launches an access point where ground stations can connect to
 *
 * @param wifi_mode Allowes to overwrite an AP mode from traditional WiFi to LR Mode
 */
void init_wifi_apmode(int wifi_mode) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_default_netif = esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    esp_event_handler_instance_t ap_staipassigned_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_AP_STAIPASSIGNED,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &ap_staipassigned_ip));

    wifi_config_t wifi_config = {
            .ap = {
                    .ssid = "DroneBridge_ESP32_Init",
                    .ssid_len = 0,
                    .authmode = WIFI_AUTH_WPA2_PSK,
                    .channel = DEFAULT_CHANNEL,
                    .ssid_hidden = 0,
                    .beacon_interval = 100,
                    .max_connection = 10
            },
    };
    strncpy((char *)wifi_config.ap.ssid, (char *)DEFAULT_SSID, 32);
    strncpy((char *)wifi_config.ap.password, (char *)DEFAULT_PWD, 64);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    if (wifi_mode == DB_WIFI_MODE_AP_LR) {
        ESP_LOGI(TAG, "Enabling LR Mode on access point (ESP-NOW)");
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B));
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    wifi_country_t wifi_country = {.cc = "US", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(DEFAULT_AP_IP);
    ip.netmask.addr = ipaddr_addr("255.255.255.0");
    ip.gw.addr = ipaddr_addr(DEFAULT_AP_IP);
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(esp_default_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_default_netif, &ip));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_default_netif));

    ESP_ERROR_CHECK(esp_netif_set_hostname(esp_default_netif, "DBESP32"));
    strncpy(CURRENT_CLIENT_IP, DEFAULT_AP_IP, sizeof(CURRENT_CLIENT_IP));
}

/**
 * Initializes the ESP Wifi client mode where we connect to a known access point.
 */
int init_wifi_clientmode() {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_default_netif = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
            .sta = {
                    .ssid = "DroneBridge_ESP32_Init",
                    .password = "dronebridge",
                    .threshold.authmode = WIFI_AUTH_WEP
            },
    };
    strncpy((char *)wifi_config.sta.ssid, (char *)DEFAULT_SSID, 32);
    strncpy((char *)wifi_config.sta.password, (char *)DEFAULT_PWD, 64);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Init of WiFi Client-Mode finished. (SSID: %s PASS: %s)", DEFAULT_SSID, DEFAULT_PWD);

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    bool enable_temp_ap_mode = false;
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to ap SSID:%s password:%s", DEFAULT_SSID, DEFAULT_PWD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "Failed to connect to SSID:%s, password:%s", DEFAULT_SSID, DEFAULT_PWD);
        enable_temp_ap_mode = true;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED WIFI EVENT");
    }
    if (enable_temp_ap_mode) {
        ESP_LOGW(TAG, "WiFi client mode was not able to connect to the specified access point");
        return -1;
    }
    ESP_LOGI(TAG, "WiFi client mode enabled and connected!");
    return 0;
}

/**
 * Write settings to non-volatile storage
 */
void write_settings_to_nvs() {
    ESP_LOGI(TAG,
             "Trying to save:\nWifi Mode: %i\nssid %s\nwifi_pass %s\nwifi_chan %i\nbaud %liu\ngpio_tx %i\ngpio_rx %i\ngpio_cts %i\ngpio_rts %i\nrts_thresh %i\nproto %i\n"
             "trans_pack_size %i\nltm_per_packet %i\nmsp_ltm %i\nap_ip %s",
             DB_WIFI_MODE, DEFAULT_SSID, DEFAULT_PWD, DEFAULT_CHANNEL, DB_UART_BAUD_RATE, DB_UART_PIN_TX, DB_UART_PIN_RX,
             DB_UART_PIN_CTS, DB_UART_PIN_RTS, DB_UART_RTS_THRESH,
             SERIAL_PROTOCOL, TRANSPARENT_BUF_SIZE, LTM_FRAME_NUM_BUFFER, MSP_LTM_SAMEPORT, DEFAULT_AP_IP);
    ESP_LOGI(TAG, "Saving to NVS %s", NVS_NAMESPACE);
    nvs_handle my_handle;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "esp32_mode", DB_WIFI_MODE));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ssid", (char *) DEFAULT_SSID));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "wifi_pass", (char *) DEFAULT_PWD));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "wifi_chan", DEFAULT_CHANNEL));
    ESP_ERROR_CHECK(nvs_set_i32(my_handle, "baud", DB_UART_BAUD_RATE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_tx", DB_UART_PIN_TX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_rx", DB_UART_PIN_RX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_cts", DB_UART_PIN_CTS));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_rts", DB_UART_PIN_RTS));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "rts_thresh", DB_UART_RTS_THRESH));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "proto", SERIAL_PROTOCOL));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "trans_pack_size", TRANSPARENT_BUF_SIZE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "ltm_per_packet", LTM_FRAME_NUM_BUFFER));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "msp_ltm", MSP_LTM_SAMEPORT));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ap_ip", DEFAULT_AP_IP));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    nvs_close(my_handle);
}

/**
 * Read stored settings from internal storage
 */
void read_settings_nvs() {
    nvs_handle my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK) {
        // First start
        nvs_close(my_handle);
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        write_settings_to_nvs();
    } else {
        ESP_LOGI(TAG, "Reading settings from NVS");
        size_t required_size = 0;
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "esp32_mode", &DB_WIFI_MODE));

        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ssid", NULL, &required_size));
        char *ssid = malloc(required_size);
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ssid", ssid, &required_size));
        memcpy(DEFAULT_SSID, ssid, required_size);

        ESP_ERROR_CHECK(nvs_get_str(my_handle, "wifi_pass", NULL, &required_size));
        char *wifi_pass = malloc(required_size);
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "wifi_pass", wifi_pass, &required_size));
        memcpy(DEFAULT_PWD, wifi_pass, required_size);

        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ap_ip", NULL, &required_size));
        char *ap_ip = malloc(required_size);
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ap_ip", ap_ip, &required_size));
        memcpy(DEFAULT_AP_IP, ap_ip, required_size);

        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "wifi_chan", &DEFAULT_CHANNEL));
        ESP_ERROR_CHECK(nvs_get_i32(my_handle, "baud", &DB_UART_BAUD_RATE));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "gpio_tx", &DB_UART_PIN_TX));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "gpio_rx", &DB_UART_PIN_RX));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "gpio_cts", &DB_UART_PIN_CTS));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "gpio_rts", &DB_UART_PIN_RTS));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "rts_thresh", &DB_UART_RTS_THRESH));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "proto", &SERIAL_PROTOCOL));
        ESP_ERROR_CHECK(nvs_get_u16(my_handle, "trans_pack_size", &TRANSPARENT_BUF_SIZE));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "ltm_per_packet", &LTM_FRAME_NUM_BUFFER));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "msp_ltm", &MSP_LTM_SAMEPORT));
        nvs_close(my_handle);
        free(wifi_pass);
        free(ssid);
        free(ap_ip);
        ESP_LOGI(TAG,
                 "Stored settings:\nWifi Mode: %i\nssid %s\nwifi_pass %s\nwifi_chan %i\nbaud %liu\ngpio_tx %i\ngpio_rx %i\ngpio_cts %i\n"
                 "gpio_rts %i\nrts_thresh %i\nproto %i\ntrans_pack_size %i\nltm_per_packet %i\nmsp_ltm %i\nap_ip %s",
                 DB_WIFI_MODE, DEFAULT_SSID, DEFAULT_PWD, DEFAULT_CHANNEL, DB_UART_BAUD_RATE, DB_UART_PIN_TX, DB_UART_PIN_RX,
                 DB_UART_PIN_CTS, DB_UART_PIN_RTS, DB_UART_RTS_THRESH, SERIAL_PROTOCOL, TRANSPARENT_BUF_SIZE,
                 LTM_FRAME_NUM_BUFFER, MSP_LTM_SAMEPORT, DEFAULT_AP_IP);
    }
}

/**
 * Client Mode: ESP32 connects to a known access point.
 *  If the ESP32 could not connect to the specified access point using WIFI_ESP_MAXIMUM_RETRY retries, the
 *  ESP32 will switch temporarily to access point mode to allow the user to check the configuration. On reboot of the
 *  ESP32 the WiFi client mode will be re-enabled if not changed otherwise by the user.
 *
 *  AP-Mode: ESP32 creates an WiFi access point of its own where the ground control stations can connect
 */
void app_main() {
    udp_conn_list = udp_client_list_create();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    read_settings_nvs();
    esp_log_level_set("*", ESP_LOG_INFO);
    if (DB_WIFI_MODE == DB_WIFI_MODE_AP || DB_WIFI_MODE == DB_WIFI_MODE_AP_LR) {
        init_wifi_apmode(DB_WIFI_MODE);
    } else {
        if (init_wifi_clientmode() < 0) {
            ESP_LOGW(TAG, "Switching to failsafe: Enabling access point mode");
            // De-Init all Wi-Fi and enable the AP-Mode temporarily
            ESP_ERROR_CHECK(esp_event_loop_delete_default());
            esp_netif_destroy_default_wifi(esp_default_netif);
            ESP_ERROR_CHECK(esp_wifi_stop());
            strncpy((char *) DEFAULT_SSID, "Failsafe DroneBridge ESP32", sizeof(DEFAULT_SSID));
            strncpy((char *) DEFAULT_PWD, "dronebridge", sizeof(DEFAULT_PWD));
            init_wifi_apmode(DB_WIFI_MODE_AP);
        }
    }

    start_mdns_service();
    netbiosns_init();
    netbiosns_set_name("dronebridge");

    ESP_ERROR_CHECK(init_fs());
    control_module();
    ESP_ERROR_CHECK(start_rest_server(CONFIG_WEB_MOUNT_POINT));
    communication_module();
}