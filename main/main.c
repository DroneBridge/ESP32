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
#include <esp_now.h>

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
#include "db_esp_now.h"
#include "iot_button.h"

#define NVS_NAMESPACE "settings"

static const char *TAG = "DB_ESP32";

uint8_t DB_WIFI_MODE = DB_WIFI_MODE_AP; // 1=Wifi AP mode, 2=Wifi client mode, 3=ESP-NOW LR Mode
uint8_t DB_WIFI_SSID[32] = "DroneBridge ESP32";
uint8_t DB_WIFI_PWD[64] = "dronebridge";
char DEFAULT_AP_IP[IP4ADDR_STRLEN_MAX] = "192.168.2.1";
char DB_STATIC_STA_IP[IP4ADDR_STRLEN_MAX] = "";
char DB_STATIC_STA_IP_GW[IP4ADDR_STRLEN_MAX] = "";
char DB_STATIC_STA_IP_NETMASK[IP4ADDR_STRLEN_MAX] = "";
char CURRENT_CLIENT_IP[IP4ADDR_STRLEN_MAX] = "192.168.2.1";
uint8_t DB_WIFI_CHANNEL = 6;
uint8_t DB_SERIAL_PROTOCOL = 4;  // 1=MSP, 4=MAVLink, 5=Transparent

// initially set pins to 0 to allow the start of the system on all boards. User has to set the correct pins
uint8_t DB_UART_PIN_TX = GPIO_NUM_0;
uint8_t DB_UART_PIN_RX = GPIO_NUM_0;
uint8_t DB_UART_PIN_RTS = GPIO_NUM_0;
uint8_t DB_UART_PIN_CTS = GPIO_NUM_0;
uint8_t DB_UART_RTS_THRESH = 64;

/* used to reset ESP to defaults and force restart or to reset the mode to access point mode */
#ifdef CONFIG_IDF_TARGET_ESP32C3
    #define DB_RESET_PIN GPIO_NUM_9
#elif CONFIG_IDF_TARGET_ESP32S2
    #define DB_RESET_PIN GPIO_NUM_0
#elif CONFIG_IDF_TARGET_ESP32S3
    #define DB_RESET_PIN GPIO_NUM_0
#elif CONFIG_IDF_TARGET_ESP32
    #define DB_RESET_PIN GPIO_NUM_0
#else
    #define DB_RESET_PIN GPIO_NUM_0
#endif

int32_t DB_UART_BAUD_RATE = 57600;
uint16_t DB_TRANS_BUF_SIZE = 64;
uint8_t DB_LTM_FRAME_NUM_BUFFER = 2;
int station_rssi = 0;
uint8_t LOCAL_MAC_ADDRESS[6];

udp_conn_list_t *udp_conn_list;

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
        ESP_LOGI(TAG, "WIFI_EVENT - AP started! (SSID: %s PASS: %s)", DB_WIFI_SSID, DB_WIFI_PWD);
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
                    .channel = DB_WIFI_CHANNEL,
                    .ssid_hidden = 0,
                    .beacon_interval = 100,
                    .max_connection = 10
            },
    };
    strncpy((char *)wifi_config.ap.ssid, (char *)DB_WIFI_SSID, 32);
    strncpy((char *)wifi_config.ap.password, (char *)DB_WIFI_PWD, 64);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    if (wifi_mode == DB_WIFI_MODE_AP_LR) {
        ESP_LOGI(TAG, "Enabling LR Mode on access point. This device will be invisible to non-ESP32 devices!");
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_LR));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B));
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    wifi_country_t wifi_country = {.cc = "US", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Assign IP to ap/gateway */
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
    ESP_ERROR_CHECK(esp_read_mac(LOCAL_MAC_ADDRESS, ESP_MAC_WIFI_SOFTAP));
}

/**
 * Initializes the ESP Wifi client mode where we connect to a known access point.
 */
int init_wifi_clientmode() {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_default_netif = esp_netif_create_default_wifi_sta();

    /* Assign static IP to esp32 in client mode if specified */
    if (strlen(DB_STATIC_STA_IP) > 0 && strlen(DB_STATIC_STA_IP_GW) > 0 && strlen(DB_STATIC_STA_IP_NETMASK) > 0) {
        ESP_LOGI(TAG, "Assigning static IP to ESP32: ESP32-IP: %s Gateway: %s Netmask: %s", DB_STATIC_STA_IP,
                 DB_STATIC_STA_IP_GW, DB_STATIC_STA_IP_NETMASK);
        esp_netif_ip_info_t ip;
        memset(&ip, 0, sizeof(esp_netif_ip_info_t));
        ip.ip.addr = ipaddr_addr(DB_STATIC_STA_IP);
        ip.netmask.addr = ipaddr_addr(DB_STATIC_STA_IP_NETMASK);
        ip.gw.addr = ipaddr_addr(DB_STATIC_STA_IP_GW);
        ESP_ERROR_CHECK(esp_netif_dhcpc_stop(esp_default_netif));
        ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_default_netif, &ip));
        ESP_ERROR_CHECK(esp_netif_dhcpc_start(esp_default_netif));
    } else {
        //no static IP specified let the DHCP assign us one
    }

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
    strncpy((char *)wifi_config.sta.ssid, (char *)DB_WIFI_SSID, 32);
    strncpy((char *)wifi_config.sta.password, (char *)DB_WIFI_PWD, 64);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Init of WiFi Client-Mode finished. (SSID: %s PASS: %s)", DB_WIFI_SSID, DB_WIFI_PWD);

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
        ESP_LOGI(TAG, "Connected to ap SSID:%s password:%s", DB_WIFI_SSID, DB_WIFI_PWD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "Failed to connect to SSID:%s, password:%s", DB_WIFI_SSID, DB_WIFI_PWD);
        enable_temp_ap_mode = true;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED WIFI EVENT");
    }
    if (enable_temp_ap_mode) {
        ESP_LOGW(TAG, "WiFi client mode was not able to connect to the specified access point");
        return -1;
    }

    ESP_LOGI(TAG, "WiFi client mode enabled and connected!");
    ESP_ERROR_CHECK(esp_read_mac(LOCAL_MAC_ADDRESS, ESP_MAC_WIFI_STA));
    return 0;
}

/**
 * Initialize WiFi for ESP-NOW mode.
 * If someone uses ESP-NOW over WiFi it is because he wants range over everything else.
 * LR mode makes it very inconvenient to change settings but gives the most range. No AP mode since AP will not be
 * visible.
 */
void init_wifi_espnow() {
    ESP_LOGI(TAG, "Setting up Wi-Fi for ESP-NOW");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(DB_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_LR) );
    ESP_LOGI(TAG, "Enabled ESP-NOW WiFi Mode! LR Mode is set. This device will be invisible to non-ESP32 devices!");
    ESP_ERROR_CHECK(esp_read_mac(LOCAL_MAC_ADDRESS, ESP_MAC_WIFI_STA));
}

/**
 * Write settings to non-volatile storage
 */
void write_settings_to_nvs() {
    ESP_LOGI(TAG,
             "Trying to save:\nWifi Mode: %i\nssid %s\nwifi_pass %s\nwifi_chan %i\nbaud %liu\ngpio_tx %i\ngpio_rx %i\ngpio_cts %i\ngpio_rts %i\nrts_thresh %i\nproto %i\n"
             "trans_pack_size %i\nltm_per_packet %i\nap_ip %s\nip_sta %s\nip_sta_gw %s\nip_sta_netmsk %s",
             DB_WIFI_MODE, DB_WIFI_SSID, DB_WIFI_PWD, DB_WIFI_CHANNEL, DB_UART_BAUD_RATE, DB_UART_PIN_TX, DB_UART_PIN_RX,
             DB_UART_PIN_CTS, DB_UART_PIN_RTS, DB_UART_RTS_THRESH,
             DB_SERIAL_PROTOCOL, DB_TRANS_BUF_SIZE, DB_LTM_FRAME_NUM_BUFFER,
             DEFAULT_AP_IP, DB_STATIC_STA_IP, DB_STATIC_STA_IP_GW, DB_STATIC_STA_IP_NETMASK);
    ESP_LOGI(TAG, "Saving to NVS %s", NVS_NAMESPACE);
    nvs_handle my_handle;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "esp32_mode", DB_WIFI_MODE));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ssid", (char *) DB_WIFI_SSID));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "wifi_pass", (char *) DB_WIFI_PWD));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "wifi_chan", DB_WIFI_CHANNEL));
    ESP_ERROR_CHECK(nvs_set_i32(my_handle, "baud", DB_UART_BAUD_RATE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_tx", DB_UART_PIN_TX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_rx", DB_UART_PIN_RX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_cts", DB_UART_PIN_CTS));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_rts", DB_UART_PIN_RTS));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "rts_thresh", DB_UART_RTS_THRESH));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "proto", DB_SERIAL_PROTOCOL));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "trans_pack_size", DB_TRANS_BUF_SIZE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "ltm_per_packet", DB_LTM_FRAME_NUM_BUFFER));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ap_ip", DEFAULT_AP_IP));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ip_sta", DB_STATIC_STA_IP));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ip_sta_gw", DB_STATIC_STA_IP_GW));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ip_sta_netmsk", DB_STATIC_STA_IP_NETMASK));

    ESP_ERROR_CHECK(nvs_commit(my_handle));
    nvs_close(my_handle);
}

/**
 * Helper function to read a string from the NVS based on a key. Handles errors accordingly and print result to console
 *
 * @param my_handle nvs_handle to use
 * @param key NVS key as string with max length NVS_KEY_NAME_MAX_SIZE-1
 * @param dst Destination for the read value
 */
void db_read_str_nvs(nvs_handle my_handle, char *key, char *dst) {
    if (strlen(key)+1 > NVS_KEY_NAME_MAX_SIZE) ESP_LOGW(TAG, "key %s is longer than %i bytes", key, NVS_KEY_NAME_MAX_SIZE);
    size_t required_size = 0;
    esp_err_t err = nvs_get_str(my_handle, key, NULL, &required_size);
    if (err == ESP_OK) {
        char *read_nvs_val = malloc(required_size);
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_str(my_handle, key, read_nvs_val, &required_size));
        memcpy(dst, read_nvs_val, required_size);
        free(read_nvs_val);
        ESP_LOGI(TAG, "\t%s: %s", key, dst);
    } else {
        ESP_LOGW(TAG, "Could not read key %s from NVS", key);
    }
}

/**
 * Read stored settings from internal storage
 */
void read_settings_nvs() {
    nvs_handle my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK) {
        ESP_LOGI(TAG, "NVS namespace not found. Erasing flash, init NVS ...");
        nvs_close(my_handle);
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        write_settings_to_nvs();
    } else {
        ESP_LOGI(TAG, "Reading settings from NVS");

        db_read_str_nvs(my_handle, "ssid", (char *) DB_WIFI_SSID);
        db_read_str_nvs(my_handle, "wifi_pass", (char *) DB_WIFI_PWD);
        db_read_str_nvs(my_handle, "ap_ip", DEFAULT_AP_IP);
        db_read_str_nvs(my_handle, "ip_sta", DB_STATIC_STA_IP);
        db_read_str_nvs(my_handle, "ip_sta_gw", DB_STATIC_STA_IP_GW);
        db_read_str_nvs(my_handle, "ip_sta_netmsk", DB_STATIC_STA_IP_NETMASK);
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "esp32_mode", &DB_WIFI_MODE));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "wifi_chan", &DB_WIFI_CHANNEL));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_i32(my_handle, "baud", &DB_UART_BAUD_RATE));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "gpio_tx", &DB_UART_PIN_TX));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "gpio_rx", &DB_UART_PIN_RX));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "gpio_cts", &DB_UART_PIN_CTS));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "gpio_rts", &DB_UART_PIN_RTS));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "rts_thresh", &DB_UART_RTS_THRESH));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "proto", &DB_SERIAL_PROTOCOL));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u16(my_handle, "trans_pack_size", &DB_TRANS_BUF_SIZE));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "ltm_per_packet", &DB_LTM_FRAME_NUM_BUFFER));

        nvs_close(my_handle);
        ESP_LOGI(TAG,
                 "\tWifi Mode: %i\n\twifi_chan %i\n\tbaud %liu\n\tgpio_tx %i\n\tgpio_rx %i\n\tgpio_cts %i\n\t"
                 "gpio_rts %i\n\trts_thresh %i\n\tproto %i\n\ttrans_pack_size %i\n\tltm_per_packet %i",
                 DB_WIFI_MODE, DB_WIFI_CHANNEL, DB_UART_BAUD_RATE, DB_UART_PIN_TX, DB_UART_PIN_RX,
                 DB_UART_PIN_CTS, DB_UART_PIN_RTS, DB_UART_RTS_THRESH, DB_SERIAL_PROTOCOL, DB_TRANS_BUF_SIZE,
                 DB_LTM_FRAME_NUM_BUFFER);
    }
}

/**
 * Callback for a short press (<CONFIG_BUTTON_SHORT_PRESS_TIME_MS) of the reset/boot button.
 * Sets mode to WiFi access point mode with default password "dronebridge" so user can check/change the config
 * @param arg
 */
void short_press_callback(void *arg,void *usr_data) {
    ESP_LOGW(TAG, "Short press detected setting wifi mode to access point with password: dronebridge");
    DB_WIFI_MODE = DB_WIFI_MODE_AP;
    strncpy((char *) DB_WIFI_PWD, "dronebridge", sizeof(DB_WIFI_PWD) - 1);
    write_settings_to_nvs();
    esp_restart();
}

/**
 * Callback for a long press (>CONFIG_BUTTON_LONG_PRESS_TIME_MS) of the reset/boot button.
 * Resets all settings to defaults.
 * @param arg
 */
void long_press_callback(void *arg,void *usr_data) {
    ESP_LOGW(TAG, "Reset triggered via GPIO %i. Resetting settings and rebooting", DB_RESET_PIN);
    DB_WIFI_MODE = DB_WIFI_MODE_AP;
    strncpy((char *) DB_WIFI_SSID, "DroneBridge for ESP32", sizeof(DB_WIFI_SSID) - 1);
    strncpy((char *) DB_WIFI_PWD, "dronebridge", sizeof(DB_WIFI_PWD) - 1);
    strncpy(DEFAULT_AP_IP, "192.168.2.1", sizeof(DEFAULT_AP_IP) - 1);
    memset(DB_STATIC_STA_IP, 0, strlen(DB_STATIC_STA_IP));
    memset(DB_STATIC_STA_IP_GW, 0, strlen(DB_STATIC_STA_IP_GW));
    memset(DB_STATIC_STA_IP_NETMASK, 0, strlen(DB_STATIC_STA_IP_NETMASK));
    DB_WIFI_CHANNEL = 6;
    DB_UART_PIN_TX = GPIO_NUM_0;
    DB_UART_PIN_RX = GPIO_NUM_0;
    DB_UART_PIN_CTS = GPIO_NUM_0;
    DB_UART_PIN_RTS = GPIO_NUM_0;
    DB_SERIAL_PROTOCOL = 4;
    DB_TRANS_BUF_SIZE = 64;
    DB_UART_RTS_THRESH = 64;
    write_settings_to_nvs();
    esp_restart();
}

/**
 * Setup boot button GPIO to reset entire ESP32 settings and to force a reboot of the system
 */
void set_reset_trigger() {
    button_config_t gpio_btn_cfg = {
            .type = BUTTON_TYPE_GPIO,
            .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
            .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
            .gpio_button_config = {
                    .gpio_num = DB_RESET_PIN,
                    .active_level = 0,
            },
    };
    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if(NULL == gpio_btn) {
        ESP_LOGE(TAG, "Creating reset button failed");
    } else {
        iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, short_press_callback,NULL);
        iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_UP, long_press_callback,NULL);
    }
}

/**
 * Main entry point.
 * Client Mode: ESP32 connects to a known access point.
 * If the ESP32 could not connect to the specified access point using WIFI_ESP_MAXIMUM_RETRY retries, the
 * ESP32 will switch temporarily to access point mode to allow the user to check the configuration. On reboot of the
 * ESP32 the WiFi client mode will be re-enabled if not changed otherwise by the user.
 *
 * AP-Mode: ESP32 creates an WiFi access point of its own where the ground control stations can connect
 */
void app_main() {
    udp_conn_list = udp_client_list_create();   // http server functions expect the list to exist
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
    } else if (DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_AIR || DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_GND) {
        init_wifi_espnow();
        db_espnow_module();
    } else {
        // Wi-Fi client mode with LR mode enabled
        if (init_wifi_clientmode() < 0) {
            ESP_LOGW(TAG, "Switching to failsafe: Enabling access point mode");
            // De-Init all Wi-Fi and enable the AP-Mode temporarily
            ESP_ERROR_CHECK(esp_event_loop_delete_default());
            esp_netif_destroy_default_wifi(esp_default_netif);
            ESP_ERROR_CHECK(esp_wifi_stop());
            strncpy((char *) DB_WIFI_SSID, "Failsafe DroneBridge ESP32", sizeof(DB_WIFI_SSID));
            strncpy((char *) DB_WIFI_PWD, "dronebridge", sizeof(DB_WIFI_PWD));
            init_wifi_apmode(DB_WIFI_MODE_AP);
        }
    }

    if (DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_AIR && DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_GND) {
        // no need to start these services - won`t be available anyway - safe the resources
        start_mdns_service();
        netbiosns_init();
        netbiosns_set_name("dronebridge");
    }

    ESP_ERROR_CHECK(init_fs());
    control_module();

    if (DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_AIR && DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_GND) {
        // no need to start these services - won`t be available anyway - safe the resources
        ESP_ERROR_CHECK(start_rest_server(CONFIG_WEB_MOUNT_POINT));
        communication_module();
    }
    set_reset_trigger();
}
