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
#include "db_esp_now.h"
#include "iot_button.h"
#include "db_serial.h"
#include "globals.h"

#define NVS_NAMESPACE "settings"

/* used to reset ESP to defaults and force restart or to reset the mode to access point mode */
#ifdef CONFIG_IDF_TARGET_ESP32C3
#define DB_RESET_PIN GPIO_NUM_9
#elif CONFIG_IDF_TARGET_ESP32C6
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

#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_5
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_4
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_6
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_7
#define DB_DEFAULT_UART_BAUD_RATE 115200
#elif CONFIG_DB_OFFICIAL_BOARD_1_X_C6
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_21
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_2
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_22
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_23
#define DB_DEFAULT_UART_BAUD_RATE 115200
#elif CONFIG_DB_GENERIC_BOARD
// initially set pins to 0 to allow the start of the system on all boards. User has to set the correct pins
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_BAUD_RATE 57600
#else
// someone fucked up the config - fallback to generic config
#define DB_DEFAULT_UART_TX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RX_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_RTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_CTS_PIN GPIO_NUM_0
#define DB_DEFAULT_UART_BAUD_RATE 57600
#endif

static const char *TAG = "DB_ESP32";

/* DroneBridge Parameters */
uint8_t DB_WIFI_MODE = DB_WIFI_MODE_AP;
uint8_t DB_WIFI_MODE_DESIGNATED = DB_WIFI_MODE_AP;  // initially assign the same value as DB_WIFI_MODE
uint8_t DB_WIFI_SSID[32] = "DroneBridge for ESP32";
uint8_t DB_WIFI_PWD[64] = "dronebridge";
char DEFAULT_AP_IP[IP4ADDR_STRLEN_MAX] = "192.168.2.1";
char DB_STATIC_STA_IP[IP4ADDR_STRLEN_MAX] = "";
char DB_STATIC_STA_IP_GW[IP4ADDR_STRLEN_MAX] = "";
char DB_STATIC_STA_IP_NETMASK[IP4ADDR_STRLEN_MAX] = "";
char CURRENT_CLIENT_IP[IP4ADDR_STRLEN_MAX] = "192.168.2.1";
uint8_t DB_WIFI_CHANNEL = 6;
uint8_t DB_SERIAL_PROTOCOL = DB_SERIAL_PROTOCOL_MAVLINK;
uint8_t DB_DISABLE_WIFI_ARMED = false;
uint8_t DB_UART_PIN_TX = DB_DEFAULT_UART_TX_PIN;
uint8_t DB_UART_PIN_RX = DB_DEFAULT_UART_RX_PIN;
uint8_t DB_UART_PIN_RTS = DB_DEFAULT_UART_RTS_PIN;
uint8_t DB_UART_PIN_CTS = DB_DEFAULT_UART_CTS_PIN;
uint8_t DB_UART_RTS_THRESH = 64;
int32_t DB_UART_BAUD_RATE = DB_DEFAULT_UART_BAUD_RATE;
uint16_t DB_TRANS_BUF_SIZE = 128;
uint8_t DB_LTM_FRAME_NUM_BUFFER = 2;
uint8_t DB_EN_EXT_ANT = false;

uint8_t DB_WIFI_IS_OFF = false;  // keep track if we switched Wi-Fi off already
db_esp_signal_quality_t db_esp_signal_quality = {.air_rssi = -127, .air_noise_floor = -1, .gnd_rssi= -127, .gnd_noise_floor = -1};
wifi_sta_list_t wifi_sta_list = {.num = 0};
uint8_t LOCAL_MAC_ADDRESS[6];

udp_conn_list_t *udp_conn_list;

// Wifi client mode vars
// int WIFI_ESP_MAXIMUM_RETRY = 25;   // max number of retries to connect to the ap before enabling temp. ap mode
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

esp_netif_t *esp_default_netif;

static esp_err_t db_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type) {
    if (addr && (addr != IPADDR_NONE)) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

/**
 * Assigns static IP to ESP32 when in client mode and static IP, GW and netmask are set in config.
 * Stops client DHCP server
 */
static void set_client_static_ip() {
    if (DB_WIFI_MODE == DB_WIFI_MODE_STA && strlen(DB_STATIC_STA_IP) > 0 && strlen(DB_STATIC_STA_IP_GW) > 0 &&
        strlen(DB_STATIC_STA_IP_NETMASK) > 0) {
        ESP_LOGI(TAG, "Assigning static IP to ESP32: ESP32-IP: %s Gateway: %s Netmask: %s", DB_STATIC_STA_IP,
                 DB_STATIC_STA_IP_GW, DB_STATIC_STA_IP_NETMASK);

        if (esp_netif_dhcpc_stop(esp_default_netif) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop dhcp client in order to set static IP");
            return;
        }
        esp_netif_ip_info_t ip;
        memset(&ip, 0, sizeof(esp_netif_ip_info_t));
        ip.ip.addr = ipaddr_addr(DB_STATIC_STA_IP);
        ip.netmask.addr = ipaddr_addr(DB_STATIC_STA_IP_NETMASK);
        ip.gw.addr = ipaddr_addr(DB_STATIC_STA_IP_GW);
        if (esp_netif_set_ip_info(esp_default_netif, &ip) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set static ip info");
        }
        ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", DB_STATIC_STA_IP, DB_STATIC_STA_IP_NETMASK,
                 DB_STATIC_STA_IP_GW);
        ESP_ERROR_CHECK(db_set_dns_server(esp_default_netif, ipaddr_addr(DB_STATIC_STA_IP_GW), ESP_NETIF_DNS_MAIN));
        ESP_ERROR_CHECK(db_set_dns_server(esp_default_netif, ipaddr_addr("0.0.0.0"), ESP_NETIF_DNS_BACKUP));
    } else {
        //no static IP specified let the DHCP assign us one
    }
}

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
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_ap_get_sta_list(&wifi_sta_list)); // update list of connected stations
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG, "WIFI_EVENT - Client disconnected - station:"MACSTR", AID=%d", MAC2STR(event->mac), event->aid);
        struct db_udp_client_t db_udp_client;
        memcpy(db_udp_client.mac, event->mac, sizeof(db_udp_client.mac));
        remove_from_known_udp_clients(udp_conn_list, db_udp_client);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_ap_get_sta_list(&wifi_sta_list)); // update list of connected stations
    } else if (event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "WIFI_EVENT_AP_START (SSID: %s PASS: %s)", DB_WIFI_SSID, DB_WIFI_PWD);
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        ESP_LOGI(TAG, "WIFI_EVENT - AP stopped!");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t *event = (ip_event_ap_staipassigned_t *) event_data;
        ESP_LOGI(TAG, "IP_EVENT_AP_STAIPASSIGNED - New station IP:" IPSTR, IP2STR(&event->ip));
        ESP_LOGI(TAG, "IP_EVENT_AP_STAIPASSIGNED - MAC: " MACSTR, MAC2STR(event->mac));
        struct db_udp_client_t db_udp_client;
        db_udp_client.udp_client.sin_family = PF_INET;
        db_udp_client.udp_client.sin_port = htons(APP_PORT_PROXY_UDP);
        db_udp_client.udp_client.sin_len = 16;
        db_udp_client.udp_client.sin_addr.s_addr = event->ip.addr;
        memcpy(db_udp_client.mac, event->mac, sizeof(db_udp_client.mac));
        add_to_known_udp_clients(udp_conn_list, db_udp_client, false);
    }
    // Wifi client mode events
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START - Wifi Started");
        if (!DB_WIFI_IS_OFF) {  // maybe the other task did set it in the meantime
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
        } else {
            ESP_LOGW(TAG, "Did not start Wi-Fi since autopilot told us he is armed (WIFI_EVENT_STA_START)");
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        set_client_static_ip();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED - Lost connection to access point");
        // Keep on trying
        if (!DB_WIFI_IS_OFF) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP (%i)", s_retry_num);
        } else {
            ESP_LOGD(TAG, "WIFI_EVENT_STA_DISCONNECTED - did not try to re-connect since WiFi was commanded off");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP:" IPSTR, IP2STR(&event->ip_info.ip));
        sprintf(CURRENT_CLIENT_IP, IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void start_mdns_service() {
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
        ESP_LOGI(TAG, "Filesystem init finished! Partition size: total: %d bytes, used: %d bytes (%i%%)", total, used,
                 (used * 100) / total);
    }
    return ret;
}

#endif

/**
 * Launches an access point where ground stations can connect to
 *
 * @param wifi_mode Allows to overwrite an AP mode from traditional WiFi to LR Mode
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
    strncpy((char *) wifi_config.ap.ssid, (char *) DB_WIFI_SSID, 32);
    strncpy((char *) wifi_config.ap.password, (char *) DB_WIFI_PWD, 64);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    if (wifi_mode == DB_WIFI_MODE_AP_LR) {
        ESP_LOGI(TAG, "Enabling LR Mode on access point. This device will be invisible to non-ESP32 devices!");
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B));
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    wifi_country_t wifi_country = {.cc = "US", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    DB_WIFI_IS_OFF = false; // just to be sure, but should not be necessary

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
 * Initializes the ESP Wifi client/station mode where we connect to a known access point.
 */
int init_wifi_clientmode() {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_default_netif = esp_netif_create_default_wifi_sta();
    assert(esp_default_netif);

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
    strncpy((char *) wifi_config.sta.ssid, (char *) DB_WIFI_SSID, 32);
    strncpy((char *) wifi_config.sta.password, (char *) DB_WIFI_PWD, 64);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // disable power saving
    ESP_ERROR_CHECK(esp_wifi_start());
    DB_WIFI_IS_OFF = false; // just to be sure, but should not be necessary
    // consider connection lost after 1s of no beacon - triggers reconnect via WIFI_EVENT_STA_DISCONNECTED event
    ESP_ERROR_CHECK(esp_wifi_set_inactive_time(WIFI_IF_STA, 3));

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
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR));
    ESP_LOGI(TAG, "Enabled ESP-NOW WiFi Mode! LR Mode is set. This device will be invisible to non-ESP32 devices!");
    ESP_ERROR_CHECK(esp_read_mac(LOCAL_MAC_ADDRESS, ESP_MAC_WIFI_STA));
}

/**
 * Enables or disables (via reboot) the WiFi if the DB_DISABLE_WIFI_ARMED parameter is set. Not used during boot.
 * Usually called when arm state change of the autopilot is detected. As internal check if the WiFi is already enabled/disabled
 * WiFi must be inited first (done during boot).
 * @param enable_wifi True to enable the WiFi and FALSE to disable it
 */
void db_set_wifi_status(uint8_t enable_wifi) {
    if (DB_DISABLE_WIFI_ARMED) {    // check if that feature is enabled by the user
        if (enable_wifi && DB_WIFI_IS_OFF) {
            ESP_LOGI(TAG, "Rebooting ESP32 to re-enable Wi-Fi");
            esp_restart(); // enable Wi-Fi by restarting ESP32 - easy way to make sure all things are set up right
        } else if (!enable_wifi && !DB_WIFI_IS_OFF) {
            ESP_LOGI(TAG, "Disabling Wi-Fi");
            if (esp_wifi_stop() == ESP_OK) { // disable WiFi
                DB_WIFI_IS_OFF = true;
            } else {
                ESP_LOGW(TAG, "db_set_wifi_status tried to disable Wi-Fi. FAILED");
            }
        }
    } else {
        // nothing to do
    }
}

/**
 * Write settings to non-volatile memory so they can be loaded on next startup. The UDP clients are saved using a
 * separate function since the "save" operation is triggered by a separate button on the UI.
 */
void db_write_settings_to_nvs() {
    ESP_LOGI(TAG,
             "Trying to save:\nWifi Mode: %i\nssid %s\nwifi_pass %s\nwifi_chan %i\nbaud %liu\ngpio_tx %i\ngpio_rx %i\ngpio_cts %i\ngpio_rts %i\nrts_thresh %i\nproto %i\n"
             "trans_pack_size %i\nltm_per_packet %i\nap_ip %s\nip_sta %s\nip_sta_gw %s\nip_sta_netmsk %s\ndis_wifi_arm %i",
             DB_WIFI_MODE_DESIGNATED, DB_WIFI_SSID, DB_WIFI_PWD, DB_WIFI_CHANNEL, DB_UART_BAUD_RATE, DB_UART_PIN_TX,
             DB_UART_PIN_RX,
             DB_UART_PIN_CTS, DB_UART_PIN_RTS, DB_UART_RTS_THRESH,
             DB_SERIAL_PROTOCOL, DB_TRANS_BUF_SIZE, DB_LTM_FRAME_NUM_BUFFER,
             DEFAULT_AP_IP, DB_STATIC_STA_IP, DB_STATIC_STA_IP_GW, DB_STATIC_STA_IP_NETMASK, DB_DISABLE_WIFI_ARMED);
    ESP_LOGI(TAG, "Saving to NVS %s", NVS_NAMESPACE);
    nvs_handle my_handle;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "esp32_mode",
                               DB_WIFI_MODE_DESIGNATED));  // only DB_WIFI_MODE_DESIGNATED gets updated by user
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ssid", (char *) DB_WIFI_SSID));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "wifi_pass", (char *) DB_WIFI_PWD));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "wifi_chan", DB_WIFI_CHANNEL));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "ant_use_ext", DB_EN_EXT_ANT));
    ESP_ERROR_CHECK(nvs_set_i32(my_handle, "baud", DB_UART_BAUD_RATE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_tx", DB_UART_PIN_TX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_rx", DB_UART_PIN_RX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_cts", DB_UART_PIN_CTS));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_rts", DB_UART_PIN_RTS));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "rts_thresh", DB_UART_RTS_THRESH));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "proto", DB_SERIAL_PROTOCOL));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "trans_pack_size", DB_TRANS_BUF_SIZE));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "serial_timeout", DB_SERIAL_READ_TIMEOUT_MS));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "ltm_per_packet", DB_LTM_FRAME_NUM_BUFFER));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "dis_wifi_arm", DB_DISABLE_WIFI_ARMED));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ap_ip", DEFAULT_AP_IP));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ip_sta", DB_STATIC_STA_IP));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ip_sta_gw", DB_STATIC_STA_IP_GW));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ip_sta_netmsk", DB_STATIC_STA_IP_NETMASK));

    ESP_ERROR_CHECK(nvs_commit(my_handle));
    nvs_close(my_handle);
}

/**
 * Saves a udp client to the NVM so it can be automatically added on the next boot. No need for the user to manually add it again.
 * Only one UDP client can be saved to the NVM.
 * @param new_db_udp_client The client to add to NVM. Must have IP and port set.
 * @param clear_client Set to true to remove the current client from NVM. In that case the new_db_udp_client param will be ignored.
 */
void save_udp_client_to_nvm(struct db_udp_client_t *new_db_udp_client, bool clear_client) {
    char ip[INET_ADDRSTRLEN];
    uint16_t port;
    if (!clear_client) {
        // convert addr to string
        char client_str[INET_ADDRSTRLEN + 6];
        inet_ntop(AF_INET, &(new_db_udp_client->udp_client.sin_addr), ip, INET_ADDRSTRLEN);
        port = ntohs(new_db_udp_client->udp_client.sin_port);
        snprintf(client_str, sizeof(client_str), "%s:%d", ip, port);
        ESP_LOGI(TAG, "Saving UDP client %s to NVS %s", client_str, NVS_NAMESPACE);
    } else {
        // clear client from NVM by setting string to empty "" and port to 0
        ip[0] = '\0';
        port = 0;
        ESP_LOGI(TAG, "Clearing UDP client from NVM");
    }

    nvs_handle my_handle;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "udp_client_ip", ip));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "udp_client_port", port));

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
    if (strlen(key) + 1 > NVS_KEY_NAME_MAX_SIZE)
        ESP_LOGW(TAG, "key %s is longer than %i bytes", key, NVS_KEY_NAME_MAX_SIZE);
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
 * Read stored settings from internal storage including the saved UDP client.
 */
void db_read_settings_nvs() {
    nvs_handle my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK) {
        ESP_LOGI(TAG, "NVS namespace not found. Erasing flash, init NVS ...");
        nvs_close(my_handle);
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        db_write_settings_to_nvs();
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
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "ant_use_ext", &DB_EN_EXT_ANT));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_i32(my_handle, "baud", &DB_UART_BAUD_RATE));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "gpio_tx", &DB_UART_PIN_TX));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "gpio_rx", &DB_UART_PIN_RX));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "gpio_cts", &DB_UART_PIN_CTS));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "gpio_rts", &DB_UART_PIN_RTS));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "rts_thresh", &DB_UART_RTS_THRESH));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "proto", &DB_SERIAL_PROTOCOL));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u16(my_handle, "trans_pack_size", &DB_TRANS_BUF_SIZE));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u16(my_handle, "serial_timeout", &DB_SERIAL_READ_TIMEOUT_MS));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "ltm_per_packet", &DB_LTM_FRAME_NUM_BUFFER));
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(my_handle, "dis_wifi_arm", &DB_DISABLE_WIFI_ARMED));
        // get saved UDP client - this read might result in an error if no client was saved prev. by the user
        char udp_client_ip_str[INET_ADDRSTRLEN + 6];
        udp_client_ip_str[0] = '\0';
        db_read_str_nvs(my_handle, "udp_client_ip", udp_client_ip_str);
        uint16_t udp_client_port = 0;
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u16(my_handle, "udp_client_port", &udp_client_port));

        // close NVM
        nvs_close(my_handle);
        ESP_LOGI(TAG,
                 "\tWifi Mode: %i\n\twifi_chan %i\n\tant_use_ext %i\n\tdis_wifi_arm %i\n\tbaud %liu\n\tgpio_tx %i\n\tgpio_rx %i\n\tgpio_cts %i\n\t"
                 "gpio_rts %i\n\trts_thresh %i\n\tproto %i\n\ttrans_pack_size %i\n\tltm_per_packet %i\n\tserial_timeout %i",
                 DB_WIFI_MODE, DB_WIFI_CHANNEL, DB_EN_EXT_ANT, DB_DISABLE_WIFI_ARMED, DB_UART_BAUD_RATE, DB_UART_PIN_TX, DB_UART_PIN_RX,
                 DB_UART_PIN_CTS, DB_UART_PIN_RTS, DB_UART_RTS_THRESH, DB_SERIAL_PROTOCOL, DB_TRANS_BUF_SIZE,
                 DB_LTM_FRAME_NUM_BUFFER, DB_SERIAL_READ_TIMEOUT_MS);
        if (strlen(udp_client_ip_str) > 0 && udp_client_port != 0) {
            // there was a saved UDP client in the NVM - add it to the udp clients list
            ESP_LOGI(TAG, "Adding %s:%i to known UDP clients.", udp_client_ip_str, udp_client_port);
            struct sockaddr_in new_sockaddr;
            memset(&new_sockaddr, 0, sizeof(new_sockaddr));
            new_sockaddr.sin_family = AF_INET;
            inet_pton(AF_INET, udp_client_ip_str, &new_sockaddr.sin_addr);
            new_sockaddr.sin_port = htons(udp_client_port);
            struct db_udp_client_t new_udp_client = {
                    .udp_client = new_sockaddr,
                    .mac = {0, 0, 0, 0, 0, 0}   // dummy MAC
            };
            bool save_to_nvm = false;   // no need to save it to NVM again
            add_to_known_udp_clients(udp_conn_list, new_udp_client, save_to_nvm);
        } else {
            // no saved UDP client - do nothing
        }
    }
}

/**
 * Callback for a short press (<CONFIG_BUTTON_SHORT_PRESS_TIME_MS) of the reset/boot button.
 * Sets mode to WiFi access point mode with default password "dronebridge" so user can check/change the config
 * @param arg
 */
void short_press_callback(void *arg, void *usr_data) {
    ESP_LOGW(TAG, "Short press detected setting wifi mode to access point with password: dronebridge");
    DB_WIFI_MODE_DESIGNATED = DB_WIFI_MODE_AP;  // do not directly change DB_WIFI_MODE since it is not safe and constantly processed by other tasks. Save settings and reboot will assign DB_WIFI_MODE_DESIGNATED to DB_WIFI_MODE.
    strncpy((char *) DB_WIFI_SSID, "DroneBridge for ESP32", sizeof(DB_WIFI_SSID) - 1);
    strncpy((char *) DB_WIFI_PWD, "dronebridge", sizeof(DB_WIFI_PWD) - 1);
    db_write_settings_to_nvs();
    esp_restart();
}

/**
 * Callback for a long press (>CONFIG_BUTTON_LONG_PRESS_TIME_MS) of the reset/boot button.
 * Resets all settings to defaults.
 * @param arg
 */
void long_press_callback(void *arg, void *usr_data) {
    ESP_LOGW(TAG, "Reset triggered via GPIO %i. Resetting settings and rebooting", DB_RESET_PIN);
    DB_WIFI_MODE_DESIGNATED = DB_WIFI_MODE_AP;  // do not directly change DB_WIFI_MODE since it is not safe and constantly processed by other tasks. Save settings and reboot will assign DB_WIFI_MODE_DESIGNATED to DB_WIFI_MODE.
    strncpy((char *) DB_WIFI_SSID, "DroneBridge for ESP32", sizeof(DB_WIFI_SSID) - 1);
    strncpy((char *) DB_WIFI_PWD, "dronebridge", sizeof(DB_WIFI_PWD) - 1);
    strncpy(DEFAULT_AP_IP, "192.168.2.1", sizeof(DEFAULT_AP_IP) - 1);
    memset(DB_STATIC_STA_IP, 0, strlen(DB_STATIC_STA_IP));
    memset(DB_STATIC_STA_IP_GW, 0, strlen(DB_STATIC_STA_IP_GW));
    memset(DB_STATIC_STA_IP_NETMASK, 0, strlen(DB_STATIC_STA_IP_NETMASK));
    DB_WIFI_CHANNEL = 6;
    DB_UART_PIN_TX = DB_DEFAULT_UART_TX_PIN;
    DB_UART_PIN_RX = DB_DEFAULT_UART_RX_PIN;
    DB_UART_PIN_CTS = DB_DEFAULT_UART_CTS_PIN;
    DB_UART_PIN_RTS = DB_DEFAULT_UART_RTS_PIN; //
    DB_UART_BAUD_RATE = DB_DEFAULT_UART_BAUD_RATE;
    DB_SERIAL_PROTOCOL = DB_SERIAL_PROTOCOL_MAVLINK;
    DB_DISABLE_WIFI_ARMED = false;
    DB_TRANS_BUF_SIZE = 128;
    DB_UART_RTS_THRESH = 64;
    DB_EN_EXT_ANT = false; // on-board antenna by default
    DB_SERIAL_READ_TIMEOUT_MS = DB_SERIAL_READ_TIMEOUT_MS_DEFAULT;
    db_write_settings_to_nvs();
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
    if (NULL == gpio_btn) {
        ESP_LOGE(TAG, "Creating reset button failed");
    } else {
        iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, short_press_callback, NULL);
        iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_UP, long_press_callback, NULL);
    }
}

/**
 * For simple debugging when serial via JTAG is enabled. Printed once control module configured USB serial socket.
 * Write settings to JTAG/USB, so we can debug issues better
 */
void db_jtag_serial_info_print() {
    uint8_t buffer[512];
    int len = sprintf((char *) buffer,
                      "\tWifi Mode: %i\n\twifi_chan %i\n\tant_use_ext %i\n\tbaud %liu\n\tgpio_tx %i\n\tgpio_rx %i\n\tgpio_cts %i\n\t"
                      "gpio_rts %i\n\trts_thresh %i\n\tproto %i\n\ttrans_pack_size %i\n\tltm_per_packet %i\n\tserial_timeout %i\n\tdis_wifi_arm %i\n",
                      DB_WIFI_MODE, DB_WIFI_CHANNEL, DB_EN_EXT_ANT, DB_UART_BAUD_RATE, DB_UART_PIN_TX, DB_UART_PIN_RX,
                      DB_UART_PIN_CTS, DB_UART_PIN_RTS, DB_UART_RTS_THRESH, DB_SERIAL_PROTOCOL, DB_TRANS_BUF_SIZE,
                      DB_LTM_FRAME_NUM_BUFFER, DB_SERIAL_READ_TIMEOUT_MS, DB_DISABLE_WIFI_ARMED);
    write_to_serial(buffer, len);
    len = sprintf((char *) buffer, "\tSSID: %s\n", DB_WIFI_SSID);
    write_to_serial(buffer, len);
    len = sprintf((char *) buffer, "\tPassword: %s\n", DB_WIFI_PWD);
    write_to_serial(buffer, len);
    len = sprintf((char *) buffer, "\tAP IP: %s\n", DEFAULT_AP_IP);
    write_to_serial(buffer, len);
    len = sprintf((char *) buffer, "\tStatic IP: %s\n", DB_STATIC_STA_IP);
    write_to_serial(buffer, len);
    len = sprintf((char *) buffer, "\tStatic IP gateway: %s\n", DB_STATIC_STA_IP_GW);
    write_to_serial(buffer, len);
    len = sprintf((char *) buffer, "\tStatic IP netmask: %s\n", DB_STATIC_STA_IP_NETMASK);
    write_to_serial(buffer, len);
    ESP_LOGI(TAG, "Wrote to serial!");
}

/**
 * 1. Checks if RF Switch is configured.
 * 2. Enables RF switch (necessary on some boards).
 * 3. Sets RF switch according to user setting.
 */
void db_configure_antenna() {
#if defined(CONFIG_DB_HAS_RF_SWITCH) && defined(CONFIG_DB_RF_SWITCH_GPIO) && (CONFIG_DB_RF_SWITCH_GPIO != 0)
    #ifdef CONFIG_DB_OFFICIAL_BOARD_1_X_C6
        gpio_set_level(GPIO_NUM_13, 0); // set to low to enable RF switching
    #endif
    gpio_set_level(CONFIG_DB_RF_SWITCH_GPIO, DB_EN_EXT_ANT);   // set level to enable external/internal antenna
#endif
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
    udp_conn_list = udp_client_list_create();   // http server functions and db_read_settings_nvs expect the list to exist
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    db_read_settings_nvs();
    DB_WIFI_MODE_DESIGNATED = DB_WIFI_MODE; // must always match, mismatch only allowed when changed by user action and not rebooted, yet.
    set_reset_trigger();
    if (DB_WIFI_MODE == DB_WIFI_MODE_AP || DB_WIFI_MODE == DB_WIFI_MODE_AP_LR) {
        init_wifi_apmode(DB_WIFI_MODE);
    } else if (DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_AIR || DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_GND) {
        init_wifi_espnow();
        db_espnow_module();
    } else {
        // Wi-Fi client mode with LR mode enabled
        if (init_wifi_clientmode() < 0) {
            ESP_LOGE(TAG, "Failed to init Wifi Client Mode");
        }
    }

    if (DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_AIR && DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_GND) {
        // no need to start these services - won`t be available anyway - safe the resources
        start_mdns_service();
        netbiosns_init();
        netbiosns_set_name("dronebridge");
    }
    db_configure_antenna();
    ESP_ERROR_CHECK(init_fs());
    control_module();

    if (DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_AIR && DB_WIFI_MODE != DB_WIFI_MODE_ESPNOW_GND) {
        // no need to start these services - won`t be available anyway - safe the resources
        ESP_ERROR_CHECK(start_rest_server(CONFIG_WEB_MOUNT_POINT));
        ESP_LOGI(TAG, "Rest Server started");
        // Disable legacy support for DroneBridge communication module - no use case for DroneBridge for ESP32
        // communication_module();
    }
    ESP_LOGI(TAG, "app_main finished initial setup");
}
