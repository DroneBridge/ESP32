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
#include "db_protocol.h"
#include "esp_vfs_semihost.h"
#include "esp_spiffs.h"
#include "http_server.h"
#include "main.h"
#include "db_parameters.h"
#include "mdns.h"
#include "db_esp_now.h"
#include "iot_button.h"
#include "db_serial.h"
#include "globals.h"

#ifdef CONFIG_BT_ENABLED

#include "db_ble.h"

#endif

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

static const char *TAG = "DB_ESP32";

char CURRENT_CLIENT_IP[IP4ADDR_STRLEN_MAX] = "192.168.2.1";
uint8_t DB_RADIO_IS_OFF = false;  // keep track if we switched Wi-Fi/BLE off already - by default a radio is started
db_esp_signal_quality_t db_esp_signal_quality = {.air_rssi = UINT8_MAX, .air_noise_floor = UINT8_MAX, .gnd_rssi= UINT8_MAX, .gnd_noise_floor = UINT8_MAX};
wifi_sta_list_t wifi_sta_list = {.num = 0};
uint8_t LOCAL_MAC_ADDRESS[6];
udp_conn_list_t *udp_conn_list;

// Wi-Fi client mode vars
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
    if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_STA && strlen((char *) DB_PARAM_STA_IP) > 0 &&
        strlen((char *) DB_PARAM_STA_GW) > 0 &&
        strlen((char *) DB_PARAM_STA_IP_NETMASK) > 0) {
        ESP_LOGI(TAG, "Assigning static IP to ESP32: ESP32-IP: %s Gateway: %s Netmask: %s", (char *) DB_PARAM_STA_IP,
                 (char *) DB_PARAM_STA_GW, (char *) DB_PARAM_STA_IP_NETMASK);

        if (esp_netif_dhcpc_stop(esp_default_netif) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop dhcp client in order to set static IP");
            return;
        }
        esp_netif_ip_info_t ip;
        memset(&ip, 0, sizeof(esp_netif_ip_info_t));
        ip.ip.addr = ipaddr_addr((char *) DB_PARAM_STA_IP);
        ip.netmask.addr = ipaddr_addr((char *) DB_PARAM_STA_IP_NETMASK);
        ip.gw.addr = ipaddr_addr((char *) DB_PARAM_STA_GW);
        if (esp_netif_set_ip_info(esp_default_netif, &ip) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set static ip info");
        }
        ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", (char *) DB_PARAM_STA_IP,
                 (char *) DB_PARAM_STA_IP_NETMASK,
                 (char *) DB_PARAM_STA_GW);
        ESP_ERROR_CHECK(
                db_set_dns_server(esp_default_netif, ipaddr_addr((char *) DB_PARAM_STA_GW), ESP_NETIF_DNS_MAIN));
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
        ESP_LOGI(TAG, "WIFI_EVENT_AP_START (SSID: %s PASS: %s)", DB_PARAM_WIFI_SSID, DB_PARAM_PASS);
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
        if (!DB_RADIO_IS_OFF) {  // maybe the other task did set it in the meantime
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
        } else {
            ESP_LOGW(TAG, "Did not start Wi-Fi since autopilot told us he is armed (WIFI_EVENT_STA_START)");
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        set_client_static_ip();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED - Lost connection to access point");
        // Keep on trying
        if (!DB_RADIO_IS_OFF) {
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
void db_init_wifi_apmode(int wifi_mode) {
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
                    .ssid = "DroneBridge ESP32 Init Error",
                    .password = "dronebridge",
                    .ssid_len = 0,
                    .authmode = WIFI_AUTH_WPA2_PSK,
                    .channel = db_param_channel.value.db_param_u8.value,
                    .ssid_hidden = 0,
                    .beacon_interval = 100,
                    .max_connection = 10
            },
    };
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
    // Set Wi-Fi SSID and password from the stored parameters
    if (strlen(DB_PARAM_WIFI_SSID) >= db_param_ssid.value.db_param_str.min_len) {
        strncpy((char *) wifi_config.ap.ssid, DB_PARAM_WIFI_SSID, db_param_ssid.value.db_param_str.max_len);
    } else {
        // something is wrong - switch to default value
        strncpy((char *) wifi_config.ap.ssid, (char *) db_param_ssid.value.db_param_str.default_value,
                db_param_ssid.value.db_param_str.max_len);
    }
    if (strlen(DB_PARAM_PASS) >= db_param_pass.value.db_param_str.min_len) {
        strncpy((char *) wifi_config.ap.password, DB_PARAM_PASS, db_param_pass.value.db_param_str.max_len);
    } else {
        // something is wrong - switch to default value
        strncpy((char *) wifi_config.ap.password, (char *) db_param_pass.value.db_param_str.default_value,
                db_param_pass.value.db_param_str.max_len);
    }
#pragma GCC diagnostic pop

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
    DB_RADIO_IS_OFF = false; // just to be sure, but should not be necessary

    /* Assign IP to ap/gateway */
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(DB_PARAM_AP_IP);
    ip.netmask.addr = ipaddr_addr("255.255.255.0");
    ip.gw.addr = ipaddr_addr(DB_PARAM_AP_IP);
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(esp_default_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_default_netif, &ip));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_default_netif));

    ESP_ERROR_CHECK(
            esp_netif_set_hostname(esp_default_netif, (char *) db_param_wifi_hostname.value.db_param_str.value));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
    strncpy(CURRENT_CLIENT_IP, DB_PARAM_AP_IP, sizeof(CURRENT_CLIENT_IP));
#pragma GCC diagnostic pop
    ESP_ERROR_CHECK(esp_read_mac(LOCAL_MAC_ADDRESS, ESP_MAC_WIFI_SOFTAP));
}

/**
 * Initializes the ESP Wi-fi client/station mode where we connect to a known access point.
 */
int db_init_wifi_clientmode() {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_default_netif = esp_netif_create_default_wifi_sta();
    assert(esp_default_netif);
    ESP_ERROR_CHECK(
            esp_netif_set_hostname(esp_default_netif, (char *) db_param_wifi_hostname.value.db_param_str.value));

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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
    // Set Wi-Fi SSID and password from the stored parameters
    if (strlen(DB_PARAM_WIFI_SSID) >= db_param_ssid.value.db_param_str.min_len) {
        strncpy((char *) wifi_config.ap.ssid, DB_PARAM_WIFI_SSID, db_param_ssid.value.db_param_str.max_len);
    } else {
        // something is wrong - switch to default value
        strncpy((char *) wifi_config.ap.ssid, (char *) db_param_ssid.value.db_param_str.default_value,
                db_param_ssid.value.db_param_str.max_len);
    }
    if (strlen(DB_PARAM_PASS) >= db_param_pass.value.db_param_str.min_len) {
        strncpy((char *) wifi_config.ap.password, DB_PARAM_PASS, db_param_pass.value.db_param_str.max_len);
    } else {
        // something is wrong - switch to default value
        strncpy((char *) wifi_config.ap.password, (char *) db_param_pass.value.db_param_str.default_value,
                db_param_pass.value.db_param_str.max_len);
    }
#pragma GCC diagnostic pop

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    if (DB_PARAM_WIFI_EN_GN) {
        // only makes sense if the AP can not do proper N or you do not need range or want Wi-Fi 6 ax support
#ifdef CONFIG_IDF_TARGET_ESP32C6
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX | WIFI_PROTOCOL_LR));
#else
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N |
                                                           WIFI_PROTOCOL_LR));
#endif
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR));  // range for sure
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // disable power saving
    ESP_ERROR_CHECK(esp_wifi_start());
    DB_RADIO_IS_OFF = false; // just to be sure, but should not be necessary
    // Consider connection lost after 1s of no beacon - triggers reconnect via WIFI_EVENT_STA_DISCONNECTED event
    ESP_ERROR_CHECK(esp_wifi_set_inactive_time(WIFI_IF_STA, 3));

    ESP_LOGI(TAG, "Init of WiFi Client-Mode finished. (SSID: %s PASS: %s)", DB_PARAM_WIFI_SSID, DB_PARAM_PASS);

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
        ESP_LOGI(TAG, "Connected to ap SSID:%s password:%s", DB_PARAM_WIFI_SSID, DB_PARAM_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "Failed to connect to SSID:%s, password:%s", DB_PARAM_WIFI_SSID, DB_PARAM_PASS);
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
 * Initialize Wi-Fi for ESP-NOW mode.
 * If someone uses ESP-NOW over Wi-Fi it is because he wants range over everything else.
 * LR mode makes it very inconvenient to change settings but gives the most range. No AP mode since AP will not be
 * visible.
 */
void db_init_wifi_espnow() {
    ESP_LOGI(TAG, "Setting up Wi-Fi for ESP-NOW");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(DB_PARAM_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR));
    ESP_LOGI(TAG, "Enabled ESP-NOW WiFi Mode! LR Mode is set. This device will be invisible to non-ESP32 devices!");
    ESP_ERROR_CHECK(esp_read_mac(LOCAL_MAC_ADDRESS, ESP_MAC_WIFI_STA));
}

/**
 * Enables or disables (via reboot) the Wi-Fi/BLE if the DB_PARAM_DIS_RADIO_ON_ARM parameter is set.
 * Not used during boot.
 * Usually called when arm state change of the autopilot is detected.
 * As internal check if the Wi-Fi is already enabled/disabled Wi-Fi must be inited first (done during boot).
 *
 * This is handy for modes like BLE & WiFi AP which are used to configure the drone and usually do not act as a long
 * range telemetry link. In these cases interference on the drone can be reduced by disabling the radio.
 *
 * @param enable_wifi True to enable the Wi-Fi and FALSE to disable it
 */
void db_set_radio_status(uint8_t enable_wifi) {
    if (DB_PARAM_DIS_RADIO_ON_ARM) {    // check if the user enables that feature
        if (enable_wifi && DB_RADIO_IS_OFF) {
            ESP_LOGI(TAG, "Rebooting ESP32 to re-enable Wi-Fi/BLE");
            esp_restart(); // enable Wi-Fi/BLE by restarting ESP32 - an easy way to make sure all things are set up right
        } else if (!enable_wifi && !DB_RADIO_IS_OFF) {
            ESP_LOGI(TAG, "Disabling Wi-Fi/BLE");
            if (DB_PARAM_RADIO_MODE == DB_BLUETOOTH_MODE) {
                db_ble_deinit();    // disable BLE
                DB_RADIO_IS_OFF = true;
            } else {
                if (esp_wifi_stop() == ESP_OK) { // disable WiFi
                    DB_RADIO_IS_OFF = true;
                } else {
                    ESP_LOGW(TAG, "db_set_radio_status tried to disable Wi-Fi. FAILED");
                }
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
    // print parameters to console for logging
    ESP_LOGI(TAG, "Trying to save parameters:");
    uint8_t param_str_buffer[512];
    db_param_print_values_to_buffer(param_str_buffer);
    ESP_LOGI(TAG, "%s", param_str_buffer);
    ESP_LOGI(TAG, "Saving to NVS %s", NVS_NAMESPACE);
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
    db_param_write_all_params_nvs(&my_handle);
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Finished saving operation.");
}

/**
 * Saves an udp client to the NVM so it can be automatically added on the next boot. No need for the user to manually add it again.
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
    ESP_ERROR_CHECK(nvs_set_str(my_handle, (char *) db_param_udp_client_ip.db_name, ip));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, (char *) db_param_udp_client_port.db_name, port));

    ESP_ERROR_CHECK(nvs_commit(my_handle));
    nvs_close(my_handle);
}

/**
 * Read stored settings from internal storage including the saved UDP client.
 */
void db_read_settings_nvs() {
    nvs_handle my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK) {
        ESP_LOGI(TAG, "NVS namespace not found. Using default values, setting up NVS...");
        nvs_close(my_handle);
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        
        // Set all parameters to their default values when flash is empty
        db_param_reset_all();
        
        // Now save these defaults to NVS
        db_write_settings_to_nvs();
        
        // Print parameters to console for logging
        uint8_t param_str_buffer[512] = {0};
        db_param_print_values_to_buffer(param_str_buffer);
        ESP_LOGI(TAG, "Initialized with default values:\n%s", (char *)param_str_buffer);
    } else {
        ESP_LOGI(TAG, "Reading settings from NVS");
        db_param_read_all_params_nvs(&my_handle);
        nvs_close(my_handle);

        // print parameters to console for logging
        uint8_t param_str_buffer[512] = {0};
        db_param_print_values_to_buffer(param_str_buffer);
        ESP_LOGI(TAG, "\n%s", (char *) param_str_buffer);

        // Check if we have a saved UDP client from the last session. Add it to the known udp clients if there is one.
        if (strlen((char *) db_param_udp_client_ip.value.db_param_str.value) > 0 &&
            db_param_udp_client_port.value.db_param_u16.value != 0) {
            // there was a saved UDP client in the NVM from last session - add it to the udp clients list
            ESP_LOGI(TAG, "Adding %s:%i to known UDP clients.",
                     (char *) db_param_udp_client_ip.value.db_param_str.value,
                     db_param_udp_client_port.value.db_param_u8.value);
            struct sockaddr_in new_sockaddr;
            memset(&new_sockaddr, 0, sizeof(new_sockaddr));
            new_sockaddr.sin_family = AF_INET;
            inet_pton(AF_INET, (char *) db_param_udp_client_ip.value.db_param_str.value, &new_sockaddr.sin_addr);
            new_sockaddr.sin_port = htons(db_param_udp_client_port.value.db_param_u16.value);
            struct db_udp_client_t new_udp_client = {
                    .udp_client = new_sockaddr,
                    .mac = {0, 0, 0, 0, 0, 0}   // dummy MAC
            };
            bool save_to_nvm = false;   // no need to save it to NVM again
            add_to_known_udp_clients(udp_conn_list, new_udp_client, save_to_nvm);
        } else {
            // no saved UDP client - do nothing
            ESP_LOGI(TAG, "No saved UDP client - skipping");
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
    DB_RADIO_MODE_DESIGNATED = DB_WIFI_MODE_AP;  // Do not directly change DB_PARAM_RADIO_MODE since it is not safe and constantly processed by other tasks. Save settings and reboot will assign DB_RADIO_MODE_DESIGNATED to DB_PARAM_RADIO_MODE.
    db_param_set_to_default(&db_param_ssid);
    db_param_set_to_default(&db_param_pass);
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
    DB_RADIO_MODE_DESIGNATED = DB_WIFI_MODE_AP;  // Do not directly change DB_PARAM_RADIO_MODE since it is not safe and constantly processed by other tasks. Save settings and reboot will assign DB_RADIO_MODE_DESIGNATED to DB_PARAM_RADIO_MODE.
    db_param_reset_all();
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
    const int len = db_param_print_values_to_buffer(buffer);
    write_to_serial(buffer, len);
}

/**
 * 1. Checks if RF Switch is configured.
 * 2. Enables RF switch (necessary on some boards).
 * 3. Sets RF switch according to user setting.
 * Must be called before radio/Wi-Fi gets initialized!
 */
void db_configure_antenna() {
#if defined(CONFIG_DB_HAS_RF_SWITCH) && defined(CONFIG_DB_RF_SWITCH_GPIO) && (CONFIG_DB_RF_SWITCH_GPIO != 0)
#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X_C6
    gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_3, 0); // set to low to enable RF switching
#endif
    gpio_set_direction(CONFIG_DB_RF_SWITCH_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_DB_RF_SWITCH_GPIO, DB_PARAM_EN_EXT_ANT);   // set level to enable external/internal antenna
    ESP_LOGI(TAG, "External antenna usage: %i", DB_PARAM_EN_EXT_ANT);
#endif
}

/**
 * Main entry point.
 */
void app_main() {
    db_param_init_parameters();
    udp_conn_list = udp_client_list_create(); // http server functions and db_read_settings_nvs expect the list to exist
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    db_read_settings_nvs();
    DB_RADIO_MODE_DESIGNATED = DB_PARAM_RADIO_MODE; // must always match, mismatch only allowed when changed by user action and not rebooted, yet.
    set_reset_trigger();
    db_configure_antenna();

    switch (DB_PARAM_RADIO_MODE) {
        case DB_WIFI_MODE_AP:
        case DB_WIFI_MODE_AP_LR:
            db_init_wifi_apmode(DB_PARAM_RADIO_MODE);
            break;
        case DB_WIFI_MODE_ESPNOW_AIR:
        case DB_WIFI_MODE_ESPNOW_GND:
            db_init_wifi_espnow();
            db_start_espnow_module();
            break;
        case DB_BLUETOOTH_MODE:
#ifdef CONFIG_BT_ENABLED
            db_init_wifi_apmode(DB_WIFI_MODE_AP);   // WiFi & BLE co-existence to enable webinterface
            db_ble_queue_init();
            db_ble_init();
#else
            DB_RADIO_MODE_DESIGNATED = DB_WIFI_MODE_AP;
            DB_PARAM_RADIO_MODE = DB_WIFI_MODE_AP;
            ESP_LOGE(TAG, "Bluetooth is not enabled with this build. Please enable it in menuconfig and re-compile. Switching to AP mode.");
            db_init_wifi_apmode(DB_WIFI_MODE_AP);
#endif
            break;
        default:
            // Wi-Fi client mode with LR mode enabled
            if (db_init_wifi_clientmode() < 0) {
                ESP_LOGE(TAG, "Failed to init Wifi Client Mode");
            }
            break;
    }

    if (DB_PARAM_RADIO_MODE != DB_WIFI_MODE_ESPNOW_AIR && DB_PARAM_RADIO_MODE != DB_WIFI_MODE_ESPNOW_GND &&
        DB_PARAM_RADIO_MODE != DB_WIFI_MODE_AP_LR) {
        // no need to start these services - won`t be available anyway - safe the resources
        start_mdns_service();
        netbiosns_init();
        netbiosns_set_name("dronebridge");
    }
    ESP_ERROR_CHECK(init_fs());
    db_start_control_module();

    if (DB_PARAM_RADIO_MODE != DB_WIFI_MODE_ESPNOW_AIR && DB_PARAM_RADIO_MODE != DB_WIFI_MODE_ESPNOW_GND &&
        DB_PARAM_RADIO_MODE != DB_WIFI_MODE_AP_LR) {
        // no need to start these services - won`t be available anyway - safe the resources
        ESP_ERROR_CHECK(start_rest_server(CONFIG_WEB_MOUNT_POINT));
        ESP_LOGI(TAG, "Rest Server started");
        // Disable legacy support for DroneBridge communication module - no use case for DroneBridge for ESP32
        // communication_module();
    }
    ESP_LOGI(TAG, "app_main finished initial setup");
}
