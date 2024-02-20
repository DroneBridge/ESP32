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

#include "main.h"

static const char *TAG = "DB_ESP32";

uint8_t DB_NETIF_MODE = DB_ETH_MODE;  // 1=Wifi AP mode, 2=Wifi client mode, 3=ESP-NOW LR Mode, 4=Ethernet mode
uint8_t DEFAULT_SSID[32] = "DroneBridge ESP32";
uint8_t DEFAULT_PWD[64] = "dronebridge";
char DEFAULT_AP_IP[32] = "192.168.2.1";
char CURRENT_CLIENT_IP[32] = "192.168.2.1";
uint8_t DEFAULT_CHANNEL = 6;
uint8_t SERIAL_PROTOCOL = 4;  // 1=MSP, 4=MAVLink/transparent

// initially set both pins to 0 to allow the start of the system on all boards. User has to set the correct pins
uint8_t DB_UART_PIN_TX = GPIO_NUM_0;
uint8_t DB_UART_PIN_RX = GPIO_NUM_0;

int32_t DB_UART_BAUD_RATE = 115200;
uint16_t TRANSPARENT_BUF_SIZE = 64;
uint8_t LTM_FRAME_NUM_BUFFER = 1;
uint8_t MSP_LTM_SAMEPORT = 0;
int station_rssi = 0;

struct udp_conn_list_t *udp_conn_list;

// Wifi client mode vars
int WIFI_ESP_MAXIMUM_RETRY = 25;  // max number of retries to connect to the ap before enabling temp. ap mode
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

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
static void netif_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        // AP events
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "%s - Client connected - station:" MACSTR ", AID=%d", event_base, MAC2STR(event->mac), event->aid);
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGI(TAG, "%s - Client disconnected - station:" MACSTR ", AID=%d", event_base, MAC2STR(event->mac), event->aid);
            struct db_udp_client_t db_udp_client;
            memcpy(db_udp_client.mac, event->mac, sizeof(db_udp_client.mac));
            remove_from_known_udp_clients(udp_conn_list, db_udp_client);
        } else if (event_id == WIFI_EVENT_AP_START) {
            ESP_LOGI(TAG, "%s - AP started! (SSID: %s PASS: %s)", event_base, DEFAULT_SSID, DEFAULT_PWD);
        } else if (event_id == WIFI_EVENT_AP_STOP) {
            ESP_LOGI(TAG, "%s - AP stopped!", event_base);
        }
        // STA events
        else if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "%s - Wifi Started", event_base);
            ESP_ERROR_CHECK(esp_wifi_connect());
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGI(TAG, "%s - Lost connection to access point", event_base);
            if (s_retry_num < WIFI_ESP_MAXIMUM_RETRY) {
                ESP_ERROR_CHECK(esp_wifi_connect());
                s_retry_num++;
                ESP_LOGI(TAG, "Retry to connect to the AP (%i/%i)", s_retry_num, WIFI_ESP_MAXIMUM_RETRY);
            } else {
                ESP_LOGI(TAG, "Connecting to the AP failed");
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
        }
    } else if (event_base == ETH_EVENT) {
        if (event_id == ETHERNET_EVENT_CONNECTED) {
            uint8_t mac_addr[6] = {0};
            esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            ESP_LOGI(TAG, "%s - Link Up", event_base);
            ESP_LOGI(TAG, "%s - HW Addr:" MACSTR, event_base, MAC2STR(mac_addr));
        } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
            uint8_t mac_addr[6] = {0};
            esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            ESP_LOGI(TAG, "%s - Link Down", event_base);
            struct db_udp_client_t db_udp_client;
            memcpy(db_udp_client.mac, mac_addr, sizeof(db_udp_client.mac));
            remove_from_known_udp_clients(udp_conn_list, db_udp_client);
        } else if (event_id == ETHERNET_EVENT_START) {
            ESP_LOGI(TAG, "%s - Started", event_base);
        } else if (event_id == ETHERNET_EVENT_STOP) {
            ESP_LOGI(TAG, "%s - Stopped", event_base);
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_AP_STAIPASSIGNED || event_id == IP_EVENT_ETH_GOT_IP) {
            ip_event_ap_staipassigned_t *event = (ip_event_ap_staipassigned_t *)event_data;
            ESP_LOGI(TAG, "%s - New station IP:" IPSTR, event_base, IP2STR(&event->ip));
            ESP_LOGI(TAG, "%s - MAC: " MACSTR, event_base, MAC2STR(event->mac));
            struct db_udp_client_t db_udp_client;
            db_udp_client.udp_client.sin_family = PF_INET;
            db_udp_client.udp_client.sin_port = htons(APP_PORT_PROXY_UDP);
            db_udp_client.udp_client.sin_len = 16;
            db_udp_client.udp_client.sin_addr.s_addr = event->ip.addr;
            memcpy(db_udp_client.mac, event->mac, sizeof(db_udp_client.mac));
            add_to_known_udp_clients(udp_conn_list, db_udp_client);
        } else if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            sprintf(CURRENT_CLIENT_IP, IPSTR, IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, "%s - Got IP:%s", event_base, CURRENT_CLIENT_IP);
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

/**
 * @brief Initialize mDNS service
 *
 */
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
        .format_if_mount_failed = false};
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
        ESP_LOGI(TAG, "Filesystem init finished! Partition size: total: %d bytes, used: %d bytes (%i%%)", total, used, (used * 100) / total);
    }
    return ret;
}

#endif

/**
 * Launches an access point where ground stations can connect to
 *
 * @param wifi_mode Allowes to overwrite an AP mode from traditional WiFi to LR Mode
 */
void init_netif_wifi_apmode(int wifi_mode) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_default_netif = esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &netif_event_handler,
                                                        NULL,
                                                        NULL));

    esp_event_handler_instance_t ap_staipassigned_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_AP_STAIPASSIGNED,
                                                        &netif_event_handler,
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
            .max_connection = 10},
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
int init_netif_wifi_clientmode() {
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
                                                        &netif_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &netif_event_handler,
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
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
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
 * Initializes the ESP W32-ETH0 Ethernet mode
 */
void init_netif_ethernet_mode() {
    ESP_LOGD(TAG, "Initializing Ethernet MAC ...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_default_netif = esp_netif_new(&cfg);

    ESP_LOGD(TAG, "Initializing Ethernet PHY (LAN8720A) ...");
    // Init common MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.sw_reset_timeout_ms = 1000;  // from ETH.cpp
    // Update PHY config based on board specific configuration
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;
    phy_config.reset_gpio_num = 16;

    // Init vendor specific MAC config
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;
    esp32_emac_config.smi_mdc_gpio_num = 23;
    esp32_emac_config.smi_mdio_gpio_num = 18;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    // Install and start Ethernet driver
    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    // // Enable external oscillator (pulled down at boot to allow IO0 strapping)
    // ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT));
    // ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_16, 1));

    ESP_LOGD(TAG, "Starting Ethernet interface...");

    // Attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(esp_default_netif, esp_eth_new_netif_glue(eth_handle)));

    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_ETH_GOT_IP,
                                               &netif_event_handler,
                                               &instance_got_ip));
    // esp_netif_flags_t flags_before = esp_netif_get_flags(esp_default_netif);
    // ESP_LOGD(TAG, "flags before: %d", flags_before);
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    ESP_LOGI(TAG, "Ethernet interface started");

    // esp_netif_flags_t flags_after = esp_netif_get_flags(esp_default_netif);
    // if (flags_after & ESP_NETIF_DHCP_CLIENT) {
    //     ESP_LOGD(TAG, "DHCP client!");
    //     ESP_ERROR_CHECK(esp_netif_dhcpc_stop(esp_default_netif));

    // } else if (flags_after & ESP_NETIF_DHCP_SERVER) {
    //     ESP_LOGD(TAG, "DHCP server!");
    // }
    // If DHCP is already started, stop it
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(esp_default_netif));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcpc_stop(esp_default_netif));

    // Setting IP address
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(DEFAULT_AP_IP);
    ip.netmask.addr = ipaddr_addr("255.255.255.0");
    ip.gw.addr = ipaddr_addr(DEFAULT_AP_IP);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_default_netif, &ip));
    // ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_default_netif));
    ESP_ERROR_CHECK(esp_netif_set_hostname(esp_default_netif, "DBESP32"));
    strncpy(CURRENT_CLIENT_IP, DEFAULT_AP_IP, sizeof(CURRENT_CLIENT_IP));
}

void write_settings_to_nvs() {
    ESP_LOGI(TAG,
             "Trying to save: ssid %s\nwifi_pass %s\nwifi_chan %i\nbaud %liu\ngpio_tx %i\ngpio_rx %i\nproto %i\n"
             "trans_pack_size %i\nltm_per_packet %i\nmsp_ltm %i\nap_ip %s",
             DEFAULT_SSID, DEFAULT_PWD, DEFAULT_CHANNEL, DB_UART_BAUD_RATE, DB_UART_PIN_TX, DB_UART_PIN_RX,
             SERIAL_PROTOCOL, TRANSPARENT_BUF_SIZE, LTM_FRAME_NUM_BUFFER, MSP_LTM_SAMEPORT, DEFAULT_AP_IP);
    ESP_LOGI(TAG, "Saving to NVS %s", NVS_NAMESPACE);
    nvs_handle my_handle;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "esp32_mode", DB_NETIF_MODE));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ssid", (char *)DEFAULT_SSID));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "wifi_pass", (char *)DEFAULT_PWD));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "wifi_chan", DEFAULT_CHANNEL));
    ESP_ERROR_CHECK(nvs_set_i32(my_handle, "baud", DB_UART_BAUD_RATE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_tx", DB_UART_PIN_TX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_rx", DB_UART_PIN_RX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "proto", SERIAL_PROTOCOL));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "trans_pack_size", TRANSPARENT_BUF_SIZE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "ltm_per_packet", LTM_FRAME_NUM_BUFFER));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "msp_ltm", MSP_LTM_SAMEPORT));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ap_ip", DEFAULT_AP_IP));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    nvs_close(my_handle);
}

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
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "esp32_mode", &DB_NETIF_MODE));

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
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "proto", &SERIAL_PROTOCOL));
        ESP_ERROR_CHECK(nvs_get_u16(my_handle, "trans_pack_size", &TRANSPARENT_BUF_SIZE));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "ltm_per_packet", &LTM_FRAME_NUM_BUFFER));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "msp_ltm", &MSP_LTM_SAMEPORT));
        nvs_close(my_handle);
        free(wifi_pass);
        free(ssid);
        free(ap_ip);
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

    // ESP_ERROR_CHECK(esp_event_loop_delete_default());
    init_netif_ethernet_mode();
    // if (DB_NETIF_MODE == DB_WIFI_MODE_AP || DB_NETIF_MODE == DB_WIFI_MODE_AP_LR) {
    //     init_netif_wifi_apmode(DB_NETIF_MODE);
    // } else {
    //     if (init_netif_wifi_clientmode() < 0) {
    //         ESP_LOGW(TAG, "Switching to failsafe: Enabling access point mode");
    //         // De-Init all Wi-Fi and enable the AP-Mode temporarily
    //         ESP_ERROR_CHECK(esp_event_loop_delete_default());
    //         esp_netif_destroy_default_wifi(esp_default_netif);
    //         ESP_ERROR_CHECK(esp_wifi_stop());
    //         strncpy((char *)DEFAULT_SSID, "Failsafe DroneBridge ESP32", sizeof(DEFAULT_SSID));
    //         strncpy((char *)DEFAULT_PWD, "dronebridge", sizeof(DEFAULT_PWD));
    //         init_netif_wifi_apmode(DB_WIFI_MODE_AP);
    //     }
    // }

    start_mdns_service();
    netbiosns_init();
    netbiosns_set_name("dronebridge");

    ESP_ERROR_CHECK(init_fs());
    control_module();
    ESP_ERROR_CHECK(start_rest_server(CONFIG_WEB_MOUNT_POINT));
    communication_module();
}