#ifndef DB_ESP32_MAIN_H
#define DB_ESP32_MAIN_H

#include <driver/gpio.h>
#include <esp_wifi_types.h>
#include <lwip/apps/netbiosns.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>

#include "db_esp32_comm.h"
#include "db_esp32_control.h"
#include "db_protocol.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_spiffs.h"
#include "esp_vfs_semihost.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "freertos/event_groups.h"
#include "http_server.h"
#include "mdns.h"

#define NVS_NAMESPACE "settings"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

enum E_DB_NETIF_MODE {
    DB_WIFI_MODE_AP = 1,
    DB_WIFI_MODE_STA = 2,
    DB_WIFI_MODE_AP_LR = 3,
    DB_ETH_MODE = 4
};

void write_settings_to_nvs();

#endif  // DB_ESP32_MAIN_H
