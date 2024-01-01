//
// Created by cyber on 17.07.21.
//

#ifndef DB_ESP32_MAIN_H
#define DB_ESP32_MAIN_H

enum E_DB_WIFI_MODE {
    DB_WIFI_MODE_AP = 1,
    DB_WIFI_MODE_STA = 2,
    DB_WIFI_MODE_AP_LR = 3
};

void write_settings_to_nvs();

#endif //DB_ESP32_MAIN_H
