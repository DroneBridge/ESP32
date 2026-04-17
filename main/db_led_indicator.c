/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2026 Wolfgang Christl
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

#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "db_parameters.h"
#include "db_led_indicator.h"

static const char *TAG = "DB_LED_IND";

// Turn LED off if no qualifying activity was seen within this window.
#define DB_STATUS_LED_TIMEOUT_MS 1000
#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X_C6
#define DB_STATUS_LED_GPIO GPIO_NUM_15
#define DB_STATUS_LED_ACTIVE_LOW 1
#endif

#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X_C6
static TickType_t db_status_led_last_serial_mavlink_tick = 0;
static TickType_t db_status_led_last_radio_tick = 0;
static bool db_status_led_initialized = false;

/**
 * Checks whether an activity timestamp is still inside the LED "on" timeout window.
 */
static bool db_status_led_is_activity_recent(TickType_t now_tick, TickType_t last_tick) {
    if (last_tick == 0) {
        return false;
    }
    return (now_tick - last_tick) < pdMS_TO_TICKS(DB_STATUS_LED_TIMEOUT_MS);
}

/**
 * Maps logical LED state (on/off) to the required GPIO level.
 * Supports active-high and active-low wiring.
 */
static int db_status_led_gpio_level_from_on_state(bool should_be_on) {
    if (DB_STATUS_LED_ACTIVE_LOW) {
        return should_be_on ? 0 : 1;
    }
    return should_be_on ? 1 : 0;
}

/**
 * Maps raw GPIO level back to logical LED state (on/off).
 */
static bool db_status_led_on_state_from_gpio_level(int gpio_level) {
    if (DB_STATUS_LED_ACTIVE_LOW) {
        return gpio_level == 0;
    }
    return gpio_level > 0;
}
#endif

/**
 * Initializes status LED handling for C6 official boards.
 * LED starts in OFF state.
 */
void db_status_led_init() {
#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X_C6
    gpio_reset_pin(DB_STATUS_LED_GPIO);
    // Keep input enabled so gpio_get_level() can be used to check current pin state.
    gpio_set_direction(DB_STATUS_LED_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(DB_STATUS_LED_GPIO, db_status_led_gpio_level_from_on_state(false));
    db_status_led_initialized = true;
    db_status_led_last_serial_mavlink_tick = 0;
    db_status_led_last_radio_tick = 0;
#endif
}

/**
 * Marks "serial MAVLink received" activity for LED state logic.
 * Used in AP, STA and ESP-NOW AIR modes.
 */
void db_status_led_mark_serial_mavlink_rx() {
#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X_C6
    db_status_led_last_serial_mavlink_tick = xTaskGetTickCount();
#endif
}

/**
 * Marks "wireless packet received" activity for LED state logic.
 * Used in AP-LR and ESP-NOW GND modes.
 */
void db_status_led_mark_radio_rx() {
#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X_C6
    db_status_led_last_radio_tick = xTaskGetTickCount();
#endif
}

/**
 * Evaluates current mode + recent activity timestamps and updates LED GPIO if needed.
 * Intended to be called periodically (timer-driven).
 */
void db_status_led_process() {
#ifdef CONFIG_DB_OFFICIAL_BOARD_1_X_C6
    if (!db_status_led_initialized) {
        return;
    }

    TickType_t now_tick = xTaskGetTickCount();
    bool should_led_be_on = false;

    if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_AP ||
        DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_AIR ||
        DB_PARAM_RADIO_MODE == DB_WIFI_MODE_STA) {
        // AP, ESP-NOW AIR, STA: indicate MAVLink activity on serial
        should_led_be_on = db_status_led_is_activity_recent(now_tick, db_status_led_last_serial_mavlink_tick);
    } else if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND ||
               DB_PARAM_RADIO_MODE == DB_WIFI_MODE_AP_LR) {
        // ESP-NOW GND, AP-LR: indicate packet activity received via wireless link
        should_led_be_on = db_status_led_is_activity_recent(now_tick, db_status_led_last_radio_tick);
    }

    bool is_led_on = db_status_led_on_state_from_gpio_level(gpio_get_level(DB_STATUS_LED_GPIO));
    if (should_led_be_on != is_led_on) {
        int gpio_level_to_set = db_status_led_gpio_level_from_on_state(should_led_be_on);
        ESP_LOGI(TAG, "Status LED should: %s - GPIO reported: %s - setting GPIO to: %d",
                 should_led_be_on ? "ON" : "OFF", is_led_on ? "ON" : "OFF", gpio_level_to_set);
        gpio_set_level(DB_STATUS_LED_GPIO, gpio_level_to_set);
    }
#endif
}
