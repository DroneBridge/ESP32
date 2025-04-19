/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2025 Wolfgang Christl
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

#ifndef DB_LED_STRIP_H
#define DB_LED_STRIP_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "common/common.h"

// LED strip types supported
typedef enum {
    LED_STRIP_WS2812 = 0,    // GRB
    LED_STRIP_WS2811 = 1,    // RGB (also works for WS2815)
    LED_STRIP_WS2814 = 2,    // RGBW
    LED_STRIP_WS2814A = 3,   // WRGB
} led_strip_type_t;

/**
 * @brief Initialize the LED strip driver with RMT peripheral
 * 
 * @param gpio_pin GPIO pin connected to the LED strip data line
 * @param led_count Number of LEDs in the strip
 * @param led_type Type of LED strip (WS2812, WS2811/15, WS2814, WS2814A)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t db_led_strip_init(uint8_t gpio_pin, uint16_t led_count, led_strip_type_t led_type);

/**
 * @brief Set the enable state of the LED strip controller
 * 
 * @param enabled true to enable, false to disable
 */
void db_led_strip_set_enabled(bool enabled);

/**
 * @brief Get the enable state of the LED strip controller
 * 
 * @return true if enabled, false if disabled
 */
bool db_led_strip_is_enabled(void);

/**
 * @brief Process a MAVLink DEBUG_VECT message for RGB data
 * 
 * @param msg Pointer to the MAVLink message
 * @return true if message was handled
 */
bool db_led_strip_process_debug_vect(const fmav_message_t* msg);

/**
 * @brief Run a test sequence on the LED strip (Red -> Green -> Blue -> Off)
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t db_led_strip_test(void);

/**
 * @brief Run a test sequence on the LED strip in a separate FreeRTOS task
 * 
 * @param pvParameters Task parameters (not used)
 */
void db_led_strip_test_task(void *pvParameters);

/**
 * @brief Deinitialize the LED strip driver
 */
void db_led_strip_deinit(void);

#endif // DB_LED_STRIP_H