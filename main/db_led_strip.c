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

#include <string.h>
#include <esp_log.h>
#include <driver/rmt_tx.h>
#include <driver/gpio.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "db_led_strip.h"
#include "common/common.h"

static const char *TAG = "DB_LED_STRIP";

// LED strip configurations
#define LED_STRIP_RMT_RESOLUTION_HZ 10000000 // 10MHz resolution for RMT channel
#define LED_STRIP_DEFAULT_TIMEOUT_MS 1000     // Default timeout for LED strip operations

// WS2812 timing parameters (in nanoseconds)
#define WS2812_T0H_NS 350   // Logic 0 high time
#define WS2812_T0L_NS 900   // Logic 0 low time
#define WS2812_T1H_NS 900   // Logic 1 high time
#define WS2812_T1L_NS 350   // Logic 1 low time
#define WS2812_RESET_US 280 // Reset time in microseconds

// Color component order defines
typedef enum {
    COLOR_ORDER_RGB,
    COLOR_ORDER_GRB,
    COLOR_ORDER_RGBW,
    COLOR_ORDER_WRGB,
} led_color_order_t;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t white;  // Only used for RGBW strips
} led_color_t;

typedef struct {
    rmt_channel_handle_t rmt_chan;
    rmt_encoder_handle_t encoder;
    rmt_transmit_config_t tx_config;
    uint16_t led_count;
    led_color_t *led_buffer;
    led_strip_type_t led_type;
    led_color_order_t color_order;
    uint8_t bytes_per_led;
    bool is_enabled;
} led_strip_t;

static led_strip_t led_strip = {0};

// RMT encoder for WS2812/WS2811/WS2815/WS2814 LED protocols
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    uint32_t reset_code;
    uint8_t encoded_buf[4];  // Intermediate buffer to translate colors
} led_strip_encoder_t;

// Converts RGB values to byte sequence based on LED type color order
static void rgb_to_bytes(uint8_t red, uint8_t green, uint8_t blue, uint8_t white, 
                         uint8_t *bytes, led_color_order_t color_order) {
    switch (color_order) {
        case COLOR_ORDER_RGB:
            bytes[0] = red;
            bytes[1] = green;
            bytes[2] = blue;
            break;
        case COLOR_ORDER_GRB: // WS2812
            bytes[0] = green;
            bytes[1] = red;
            bytes[2] = blue;
            break;
        case COLOR_ORDER_RGBW: // WS2814
            bytes[0] = red;
            bytes[1] = green;
            bytes[2] = blue;
            bytes[3] = white;
            break;
        case COLOR_ORDER_WRGB: // WS2814A
            bytes[0] = white;
            bytes[1] = red;
            bytes[2] = green;
            bytes[3] = blue;
            break;
    }
}

// Encode LED strip pixels into RMT symbols
static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                  const void *primary_data, size_t data_size,
                                  rmt_encode_state_t *ret_state) {
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET; // Initialize session state
    rmt_encode_state_t state = RMT_ENCODING_RESET;         // Initialize state
    size_t encoded_symbols = 0;
    led_color_t *led_colors = (led_color_t *)primary_data;
    uint16_t num_leds_to_encode = data_size / sizeof(led_color_t); // Calculate how many LEDs are in the data buffer

    // ESP_LOGD(TAG, "Encoding %d LEDs", num_leds_to_encode); // <-- Removed: Unsafe to log from RMT encoder (ISR context)

    // Convert entire buffer to byte sequence based on LED color order first
    // This ensures we process the entire buffer as one unit
    uint8_t *bytes_buffer = malloc(num_leds_to_encode * led_strip.bytes_per_led);
    if (!bytes_buffer) {
        ESP_LOGE(TAG, "Failed to allocate bytes buffer");
        if (ret_state) {
            *ret_state = RMT_ENCODING_RESET;
        }
        return 0;
    }

    // Populate the bytes buffer with all LED color data
    for (uint16_t i = 0; i < num_leds_to_encode; i++) {
        rgb_to_bytes(led_colors[i].red, led_colors[i].green, led_colors[i].blue, led_colors[i].white,
                     &bytes_buffer[i * led_strip.bytes_per_led], led_strip.color_order);
    }

    // Encode the entire byte buffer at once
    size_t symbols = led_encoder->bytes_encoder->encode(led_encoder->bytes_encoder, channel,
                                                         bytes_buffer,
                                                         num_leds_to_encode * led_strip.bytes_per_led,
                                                         &session_state);
    encoded_symbols += symbols;
    free(bytes_buffer);

    // Check if encoding completed or needs more memory
    if (session_state & RMT_ENCODING_COMPLETE) {
        state |= RMT_ENCODING_COMPLETE;
    } else if (session_state & RMT_ENCODING_MEM_FULL) {
        state |= RMT_ENCODING_MEM_FULL;
    }

    if (ret_state) {
        *ret_state = state;
    }

    return encoded_symbols;
}

// Delete the LED strip encoder
static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder) {
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    if (led_encoder->bytes_encoder) {
        ESP_RETURN_ON_ERROR(rmt_del_encoder(led_encoder->bytes_encoder), TAG, "Failed to delete bytes encoder");
    }
    free(led_encoder);
    return ESP_OK;
}

// Reset method for LED strip encoder
static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder) {
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    if (led_encoder->bytes_encoder) {
        ESP_RETURN_ON_ERROR(rmt_encoder_reset(led_encoder->bytes_encoder), TAG, 
                           "Failed to reset bytes encoder");
    }
    return ESP_OK;
}

// Create a new LED strip encoder
static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder, 
                                          const uint32_t t0h_ticks, const uint32_t t0l_ticks,
                                          const uint32_t t1h_ticks, const uint32_t t1l_ticks,
                                          const uint32_t reset_ticks) {
    esp_err_t ret = ESP_OK;
    
    led_strip_encoder_t *led_encoder = calloc(1, sizeof(led_strip_encoder_t));
    if (!led_encoder) {
        ESP_LOGE(TAG, "Failed to allocate LED strip encoder");
        return ESP_ERR_NO_MEM;
    }
    
    // Configure the encoder functions
    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;
    
    // Create bytes encoder for bit-banging
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .duration0 = t0h_ticks,
            .level0 = 1,
            .duration1 = t0l_ticks,
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = t1h_ticks,
            .level0 = 1,
            .duration1 = t1l_ticks,
            .level1 = 0,
        },
        .flags.msb_first = 1,  // WS2812 transfers MSB first
    };
    
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder),
                      TAG, "Failed to create bytes encoder");
                      
    // Setup the reset code
    led_encoder->reset_code = reset_ticks;
    
    *ret_encoder = &led_encoder->base;
    return ret;
}

esp_err_t db_led_strip_init(uint8_t gpio_pin, uint16_t led_count, led_strip_type_t led_type) {
    esp_err_t ret = ESP_OK;
    
    // Check parameters
    if (gpio_pin >= SOC_GPIO_PIN_COUNT) {
        ESP_LOGE(TAG, "Invalid GPIO number");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (led_count == 0) {
        ESP_LOGE(TAG, "LED count must be greater than 0");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clean up if already initialized
    db_led_strip_deinit();
    
    // Set LED parameters based on type
    switch (led_type) {
        case LED_STRIP_WS2812:  // WS2812 - GRB ordering
            led_strip.color_order = COLOR_ORDER_GRB;
            led_strip.bytes_per_led = 3;
            break;
        case LED_STRIP_WS2811:  // WS2811/WS2815 - RGB ordering
            led_strip.color_order = COLOR_ORDER_RGB;
            led_strip.bytes_per_led = 3;
            break;
        case LED_STRIP_WS2814:  // WS2814 - RGBW ordering
            led_strip.color_order = COLOR_ORDER_RGBW;
            led_strip.bytes_per_led = 4;
            break;
        case LED_STRIP_WS2814A: // WS2814A - WRGB ordering
            led_strip.color_order = COLOR_ORDER_WRGB;
            led_strip.bytes_per_led = 4;
            break;
        default:
            ESP_LOGE(TAG, "Unsupported LED strip type");
            return ESP_ERR_INVALID_ARG;
    }
    
    // Save configuration
    led_strip.led_count = led_count;
    led_strip.led_type = led_type;
    led_strip.is_enabled = false;
    
    // Allocate LED buffer
    led_strip.led_buffer = calloc(led_count, sizeof(led_color_t));
    if (!led_strip.led_buffer) {
        ESP_LOGE(TAG, "Failed to allocate LED buffer memory");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize RMT driver for LED strip
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = gpio_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 128, // Increased from 64 to handle more LEDs per transaction
        .trans_queue_depth = 4,  // Depth of internal transaction queue
    };
    
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &led_strip.rmt_chan),
                       TAG, "Failed to create RMT TX channel");
    
    // Convert nanoseconds to RMT ticks
    uint32_t t0h_ticks = (uint32_t)((float)WS2812_T0H_NS * LED_STRIP_RMT_RESOLUTION_HZ / 1e9f);
    uint32_t t0l_ticks = (uint32_t)((float)WS2812_T0L_NS * LED_STRIP_RMT_RESOLUTION_HZ / 1e9f);
    uint32_t t1h_ticks = (uint32_t)((float)WS2812_T1H_NS * LED_STRIP_RMT_RESOLUTION_HZ / 1e9f);
    uint32_t t1l_ticks = (uint32_t)((float)WS2812_T1L_NS * LED_STRIP_RMT_RESOLUTION_HZ / 1e9f);
    uint32_t reset_ticks = (uint32_t)((float)WS2812_RESET_US * 1000 * LED_STRIP_RMT_RESOLUTION_HZ / 1e9f);
    
    // Create LED strip encoder
    ESP_RETURN_ON_ERROR(rmt_new_led_strip_encoder(&led_strip.encoder, t0h_ticks, t0l_ticks,
                                                t1h_ticks, t1l_ticks, reset_ticks),
                      TAG, "Failed to create LED strip encoder");
    
    // Configure transmit parameters
    led_strip.tx_config = (rmt_transmit_config_t) {
        .loop_count = 0, // No loop
        .flags.eot_level = 0, // Keep output level low when idle
    };
    
    // Enable the RMT driver
    ESP_RETURN_ON_ERROR(rmt_enable(led_strip.rmt_chan), TAG, "Failed to enable RMT channel");
    
    ESP_LOGI(TAG, "LED strip initialized with %d LEDs on GPIO %d", led_count, gpio_pin);
    led_strip.is_enabled = true;
    
    // Clear all LEDs
    for (int i = 0; i < led_count; i++) {
        led_strip.led_buffer[i] = (led_color_t){0, 0, 0, 0};
    }
    
    // Update the strip to turn off all LEDs
    ESP_RETURN_ON_ERROR(rmt_transmit(led_strip.rmt_chan, led_strip.encoder,
                                   led_strip.led_buffer, sizeof(led_color_t) * led_count,
                                   &led_strip.tx_config),
                     TAG, "Failed to clear LED strip");
    
    return ret;
}

void db_led_strip_set_enabled(bool enabled) {
    led_strip.is_enabled = enabled;
    
    // If disabling, turn off all LEDs
    if (!enabled && led_strip.led_buffer) {
        for (int i = 0; i < led_strip.led_count; i++) {
            led_strip.led_buffer[i] = (led_color_t){0, 0, 0, 0};
        }
        
        if (led_strip.rmt_chan) {
            rmt_transmit(led_strip.rmt_chan, led_strip.encoder,
                       led_strip.led_buffer, sizeof(led_color_t) * led_strip.led_count,
                       &led_strip.tx_config);
        }
    }
}

bool db_led_strip_is_enabled(void) {
    return led_strip.is_enabled;
}

void db_led_strip_deinit(void) {
    // Turn off all LEDs first
    db_led_strip_set_enabled(false);
    
    // Clean up RMT resources
    if (led_strip.rmt_chan) {
        rmt_disable(led_strip.rmt_chan);
        rmt_del_channel(led_strip.rmt_chan);
        led_strip.rmt_chan = NULL;
    }
    
    if (led_strip.encoder) {
        rmt_del_encoder(led_strip.encoder);
        led_strip.encoder = NULL;
    }
    
    // Free LED buffer
    if (led_strip.led_buffer) {
        free(led_strip.led_buffer);
        led_strip.led_buffer = NULL;
    }
}

// Set a specific LED to the given color
static bool set_led_color(uint16_t index, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
    if (!led_strip.is_enabled || !led_strip.led_buffer) {
        return false;
    }
    
    if (index >= led_strip.led_count) {
        ESP_LOGW(TAG, "LED index %d out of range (0-%d)", index, led_strip.led_count - 1);
        return false;
    }
    
    led_strip.led_buffer[index].red = red;
    led_strip.led_buffer[index].green = green;
    led_strip.led_buffer[index].blue = blue;
    led_strip.led_buffer[index].white = white;
    
    return true;
}

// Update the LED strip with the current buffer contents
static bool update_strip(void) {
    if (!led_strip.is_enabled || !led_strip.rmt_chan || !led_strip.encoder) {
        return false;
    }

    // Reset the encoder before each transmission to ensure clean state
    rmt_encoder_reset(led_strip.encoder);

    esp_err_t ret = rmt_transmit(led_strip.rmt_chan, led_strip.encoder,
                               led_strip.led_buffer, sizeof(led_color_t) * led_strip.led_count,
                               &led_strip.tx_config);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update LED strip: %s", esp_err_to_name(ret));
        return false;
    }
    // Wait for the transmission to complete
    ret = rmt_tx_wait_all_done(led_strip.rmt_chan, LED_STRIP_DEFAULT_TIMEOUT_MS);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed waiting for RMT transmission: %s", esp_err_to_name(ret));
        // Don't necessarily return false here, the transmission might have partially succeeded
    }

    return true;
}

// Process DEBUG_VECT message for LED control
bool db_led_strip_process_debug_vect(const fmav_message_t* msg) {
    if (!led_strip.is_enabled || !msg) {
        return false;
    }
    
    // Decode the DEBUG_VECT message
    fmav_debug_vect_t debug_vect;
    fmav_msg_debug_vect_decode(&debug_vect, msg);
    
    // Check if this is an RGB or RGBW message
    bool is_led_message = false;
    uint16_t led_index = 0;
    uint8_t white = 0; // Default white value
    
    // Check if name starts with "rgb"
    if (strncmp(debug_vect.name, "rgb", 3) == 0) {
        is_led_message = true;
        
        // Check if this is an RGB message (r,g,b values in x,y,z) or RGBW message
        bool is_rgbw = false;
        if (strstr(debug_vect.name, "rgbw") == debug_vect.name) {
            // RGBW format: name starts with "rgbw"
            is_rgbw = true;
            
            // Format: "rgbwX" where X is optional LED index
            if (strlen(debug_vect.name) > 4) {
                led_index = atoi(&debug_vect.name[4]);
            }
            
            // In RGBW mode, white value is stored in lower byte of time_usec field
            white = (uint8_t)(debug_vect.time_usec & 0xFF);
            
            ESP_LOGD(TAG, "RGBW message detected, white=%d", white);
        } else {
            // Regular RGB format: name starts with "rgb"
            // Format: "rgbX" where X is optional LED index
            if (strlen(debug_vect.name) > 3) {
                led_index = atoi(&debug_vect.name[3]);
            }
        }
        
        // Extract RGB values - convert from normalized floats (0.0-1.0) to raw (0-255)
        uint8_t red = (uint8_t)(debug_vect.x * 255.0f);
        uint8_t green = (uint8_t)(debug_vect.y * 255.0f);
        uint8_t blue = (uint8_t)(debug_vect.z * 255.0f);
        
        // Check if it's for a specific LED
        if ((is_rgbw && strlen(debug_vect.name) > 4) || 
            (!is_rgbw && strlen(debug_vect.name) > 3)) {
            // Update a specific LED
            if (led_index < led_strip.led_count) {
                if (is_rgbw) {
                    ESP_LOGD(TAG, "Setting LED %d to RGBW(%d,%d,%d,%d)", 
                             led_index, red, green, blue, white);
                } else {
                    ESP_LOGD(TAG, "Setting LED %d to RGB(%d,%d,%d)", 
                             led_index, red, green, blue);
                }
                
                if (set_led_color(led_index, red, green, blue, white)) {
                    update_strip();
                    return true;
                }
            } else {
                ESP_LOGW(TAG, "LED index %d out of range (0-%d)", 
                         led_index, led_strip.led_count - 1);
            }
        } else {
            // Set all LEDs to the same color
            if (is_rgbw) {
                ESP_LOGD(TAG, "Setting all LEDs to RGBW(%d,%d,%d,%d)", 
                         red, green, blue, white);
            } else {
                ESP_LOGD(TAG, "Setting all LEDs to RGB(%d,%d,%d)", 
                         red, green, blue);
            }
            
            for (uint16_t i = 0; i < led_strip.led_count; i++) {
                set_led_color(i, red, green, blue, white);
            }
            
            update_strip();
            return true;
        }
    }
    
    return is_led_message;
}

// Run a test sequence on the LED strip
esp_err_t db_led_strip_test(void) {
    if (!led_strip.is_enabled || !led_strip.led_buffer || !led_strip.rmt_chan || !led_strip.encoder) {
        ESP_LOGW(TAG, "LED strip not enabled or initialized for test.");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Running LED strip test sequence...");

    // Red
    for (uint16_t i = 0; i < led_strip.led_count; i++) {
        set_led_color(i, 255, 0, 0, 0);
    }
    if (!update_strip()) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(500));

    // Green
    for (uint16_t i = 0; i < led_strip.led_count; i++) {
        set_led_color(i, 0, 255, 0, 0);
    }
    if (!update_strip()) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(500));

    // Blue
    for (uint16_t i = 0; i < led_strip.led_count; i++) {
        set_led_color(i, 0, 0, 255, 0);
    }
    if (!update_strip()) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // White channel test (only for RGBW/WRGB strips)
    if (led_strip.bytes_per_led == 4) {
        ESP_LOGI(TAG, "Testing white channel for RGBW strip");
        for (uint16_t i = 0; i < led_strip.led_count; i++) {
            set_led_color(i, 0, 0, 0, 255);
        }
        if (!update_strip()) return ESP_FAIL;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Off
    for (uint16_t i = 0; i < led_strip.led_count; i++) {
        set_led_color(i, 0, 0, 0, 0);
    }
    if (!update_strip()) return ESP_FAIL;
    
    ESP_LOGI(TAG, "LED strip test sequence complete.");
    return ESP_OK;
}

// Task-safe version of the LED strip test for running in a separate FreeRTOS task
void db_led_strip_test_task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting LED strip test sequence in task...");
    
    if (!led_strip.is_enabled || !led_strip.led_buffer || !led_strip.rmt_chan || !led_strip.encoder) {
        ESP_LOGW(TAG, "LED strip not enabled or initialized for test.");
        vTaskDelete(NULL);
        return;
    }

    // Red
    for (uint16_t i = 0; i < led_strip.led_count; i++) {
        set_led_color(i, 255, 0, 0, 0);
    }
    update_strip();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Green
    for (uint16_t i = 0; i < led_strip.led_count; i++) {
        set_led_color(i, 0, 255, 0, 0);
    }
    update_strip();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Blue
    for (uint16_t i = 0; i < led_strip.led_count; i++) {
        set_led_color(i, 0, 0, 255, 0);
    }
    update_strip();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // White channel test (only for RGBW/WRGB strips)
    if (led_strip.bytes_per_led == 4) {
        ESP_LOGI(TAG, "Testing white channel for RGBW strip");
        for (uint16_t i = 0; i < led_strip.led_count; i++) {
            set_led_color(i, 0, 0, 0, 255);
        }
        update_strip();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Off
    for (uint16_t i = 0; i < led_strip.led_count; i++) {
        set_led_color(i, 0, 0, 0, 0);
    }
    update_strip();
    
    ESP_LOGI(TAG, "LED strip test sequence complete.");
    vTaskDelete(NULL);  // Properly delete the task when done
}