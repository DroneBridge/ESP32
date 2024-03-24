#include "mavlink_parser.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include "ardupilotmega/mavlink.h"
#include "mavlink_helpers.h"

#define MAVLINK_STREAM_BUFFER_BYTES_SIZE 128

static const char *TAG = "MAVLINK_PARSER";

volatile StreamBufferHandle_t xStreamBufferMavlinkSerial;

void mavlink_parser_task(void *pvParameters) {
    size_t xReceivedBytes = 0;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t incoming_bytes[MAVLINK_STREAM_BUFFER_BYTES_SIZE];
    int delay_timer_cnt = 0;
    while (1) {
        xReceivedBytes = xStreamBufferReceive(xStreamBufferMavlinkSerial, (void *)incoming_bytes, sizeof(incoming_bytes), 0);
        if (xReceivedBytes > 0) {
            for (size_t incoming_byte = 0; incoming_byte < xReceivedBytes; incoming_byte++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, incoming_bytes[incoming_byte], &msg, &status)) {
                    // ESP_LOGW(TAG, "MSG ID: %d", msg.msgid);
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                            int32_t longitude = mavlink_msg_global_position_int_get_lon(&msg);
                            int32_t latitude = mavlink_msg_global_position_int_get_lat(&msg);
                            // ESP_LOGI(TAG, "Current coordinates: (%ld, %ld)", longitude, latitude);
                            break;
                        }
                        case MAVLINK_MSG_ID_GPS_RAW_INT: {
                            int32_t longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
                            int32_t latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
                            uint8_t satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
                            // ESP_LOGI(TAG, "Current coordinates: (%ld, %ld); Sattelites: %d", longitude, latitude, satellites_visible);
                            break;
                        }
                        default:
                            break;
                    }
                }
            }
        }
        if (delay_timer_cnt > 10000) {
            delay_timer_cnt = 0;
            vTaskDelay(10 / portTICK_PERIOD_MS);
        } else {
            delay_timer_cnt++;
        }
    }
}

void mavlink_parse_start() {
    xStreamBufferMavlinkSerial = xStreamBufferCreate(MAVLINK_STREAM_BUFFER_BYTES_SIZE, 1);  // TODO: add error handling
    xTaskCreatePinnedToCore(&mavlink_parser_task, "mavlink_parser_task", 40960, NULL, 5, NULL, 1);
}