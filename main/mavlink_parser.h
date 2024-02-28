#ifndef MAVLINK_PARSER_H
#define MAVLINK_PARSER_H

#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

/**
 * @brief Global Stream Buffer for incoming bytes to be parsed
 *
 */
extern volatile StreamBufferHandle_t xStreamBufferMavlinkSerial;

void mavlink_parse_start();

#endif  // MAVLINK_PARSER_H