/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2024 Wolfgang Christl
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

#ifndef DB_ESP32_DB_MAVLINK_PARSER_H
#define DB_ESP32_DB_MAVLINK_PARSER_H

#include <stdbool.h>

#include "lib/fastmavlink_types.h"

/** MAVLink receive and transmit state for one independent byte stream. */
typedef struct {
    fmav_status_t status;
    uint8_t frame_buf[FASTMAVLINK_FRAME_LEN_MAX];
} db_mavlink_parser_t;

/**
 * Returns whether a structurally complete MAVLink frame may be forwarded.
 *
 * Unknown message IDs are intentionally forwarded transparently because this
 * firmware only compiles the common MAVLink dialect and therefore cannot
 * validate the CRC extra of custom dialect messages. Such frames must never
 * be decoded or handled by the local DroneBridge endpoint.
 *
 * @param parse_result fastMavlink parse result for the completed frame.
 * @return true for validated or unknown-dialect frames, otherwise false.
 */
static inline bool db_mavlink_parse_result_is_forwardable(const uint8_t parse_result) {
    return parse_result == FASTMAVLINK_PARSE_RESULT_OK ||
           parse_result == FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN;
}

#endif // DB_ESP32_DB_MAVLINK_PARSER_H
