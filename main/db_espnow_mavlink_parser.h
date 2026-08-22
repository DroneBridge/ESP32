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

#ifndef DB_ESP32_DB_ESPNOW_MAVLINK_PARSER_H
#define DB_ESP32_DB_ESPNOW_MAVLINK_PARSER_H

#include <stdbool.h>
#include <stdint.h>

#include "db_espnow_limits.h"
#include "db_mavlink_parser.h"

/** Runtime parser state associated with one ESP-NOW source MAC address. */
typedef struct {
    bool in_use;
    bool has_last_sequence;
    uint8_t source_mac[DB_ESPNOW_MAC_ADDR_LEN];
    uint32_t last_sequence_number;
    db_mavlink_parser_t parser;
} db_espnow_mavlink_parser_peer_t;

/** Fixed-size table containing the MAVLink parser state of each ESP-NOW peer. */
typedef struct {
    db_espnow_mavlink_parser_peer_t peers[DB_ESPNOW_MAX_BROADCAST_PEERS];
} db_espnow_mavlink_parser_table_t;

/**
 * Initializes an ESP-NOW MAVLink parser table and removes all peer state.
 *
 * @param table Parser table to initialize.
 */
void db_espnow_mavlink_parser_table_init(db_espnow_mavlink_parser_table_t *table);

/**
 * Selects the parser for a source MAC and updates its ESP-NOW sequence state.
 * A non-contiguous sequence resets only the receive side of that source's
 * MAVLink parser before the current packet is accepted.
 *
 * @param table Parser table containing the source-specific parser states.
 * @param source_mac MAC address of the ESP-NOW source.
 * @param sequence_number Sequence number from the received ESP-NOW packet.
 * @param sequence_tracking_enabled Whether the sequence number represents
 *                                  every packet in the source stream.
 * @param sequence_gap Set to true when the source sequence is not contiguous.
 * @return Parser belonging to the source, or NULL when arguments are invalid
 *         or the fixed table has no free entry.
 */
db_mavlink_parser_t *db_espnow_mavlink_parser_table_select(
        db_espnow_mavlink_parser_table_t *table,
        const uint8_t source_mac[DB_ESPNOW_MAC_ADDR_LEN],
        uint32_t sequence_number,
        bool sequence_tracking_enabled,
        bool *sequence_gap);

#endif // DB_ESP32_DB_ESPNOW_MAVLINK_PARSER_H
