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

#include <string.h>

#include "db_espnow_mavlink_parser.h"

/**
 * Resets the receive side of a MAVLink parser while preserving its transmit
 * sequence number for generated MAVLink responses.
 *
 * @param parser Parser state whose receive state shall be reset.
 */
static void db_espnow_mavlink_parser_reset_rx(db_mavlink_parser_t *parser) {
    const uint8_t tx_sequence = parser->status.tx_seq;
    memset(&parser->status, 0, sizeof(parser->status));
    parser->status.tx_seq = tx_sequence;
    memset(parser->frame_buf, 0, sizeof(parser->frame_buf));
}

/**
 * Finds an active parser table entry for a source MAC address.
 *
 * @param table Parser table to search.
 * @param source_mac MAC address to find.
 * @return Matching peer entry, or NULL when the source is not known.
 */
static db_espnow_mavlink_parser_peer_t *db_espnow_mavlink_parser_find_peer(
        db_espnow_mavlink_parser_table_t *table,
        const uint8_t source_mac[DB_ESPNOW_MAC_ADDR_LEN]) {
    for (uint8_t i = 0; i < DB_ESPNOW_MAX_BROADCAST_PEERS; i++) {
        if (table->peers[i].in_use &&
            memcmp(table->peers[i].source_mac, source_mac, DB_ESPNOW_MAC_ADDR_LEN) == 0) {
            return &table->peers[i];
        }
    }
    return NULL;
}

/**
 * Allocates and initializes the first unused parser table entry.
 *
 * @param table Parser table that owns the entries.
 * @param source_mac MAC address to associate with the new entry.
 * @return Newly initialized peer entry, or NULL when the table is full.
 */
static db_espnow_mavlink_parser_peer_t *db_espnow_mavlink_parser_add_peer(
        db_espnow_mavlink_parser_table_t *table,
        const uint8_t source_mac[DB_ESPNOW_MAC_ADDR_LEN]) {
    for (uint8_t i = 0; i < DB_ESPNOW_MAX_BROADCAST_PEERS; i++) {
        db_espnow_mavlink_parser_peer_t *peer = &table->peers[i];
        if (!peer->in_use) {
            memset(peer, 0, sizeof(*peer));
            memcpy(peer->source_mac, source_mac, DB_ESPNOW_MAC_ADDR_LEN);
            peer->in_use = true;
            return peer;
        }
    }
    return NULL;
}

/**
 * Initializes an ESP-NOW MAVLink parser table and removes all peer state.
 *
 * @param table Parser table to initialize.
 */
void db_espnow_mavlink_parser_table_init(db_espnow_mavlink_parser_table_t *table) {
    if (table != NULL) {
        memset(table, 0, sizeof(*table));
    }
}

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
        bool *sequence_gap) {
    if (table == NULL || source_mac == NULL || sequence_gap == NULL) {
        return NULL;
    }

    *sequence_gap = false;
    db_espnow_mavlink_parser_peer_t *peer = db_espnow_mavlink_parser_find_peer(table, source_mac);
    if (peer == NULL) {
        peer = db_espnow_mavlink_parser_add_peer(table, source_mac);
        if (peer == NULL) {
            return NULL;
        }
    }

    if (sequence_tracking_enabled && peer->has_last_sequence) {
        const uint32_t expected_sequence = peer->last_sequence_number + 1U;
        if (sequence_number != expected_sequence) {
            *sequence_gap = true;
            db_espnow_mavlink_parser_reset_rx(&peer->parser);
        }
    }

    if (sequence_tracking_enabled) {
        peer->last_sequence_number = sequence_number;
        peer->has_last_sequence = true;
    }
    return &peer->parser;
}
