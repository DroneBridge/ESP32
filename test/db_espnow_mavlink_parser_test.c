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

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "db_espnow_mavlink_parser.h"
#include "minimal/minimal.h"

#define TEST_MAX_CAPTURED_FRAMES 4U

typedef struct {
    uint16_t lengths[TEST_MAX_CAPTURED_FRAMES];
    uint8_t frames[TEST_MAX_CAPTURED_FRAMES][FASTMAVLINK_FRAME_LEN_MAX];
    size_t count;
} captured_frames_t;

/**
 * Creates a valid MAVLink heartbeat frame for the parser tests.
 *
 * @param buffer Destination frame buffer.
 * @param sysid MAVLink system ID to encode.
 * @return Number of bytes written to the frame buffer.
 */
static uint16_t create_heartbeat(uint8_t *buffer, uint8_t sysid) {
    fmav_status_t status = {0};
    return fmav_msg_heartbeat_pack_to_frame_buf(buffer, sysid, 1, 2, 3, 0, sysid, 4, &status);
}

/**
 * Feeds one ESP-NOW payload fragment into a source-specific MAVLink parser.
 *
 * @param parser Parser receiving the fragment.
 * @param data Fragment bytes.
 * @param length Number of bytes in the fragment.
 * @param captured Destination for complete parsed frames.
 */
static void feed_fragment(db_mavlink_parser_t *parser, const uint8_t *data, size_t length,
                           captured_frames_t *captured) {
    for (size_t i = 0; i < length; i++) {
        fmav_result_t result = {0};
        if (fmav_parse_and_check_to_frame_buf(&result, parser->frame_buf, &parser->status, data[i])) {
            assert(result.res == FASTMAVLINK_PARSE_RESULT_OK);
            assert(captured->count < TEST_MAX_CAPTURED_FRAMES);
            captured->lengths[captured->count] = result.frame_len;
            memcpy(captured->frames[captured->count], parser->frame_buf, result.frame_len);
            captured->count++;
        }
    }
}

/**
 * Verifies that interleaved fragments from two AIR units remain independent.
 */
static void test_interleaved_fragments_are_isolated(void) {
    const uint8_t mac_a[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x0A};
    const uint8_t mac_b[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x0B};
    uint8_t frame_a[FASTMAVLINK_FRAME_LEN_MAX] = {0};
    uint8_t frame_b[FASTMAVLINK_FRAME_LEN_MAX] = {0};
    const uint16_t length_a = create_heartbeat(frame_a, 11);
    const uint16_t length_b = create_heartbeat(frame_b, 22);
    const size_t split_a = length_a / 2U;
    const size_t split_b = length_b / 2U;
    bool sequence_gap = false;
    db_espnow_mavlink_parser_table_t table;
    captured_frames_t captured_a = {0};
    captured_frames_t captured_b = {0};

    db_espnow_mavlink_parser_table_init(&table);
    db_mavlink_parser_t *parser_a = db_espnow_mavlink_parser_table_select(&table, mac_a, 100U, true, &sequence_gap);
    assert(parser_a != NULL);
    assert(!sequence_gap);
    feed_fragment(parser_a, frame_a, split_a, &captured_a);

    db_mavlink_parser_t *parser_b = db_espnow_mavlink_parser_table_select(&table, mac_b, 700U, true, &sequence_gap);
    assert(parser_b != NULL);
    assert(!sequence_gap);
    feed_fragment(parser_b, frame_b, split_b, &captured_b);

    parser_a = db_espnow_mavlink_parser_table_select(&table, mac_a, 101U, true, &sequence_gap);
    assert(parser_a != NULL);
    assert(!sequence_gap);
    feed_fragment(parser_a, &frame_a[split_a], length_a - split_a, &captured_a);

    parser_b = db_espnow_mavlink_parser_table_select(&table, mac_b, 701U, true, &sequence_gap);
    assert(parser_b != NULL);
    assert(!sequence_gap);
    feed_fragment(parser_b, &frame_b[split_b], length_b - split_b, &captured_b);

    assert(captured_a.count == 1U);
    assert(captured_a.lengths[0] == length_a);
    assert(memcmp(captured_a.frames[0], frame_a, length_a) == 0);
    assert(captured_b.count == 1U);
    assert(captured_b.lengths[0] == length_b);
    assert(memcmp(captured_b.frames[0], frame_b, length_b) == 0);
}

/**
 * Verifies that a missing fragment resets only the affected source parser.
 */
static void test_sequence_gap_resets_only_affected_source(void) {
    const uint8_t mac_a[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x1A};
    const uint8_t mac_b[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x1B};
    uint8_t frame_a[FASTMAVLINK_FRAME_LEN_MAX] = {0};
    uint8_t frame_b[FASTMAVLINK_FRAME_LEN_MAX] = {0};
    const uint16_t length_a = create_heartbeat(frame_a, 31);
    const uint16_t length_b = create_heartbeat(frame_b, 32);
    bool sequence_gap = false;
    db_espnow_mavlink_parser_table_t table;
    captured_frames_t captured_a = {0};
    captured_frames_t captured_b = {0};

    db_espnow_mavlink_parser_table_init(&table);
    db_mavlink_parser_t *parser_a = db_espnow_mavlink_parser_table_select(&table, mac_a, 10U, true, &sequence_gap);
    assert(parser_a != NULL);
    assert(!sequence_gap);
    feed_fragment(parser_a, frame_a, length_a / 2U, &captured_a);

    db_mavlink_parser_t *parser_b = db_espnow_mavlink_parser_table_select(&table, mac_b, 40U, true, &sequence_gap);
    assert(parser_b != NULL);
    assert(!sequence_gap);
    feed_fragment(parser_b, frame_b, length_b, &captured_b);

    parser_a = db_espnow_mavlink_parser_table_select(&table, mac_a, 12U, true, &sequence_gap);
    assert(parser_a != NULL);
    assert(sequence_gap);
    feed_fragment(parser_a, frame_a, length_a, &captured_a);

    assert(captured_a.count == 1U);
    assert(captured_a.lengths[0] == length_a);
    assert(memcmp(captured_a.frames[0], frame_a, length_a) == 0);
    assert(captured_b.count == 1U);
    assert(captured_b.lengths[0] == length_b);
    assert(memcmp(captured_b.frames[0], frame_b, length_b) == 0);
}

/**
 * Verifies that the 32-bit ESP-NOW sequence number wraps without a false gap.
 */
static void test_sequence_wrap_is_contiguous(void) {
    const uint8_t mac[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x2A};
    bool sequence_gap = false;
    db_espnow_mavlink_parser_table_t table;

    db_espnow_mavlink_parser_table_init(&table);
    assert(db_espnow_mavlink_parser_table_select(&table, mac, UINT32_MAX, true, &sequence_gap) != NULL);
    assert(!sequence_gap);
    assert(db_espnow_mavlink_parser_table_select(&table, mac, 0U, true, &sequence_gap) != NULL);
    assert(!sequence_gap);
}

/**
 * Verifies that streams which omit non-data ESP-NOW packets can disable raw
 * sequence tracking without losing a partial MAVLink frame.
 */
static void test_sequence_tracking_can_be_disabled(void) {
    const uint8_t mac[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x3A};
    uint8_t frame[FASTMAVLINK_FRAME_LEN_MAX] = {0};
    const uint16_t frame_length = create_heartbeat(frame, 41);
    const size_t split = frame_length / 2U;
    bool sequence_gap = false;
    db_espnow_mavlink_parser_table_t table;
    captured_frames_t captured = {0};

    db_espnow_mavlink_parser_table_init(&table);
    db_mavlink_parser_t *parser = db_espnow_mavlink_parser_table_select(
            &table, mac, 100U, false, &sequence_gap);
    assert(parser != NULL);
    assert(!sequence_gap);
    feed_fragment(parser, frame, split, &captured);

    parser = db_espnow_mavlink_parser_table_select(&table, mac, 10U, false, &sequence_gap);
    assert(parser != NULL);
    assert(!sequence_gap);
    feed_fragment(parser, &frame[split], frame_length - split, &captured);

    assert(captured.count == 1U);
    assert(captured.lengths[0] == frame_length);
    assert(memcmp(captured.frames[0], frame, frame_length) == 0);
}

/**
 * Verifies that an AIR stream detects missing data while a GND stream can
 * skip its non-data internal telemetry without resetting its MAVLink parser.
 */
static void test_air_and_gnd_sequence_policies_are_independent(void) {
    const uint8_t air_mac[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x4A};
    const uint8_t gnd_mac[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x4B};
    uint8_t frame[FASTMAVLINK_FRAME_LEN_MAX] = {0};
    const uint16_t frame_length = create_heartbeat(frame, 51);
    const size_t split = frame_length / 2U;
    bool sequence_gap = false;
    db_espnow_mavlink_parser_table_t table;
    captured_frames_t captured_air = {0};
    captured_frames_t captured_gnd = {0};

    db_espnow_mavlink_parser_table_init(&table);
    db_mavlink_parser_t *air_parser = db_espnow_mavlink_parser_table_select(
            &table, air_mac, 100U, true, &sequence_gap);
    assert(air_parser != NULL);
    feed_fragment(air_parser, frame, split, &captured_air);

    db_mavlink_parser_t *gnd_parser = db_espnow_mavlink_parser_table_select(
            &table, gnd_mac, 100U, false, &sequence_gap);
    assert(gnd_parser != NULL);
    feed_fragment(gnd_parser, frame, split, &captured_gnd);

    air_parser = db_espnow_mavlink_parser_table_select(&table, air_mac, 102U, true, &sequence_gap);
    assert(air_parser != NULL);
    assert(sequence_gap);
    feed_fragment(air_parser, &frame[split], frame_length - split, &captured_air);

    gnd_parser = db_espnow_mavlink_parser_table_select(&table, gnd_mac, 102U, false, &sequence_gap);
    assert(gnd_parser != NULL);
    assert(!sequence_gap);
    feed_fragment(gnd_parser, &frame[split], frame_length - split, &captured_gnd);

    assert(captured_air.count == 0U);
    assert(captured_gnd.count == 1U);
    assert(captured_gnd.lengths[0] == frame_length);
    assert(memcmp(captured_gnd.frames[0], frame, frame_length) == 0);
}

/**
 * Verifies that the parser table rejects a source after all supported slots are used.
 */
static void test_parser_table_capacity(void) {
    bool sequence_gap = false;
    db_espnow_mavlink_parser_table_t table;

    db_espnow_mavlink_parser_table_init(&table);
    for (uint8_t i = 0; i < DB_ESPNOW_MAX_BROADCAST_PEERS; i++) {
        uint8_t mac[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x01, i};
        assert(db_espnow_mavlink_parser_table_select(&table, mac, i, true, &sequence_gap) != NULL);
        assert(!sequence_gap);
    }

    const uint8_t extra_mac[DB_ESPNOW_MAC_ADDR_LEN] = {0x02, 0x00, 0x00, 0x00, 0x02, 0x00};
    assert(db_espnow_mavlink_parser_table_select(&table, extra_mac, 0U, true, &sequence_gap) == NULL);
}

/**
 * Runs all ESP-NOW source-specific MAVLink parser unit tests.
 *
 * @return Zero when all assertions pass.
 */
int main(void) {
    test_interleaved_fragments_are_isolated();
    test_sequence_gap_resets_only_affected_source();
    test_sequence_wrap_is_contiguous();
    test_sequence_tracking_can_be_disabled();
    test_air_and_gnd_sequence_policies_are_independent();
    test_parser_table_capacity();
    return 0;
}
