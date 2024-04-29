//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_TYPES_H
#define FASTMAVLINK_TYPES_H


#include <stdint.h>
#include "../fastmavlink_config.h"
#include "fastmavlink_protocol.h"


typedef enum {
    FASTMAVLINK_MESSAGE_ENTRY_FLAGS_HAS_TARGET_SYSTEM = 1,
    FASTMAVLINK_MESSAGE_ENTRY_FLAGS_HAS_TARGET_COMPONENT = 2,
} fmav_message_entry_flags_e;


typedef struct _fmav_message_entry {
#ifdef FASTMAVLINK_MESSAGE_ENTRY_MSGID24BIT
    uint32_t msgid:24; // works and reduces RAM, but is it good?
#else
    uint32_t msgid; // :24; works and reduces RAM, but is it good?
#endif
    uint8_t crc_extra;
    uint8_t payload_max_len;
    uint8_t flags; // see FASTMAVLINK_MESSAGE_ENTRY_FLAGS
    uint8_t target_system_ofs;
    uint8_t target_component_ofs;
} fmav_message_entry_t;


typedef enum {
    FASTMAVLINK_PARSE_RESULT_NONE = 0,
    FASTMAVLINK_PARSE_RESULT_HAS_HEADER,
    FASTMAVLINK_PARSE_RESULT_OK,
    FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN,
    FASTMAVLINK_PARSE_RESULT_LENGTH_ERROR, // cannot actually happen, length can be less or larger than PAYLOAD_LEN_MAX in v2
    FASTMAVLINK_PARSE_RESULT_CRC_ERROR,
    FASTMAVLINK_PARSE_RESULT_SIGNATURE_ERROR,
} fmav_parse_result_e;


typedef struct _fmav_result {
//result of parser or check, whichever was last
    uint8_t res; // see FASTMAVLINK_PARSE_RESULT
//result of parser, or packing into frame_buf
    uint16_t frame_len;
//result of check, or packing into frame_buf
    uint32_t msgid;
    uint8_t sysid;
    uint8_t compid;
    uint8_t target_sysid;
    uint8_t target_compid;
    uint8_t crc_extra;
    uint8_t payload_max_len;
} fmav_result_t;


typedef struct _fmav_signature {
    uint8_t linkid;
    uint64_t timestamp:48;
    uint64_t signature:48;
} fmav_signature_t;


typedef enum {
    FASTMAVLINK_INCOMPAT_FLAGS_SIGNED = 0x01,
} fmav_incompat_flags_e;


typedef struct _fmav_message {
    uint8_t magic;
    uint8_t len;
    uint8_t incompat_flags; // only v2, see FASTMAVLINK_INCOMPAT_FLAGS
    uint8_t compat_flags;  // only v2
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    union {
        uint32_t msgid:24; // only uint8_t in v1
        uint8_t msgid_a[3];
    };
    uint8_t payload[FASTMAVLINK_PAYLOAD_LEN_MAX];
    union {
        uint16_t checksum;
        uint8_t checksum_a[2];
    };
    union {
        fmav_signature_t signature;
        uint8_t signature_a[FASTMAVLINK_SIGNATURE_LEN];
    };
//helper fields
    uint8_t res; // see FASTMAVLINK_PARSE_RESULT
    uint8_t target_sysid;
    uint8_t target_compid;
    uint8_t crc_extra;
    uint8_t payload_max_len;
} fmav_message_t;


typedef enum {
    FASTMAVLINK_PARSE_STATE_IDLE = 0,
    FASTMAVLINK_PARSE_STATE_LEN,
    FASTMAVLINK_PARSE_STATE_INCOMPAT_FLAGS,
    FASTMAVLINK_PARSE_STATE_COMPAT_FLAGS,
    FASTMAVLINK_PARSE_STATE_SEQ,
    FASTMAVLINK_PARSE_STATE_SYSID,
    FASTMAVLINK_PARSE_STATE_COMPID,
    FASTMAVLINK_PARSE_STATE_MSGID_1,
    FASTMAVLINK_PARSE_STATE_MSGID_2,
    FASTMAVLINK_PARSE_STATE_MSGID_3,
    FASTMAVLINK_PARSE_STATE_PAYLOAD,
    FASTMAVLINK_PARSE_STATE_CHECKSUM_1,
    FASTMAVLINK_PARSE_STATE_CHECKSUM_2,
    FASTMAVLINK_PARSE_STATE_SIGNATURE,

    FASTMAVLINK_FASTPARSE_STATE_FRAME,
} fmav_parse_state_e;


// can be used for both receiving and transmitting
typedef struct _fmav_status {
//parser, receive
    uint8_t rx_state; // parser state, see FASTMAVLINK_FASTPARSE_STATE
    uint16_t rx_cnt; // number of chars parsed
    uint16_t rx_header_len;
    uint16_t rx_frame_len; // total length of frame
    uint16_t rx_crc;
//send, emit
    uint8_t tx_seq; // is used in pack,encode, and is then incremented to ready for next use
} fmav_status_t;


// for the sake of convenience
FASTMAVLINK_PACK(
typedef struct _fmav_param_union {
    union {
        float p_float; // type = 9 = MAV_PARAM_TYPE_REAL32 = MAV_PARAM_EXT_TYPE_REAL32
        int32_t p_int32; // type = 6 = MAV_PARAM_TYPE_INT32 = MAV_PARAM_EXT_TYPE_INT32
        uint32_t p_uint32; // type = 5 = MAV_PARAM_TYPE_UINT32 = MAV_PARAM_EXT_TYPE_UINT32
        int16_t p_int16; // type = 4 = MAV_PARAM_TYPE_INT16 = MAV_PARAM_EXT_TYPE_INT16
        uint16_t p_uint16; // type = 3 = MAV_PARAM_TYPE_UINT16 = MAV_PARAM_EXT_TYPE_UINT16
        int8_t p_int8; // type = 2 = MAV_PARAM_TYPE_INT8 = MAV_PARAM_EXT_TYPE_INT8
        uint8_t p_uint8; // type = 1 = MAV_PARAM_TYPE_UINT8 = MAV_PARAM_EXT_TYPE_UINT8
        uint8_t a[4];
    };
    uint8_t type;
}) fmav_param_union_t;


// for the sake of convenience
typedef struct _fmav_param_entry {
    void* ptr; // pointer to variable storing the parameter value
    uint8_t type; // type of parameter (MAV_PARAM_TYPE_XXX)
    const char* name; // this is a C string, including a terminating '\0', do not confuse with MAVLink's param_id
} fmav_param_entry_t;


#endif // FASTMAVLINK_TYPES_H
