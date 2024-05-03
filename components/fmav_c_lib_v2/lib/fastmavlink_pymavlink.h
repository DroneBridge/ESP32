//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// This provides wrappers for the
// pymavlink-mavgen mimicry.
//------------------------------

#pragma once
#ifndef FASTMAVLINK_PYMAVLINK_H
#define FASTMAVLINK_PYMAVLINK_H


//------------------------------
//-- Pymavlink types
//------------------------------

#define MAVLINK_WIRE_PROTOCOL_VERSION  "2.0"


#define MAVPACKED(__Declaration__)  FASTMAVLINK_PACK(__Declaration__)


#define MAVLINK_STX_MAVLINK1  0xFE
#define MAVLINK_STX  0xFD

#define MAVLINK_MAX_PAYLOAD_LEN  255
#define MAVLINK_NUM_HEADER_BYTES  10
#define MAVLINK_NUM_CHECKSUM_BYTES  2
#define MAVLINK_SIGNATURE_BLOCK_LEN  13
#define MAVLINK_MAX_PACKET_LEN (MAVLINK_NUM_HEADER_BYTES+MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+MAVLINK_SIGNATURE_BLOCK_LEN)

#define MAVLINK_IFLAG_SIGNED  0x01

#define _MAV_PAYLOAD(msg)  ((const uint8_t*)((msg)->payload))

#define mavlink_status_t  fmav_status_t
#define mavlink_message_t  fmav_message_t


#define MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM  1
#define MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT  2

#define mavlink_msg_entry_t  fmav_message_entry_t


#ifndef MAVLINK_COMM_NUM_BUFFERS
#define MAVLINK_COMM_NUM_BUFFERS  1
#endif


#if MAVLINK_COMM_NUM_BUFFERS > 0
typedef enum {
    MAVLINK_COMM_0 = 0,
#if MAVLINK_COMM_NUM_BUFFERS > 1
    MAVLINK_COMM_1,
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 2
    MAVLINK_COMM_2,
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 3
    MAVLINK_COMM_3,
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 4
    MAVLINK_COMM_4,
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 5
    MAVLINK_COMM_5,
#endif
} mavlink_channel_t;
#endif


MAVPACKED(
typedef struct param_union {
    union {
        float param_float;
        int32_t param_int32;
        uint32_t param_uint32;
        int16_t param_int16;
        uint16_t param_uint16;
        int8_t param_int8;
        uint8_t param_uint8;
        uint8_t bytes[4];
    };
    uint8_t type;
}) mavlink_param_union_t;


typedef enum {
    MAVLINK_FRAMING_INCOMPLETE = 0,
    MAVLINK_FRAMING_OK,
    MAVLINK_FRAMING_BAD_CRC,
    MAVLINK_FRAMING_BAD_SIGNATURE
} mavlink_framing_t;


//------------------------------
//-- Pymavlink wrappers
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void crc_init(uint16_t* crc)
{
    fmav_crc_init(crc);
}


FASTMAVLINK_FUNCTION_DECORATOR void crc_accumulate(uint8_t data, uint16_t* crc)
{
    fmav_crc_accumulate(crc, data);
}


FASTMAVLINK_FUNCTION_DECORATOR void crc_accumulate_buffer(uint16_t* crc, const char* buf, uint16_t len)
{
    fmav_crc_accumulate_buf(crc, (const uint8_t*)buf, len);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t crc_calculate(const uint8_t* buf, uint16_t len)
{
    return fmav_crc_calculate(buf, len);
}


FASTMAVLINK_FUNCTION_DECORATOR const mavlink_msg_entry_t* mavlink_get_msg_entry(uint32_t msgid)
{
    return (mavlink_msg_entry_t*)fmav_get_message_entry(msgid);
}


#if MAVLINK_COMM_NUM_BUFFERS > 0

// must be called with chan in range
FASTMAVLINK_FUNCTION_DECORATOR mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
    FASTMAVLINK_RAM_SECTION fmav_status_t _status[MAVLINK_COMM_NUM_BUFFERS];
    return &_status[chan];
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_reset_channel_status(uint8_t chan)
{
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) return;
    fmav_status_reset(mavlink_get_channel_status(chan));
}


// must be called with chan in range
FASTMAVLINK_FUNCTION_DECORATOR mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan)
{
    FASTMAVLINK_RAM_SECTION mavlink_message_t _buf[MAVLINK_COMM_NUM_BUFFERS];
    return &_buf[chan];
}


// the pymavlink mavlink_parse_char() function returns an "obscure" status, we ignore it here
FASTMAVLINK_FUNCTION_DECORATOR uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* msg, mavlink_status_t* r_status)
{
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) return 0;
    return fmav_parse_to_msg_wbuf(msg, (uint8_t*)mavlink_get_channel_buffer(chan), mavlink_get_channel_status(chan), c);
}

#endif // MAVLINK_COMM_NUM_BUFFERS > 0


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_to_send_buffer(uint8_t* buf, mavlink_message_t* msg)
{
    return fmav_msg_to_frame_buf(buf, msg);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t mavlink_get_target_system(mavlink_message_t* msg)
{
    return fmav_msg_get_target_sysid(msg);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t mavlink_get_target_component(mavlink_message_t* msg)
{
    return fmav_msg_get_target_compid(msg);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_reset_status(mavlink_status_t* status)
{
    fmav_status_reset(status);
}


// return FRAMING_INCOMPLETE, FRAMING_BAD_CRC, FRAMING_BAD_SIGNATURE, or FRAMING_OK
FASTMAVLINK_FUNCTION_DECORATOR uint8_t mavlink_parse_nextchar(mavlink_message_t* msg, mavlink_status_t* status, uint8_t c)
{
    // returns NONE, HAS_HEADER, MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
    switch (fmav_parse_to_msg(msg, status, c)) {
        case FASTMAVLINK_PARSE_RESULT_OK: return MAVLINK_FRAMING_OK;
        case FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN: return MAVLINK_FRAMING_BAD_CRC;
        case FASTMAVLINK_PARSE_RESULT_LENGTH_ERROR: return MAVLINK_FRAMING_BAD_CRC;
        case FASTMAVLINK_PARSE_RESULT_CRC_ERROR: return MAVLINK_FRAMING_BAD_CRC;
        case FASTMAVLINK_PARSE_RESULT_SIGNATURE_ERROR: return MAVLINK_FRAMING_BAD_SIGNATURE;
    }
    return MAVLINK_FRAMING_INCOMPLETE;
}


#endif  // FASTMAVLINK_PYMAVLINK_H
