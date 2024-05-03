//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SYSTEM_TIME_H
#define FASTMAVLINK_MSG_SYSTEM_TIME_H


//----------------------------------------
//-- Message SYSTEM_TIME
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_system_time_t {
    uint64_t time_unix_usec;
    uint32_t time_boot_ms;
}) fmav_system_time_t;


#define FASTMAVLINK_MSG_ID_SYSTEM_TIME  2

#define FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX  12
#define FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA  137

#define FASTMAVLINK_MSG_SYSTEM_TIME_FLAGS  0
#define FASTMAVLINK_MSG_SYSTEM_TIME_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SYSTEM_TIME_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SYSTEM_TIME_FRAME_LEN_MAX  37



#define FASTMAVLINK_MSG_SYSTEM_TIME_FIELD_TIME_UNIX_USEC_OFS  0
#define FASTMAVLINK_MSG_SYSTEM_TIME_FIELD_TIME_BOOT_MS_OFS  8


//----------------------------------------
//-- Message SYSTEM_TIME pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_unix_usec, uint32_t time_boot_ms,
    fmav_status_t* _status)
{
    fmav_system_time_t* _payload = (fmav_system_time_t*)_msg->payload;

    _payload->time_unix_usec = time_unix_usec;
    _payload->time_boot_ms = time_boot_ms;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SYSTEM_TIME;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_system_time_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_system_time_pack(
        _msg, sysid, compid,
        _payload->time_unix_usec, _payload->time_boot_ms,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_unix_usec, uint32_t time_boot_ms,
    fmav_status_t* _status)
{
    fmav_system_time_t* _payload = (fmav_system_time_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_unix_usec = time_unix_usec;
    _payload->time_boot_ms = time_boot_ms;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SYSTEM_TIME;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SYSTEM_TIME >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SYSTEM_TIME >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_system_time_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_system_time_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_unix_usec, _payload->time_boot_ms,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_unix_usec, uint32_t time_boot_ms,
    fmav_status_t* _status)
{
    fmav_system_time_t _payload;

    _payload.time_unix_usec = time_unix_usec;
    _payload.time_boot_ms = time_boot_ms;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SYSTEM_TIME,
        FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_system_time_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SYSTEM_TIME,
        FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SYSTEM_TIME decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_system_time_decode(fmav_system_time_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_system_time_get_field_time_unix_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_system_time_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SYSTEM_TIME  2

#define mavlink_system_time_t  fmav_system_time_t

#define MAVLINK_MSG_ID_SYSTEM_TIME_LEN  12
#define MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN  12
#define MAVLINK_MSG_ID_2_LEN  12
#define MAVLINK_MSG_ID_2_MIN_LEN  12

#define MAVLINK_MSG_ID_SYSTEM_TIME_CRC  137
#define MAVLINK_MSG_ID_2_CRC  137




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_system_time_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_system_time_pack(
        _msg, sysid, compid,
        time_unix_usec, time_boot_ms,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_system_time_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_system_time_t* _payload)
{
    return mavlink_msg_system_time_pack(
        sysid,
        compid,
        _msg,
        _payload->time_unix_usec, _payload->time_boot_ms);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_system_time_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    return fmav_msg_system_time_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_unix_usec, time_boot_ms,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_system_time_decode(const mavlink_message_t* msg, mavlink_system_time_t* payload)
{
    fmav_msg_system_time_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SYSTEM_TIME_H
