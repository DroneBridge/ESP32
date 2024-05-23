//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AUTH_KEY_H
#define FASTMAVLINK_MSG_AUTH_KEY_H


//----------------------------------------
//-- Message AUTH_KEY
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_auth_key_t {
    char key[32];
}) fmav_auth_key_t;


#define FASTMAVLINK_MSG_ID_AUTH_KEY  7

#define FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_AUTH_KEY_CRCEXTRA  119

#define FASTMAVLINK_MSG_AUTH_KEY_FLAGS  0
#define FASTMAVLINK_MSG_AUTH_KEY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AUTH_KEY_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AUTH_KEY_FRAME_LEN_MAX  57

#define FASTMAVLINK_MSG_AUTH_KEY_FIELD_KEY_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_AUTH_KEY_FIELD_KEY_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_AUTH_KEY_FIELD_KEY_OFS  0


//----------------------------------------
//-- Message AUTH_KEY pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_auth_key_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const char* key,
    fmav_status_t* _status)
{
    fmav_auth_key_t* _payload = (fmav_auth_key_t*)_msg->payload;


    memcpy(&(_payload->key), key, sizeof(char)*32);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AUTH_KEY;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AUTH_KEY_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_auth_key_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_auth_key_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_auth_key_pack(
        _msg, sysid, compid,
        _payload->key,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_auth_key_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const char* key,
    fmav_status_t* _status)
{
    fmav_auth_key_t* _payload = (fmav_auth_key_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);


    memcpy(&(_payload->key), key, sizeof(char)*32);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AUTH_KEY;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTH_KEY >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTH_KEY >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTH_KEY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_auth_key_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_auth_key_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_auth_key_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->key,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_auth_key_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const char* key,
    fmav_status_t* _status)
{
    fmav_auth_key_t _payload;


    memcpy(&(_payload.key), key, sizeof(char)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AUTH_KEY,
        FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTH_KEY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_auth_key_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_auth_key_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AUTH_KEY,
        FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTH_KEY_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AUTH_KEY decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_auth_key_decode(fmav_auth_key_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTH_KEY_PAYLOAD_LEN_MAX);
#endif
}





FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_auth_key_get_field_key_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_auth_key_get_field_key(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AUTH_KEY_FIELD_KEY_NUM) return 0;
    return ((char*)&(msg->payload[0]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AUTH_KEY  7

#define mavlink_auth_key_t  fmav_auth_key_t

#define MAVLINK_MSG_ID_AUTH_KEY_LEN  32
#define MAVLINK_MSG_ID_AUTH_KEY_MIN_LEN  32
#define MAVLINK_MSG_ID_7_LEN  32
#define MAVLINK_MSG_ID_7_MIN_LEN  32

#define MAVLINK_MSG_ID_AUTH_KEY_CRC  119
#define MAVLINK_MSG_ID_7_CRC  119

#define MAVLINK_MSG_AUTH_KEY_FIELD_KEY_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_auth_key_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const char* key)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_auth_key_pack(
        _msg, sysid, compid,
        key,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_auth_key_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_auth_key_t* _payload)
{
    return mavlink_msg_auth_key_pack(
        sysid,
        compid,
        _msg,
        _payload->key);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_auth_key_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* key)
{
    return fmav_msg_auth_key_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        key,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_auth_key_decode(const mavlink_message_t* msg, mavlink_auth_key_t* payload)
{
    fmav_msg_auth_key_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AUTH_KEY_H
