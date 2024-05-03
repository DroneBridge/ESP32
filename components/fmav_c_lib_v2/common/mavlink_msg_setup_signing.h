//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SETUP_SIGNING_H
#define FASTMAVLINK_MSG_SETUP_SIGNING_H


//----------------------------------------
//-- Message SETUP_SIGNING
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_setup_signing_t {
    uint64_t initial_timestamp;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t secret_key[32];
}) fmav_setup_signing_t;


#define FASTMAVLINK_MSG_ID_SETUP_SIGNING  256

#define FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA  71

#define FASTMAVLINK_MSG_SETUP_SIGNING_FLAGS  3
#define FASTMAVLINK_MSG_SETUP_SIGNING_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_SETUP_SIGNING_TARGET_COMPONENT_OFS  9

#define FASTMAVLINK_MSG_SETUP_SIGNING_FRAME_LEN_MAX  67

#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_INITIAL_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_TARGET_COMPONENT_OFS  9
#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_OFS  10


//----------------------------------------
//-- Message SETUP_SIGNING pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp,
    fmav_status_t* _status)
{
    fmav_setup_signing_t* _payload = (fmav_setup_signing_t*)_msg->payload;

    _payload->initial_timestamp = initial_timestamp;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->secret_key), secret_key, sizeof(uint8_t)*32);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SETUP_SIGNING;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_setup_signing_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_setup_signing_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->secret_key, _payload->initial_timestamp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp,
    fmav_status_t* _status)
{
    fmav_setup_signing_t* _payload = (fmav_setup_signing_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->initial_timestamp = initial_timestamp;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->secret_key), secret_key, sizeof(uint8_t)*32);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SETUP_SIGNING;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SETUP_SIGNING >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SETUP_SIGNING >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_setup_signing_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_setup_signing_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->secret_key, _payload->initial_timestamp,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp,
    fmav_status_t* _status)
{
    fmav_setup_signing_t _payload;

    _payload.initial_timestamp = initial_timestamp;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.secret_key), secret_key, sizeof(uint8_t)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SETUP_SIGNING,
        FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_setup_signing_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SETUP_SIGNING,
        FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SETUP_SIGNING decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_setup_signing_decode(fmav_setup_signing_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_setup_signing_get_field_initial_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_setup_signing_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_setup_signing_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_setup_signing_get_field_secret_key_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[10]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_setup_signing_get_field_secret_key(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_NUM) return 0;
    return ((uint8_t*)&(msg->payload[10]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SETUP_SIGNING  256

#define mavlink_setup_signing_t  fmav_setup_signing_t

#define MAVLINK_MSG_ID_SETUP_SIGNING_LEN  42
#define MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN  42
#define MAVLINK_MSG_ID_256_LEN  42
#define MAVLINK_MSG_ID_256_MIN_LEN  42

#define MAVLINK_MSG_ID_SETUP_SIGNING_CRC  71
#define MAVLINK_MSG_ID_256_CRC  71

#define MAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_setup_signing_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_setup_signing_pack(
        _msg, sysid, compid,
        target_system, target_component, secret_key, initial_timestamp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_setup_signing_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_setup_signing_t* _payload)
{
    return mavlink_msg_setup_signing_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->secret_key, _payload->initial_timestamp);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_setup_signing_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp)
{
    return fmav_msg_setup_signing_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, secret_key, initial_timestamp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_setup_signing_decode(const mavlink_message_t* msg, mavlink_setup_signing_t* payload)
{
    fmav_msg_setup_signing_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SETUP_SIGNING_H
