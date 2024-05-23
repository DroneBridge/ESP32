//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PLAY_TUNE_H
#define FASTMAVLINK_MSG_PLAY_TUNE_H


//----------------------------------------
//-- Message PLAY_TUNE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_play_tune_t {
    uint8_t target_system;
    uint8_t target_component;
    char tune[30];
    char tune2[200];
}) fmav_play_tune_t;


#define FASTMAVLINK_MSG_ID_PLAY_TUNE  258

#define FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX  232
#define FASTMAVLINK_MSG_PLAY_TUNE_CRCEXTRA  187

#define FASTMAVLINK_MSG_PLAY_TUNE_FLAGS  3
#define FASTMAVLINK_MSG_PLAY_TUNE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PLAY_TUNE_TARGET_COMPONENT_OFS  1

#define FASTMAVLINK_MSG_PLAY_TUNE_FRAME_LEN_MAX  257

#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE_NUM  30 // number of elements in array
#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE_LEN  30 // length of array = number of bytes
#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE2_NUM  200 // number of elements in array
#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE2_LEN  200 // length of array = number of bytes

#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TARGET_COMPONENT_OFS  1
#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE_OFS  2
#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE2_OFS  32


//----------------------------------------
//-- Message PLAY_TUNE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* tune, const char* tune2,
    fmav_status_t* _status)
{
    fmav_play_tune_t* _payload = (fmav_play_tune_t*)_msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->tune), tune, sizeof(char)*30);
    memcpy(&(_payload->tune2), tune2, sizeof(char)*200);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_PLAY_TUNE;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_PLAY_TUNE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_play_tune_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_play_tune_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->tune, _payload->tune2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* tune, const char* tune2,
    fmav_status_t* _status)
{
    fmav_play_tune_t* _payload = (fmav_play_tune_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->tune), tune, sizeof(char)*30);
    memcpy(&(_payload->tune2), tune2, sizeof(char)*200);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PLAY_TUNE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PLAY_TUNE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PLAY_TUNE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PLAY_TUNE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_play_tune_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_play_tune_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->tune, _payload->tune2,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* tune, const char* tune2,
    fmav_status_t* _status)
{
    fmav_play_tune_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.tune), tune, sizeof(char)*30);
    memcpy(&(_payload.tune2), tune2, sizeof(char)*200);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PLAY_TUNE,
        FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PLAY_TUNE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_play_tune_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PLAY_TUNE,
        FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PLAY_TUNE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PLAY_TUNE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_play_tune_decode(fmav_play_tune_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_play_tune_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_play_tune_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_play_tune_get_field_tune_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[2]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_play_tune_get_field_tune(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE_NUM) return 0;
    return ((char*)&(msg->payload[2]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_play_tune_get_field_tune2_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[32]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_play_tune_get_field_tune2(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE2_NUM) return 0;
    return ((char*)&(msg->payload[32]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PLAY_TUNE  258

#define mavlink_play_tune_t  fmav_play_tune_t

#define MAVLINK_MSG_ID_PLAY_TUNE_LEN  232
#define MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN  32
#define MAVLINK_MSG_ID_258_LEN  232
#define MAVLINK_MSG_ID_258_MIN_LEN  32

#define MAVLINK_MSG_ID_PLAY_TUNE_CRC  187
#define MAVLINK_MSG_ID_258_CRC  187

#define MAVLINK_MSG_PLAY_TUNE_FIELD_TUNE_LEN 30
#define MAVLINK_MSG_PLAY_TUNE_FIELD_TUNE2_LEN 200


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_play_tune_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, const char* tune, const char* tune2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_play_tune_pack(
        _msg, sysid, compid,
        target_system, target_component, tune, tune2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_play_tune_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_play_tune_t* _payload)
{
    return mavlink_msg_play_tune_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->tune, _payload->tune2);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_play_tune_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* tune, const char* tune2)
{
    return fmav_msg_play_tune_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, tune, tune2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_play_tune_decode(const mavlink_message_t* msg, mavlink_play_tune_t* payload)
{
    fmav_msg_play_tune_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PLAY_TUNE_H
