//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_H
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_H


//----------------------------------------
//-- Message CHANGE_OPERATOR_CONTROL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_change_operator_control_t {
    uint8_t target_system;
    uint8_t control_request;
    uint8_t version;
    char passkey[25];
}) fmav_change_operator_control_t;


#define FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL  5

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_CRCEXTRA  217

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FLAGS  1
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FRAME_LEN_MAX  53

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FIELD_PASSKEY_NUM  25 // number of elements in array
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FIELD_PASSKEY_LEN  25 // length of array = number of bytes

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FIELD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FIELD_CONTROL_REQUEST_OFS  1
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FIELD_VERSION_OFS  2
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FIELD_PASSKEY_OFS  3


//----------------------------------------
//-- Message CHANGE_OPERATOR_CONTROL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t control_request, uint8_t version, const char* passkey,
    fmav_status_t* _status)
{
    fmav_change_operator_control_t* _payload = (fmav_change_operator_control_t*)_msg->payload;

    _payload->target_system = target_system;
    _payload->control_request = control_request;
    _payload->version = version;
    memcpy(&(_payload->passkey), passkey, sizeof(char)*25);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL;
    _msg->target_sysid = target_system;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_change_operator_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_change_operator_control_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->control_request, _payload->version, _payload->passkey,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t control_request, uint8_t version, const char* passkey,
    fmav_status_t* _status)
{
    fmav_change_operator_control_t* _payload = (fmav_change_operator_control_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->control_request = control_request;
    _payload->version = version;
    memcpy(&(_payload->passkey), passkey, sizeof(char)*25);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_change_operator_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_change_operator_control_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->control_request, _payload->version, _payload->passkey,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t control_request, uint8_t version, const char* passkey,
    fmav_status_t* _status)
{
    fmav_change_operator_control_t _payload;

    _payload.target_system = target_system;
    _payload.control_request = control_request;
    _payload.version = version;
    memcpy(&(_payload.passkey), passkey, sizeof(char)*25);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_change_operator_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CHANGE_OPERATOR_CONTROL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_change_operator_control_decode(fmav_change_operator_control_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_change_operator_control_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_change_operator_control_get_field_control_request(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_change_operator_control_get_field_version(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_change_operator_control_get_field_passkey_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[3]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_change_operator_control_get_field_passkey(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FIELD_PASSKEY_NUM) return 0;
    return ((char*)&(msg->payload[3]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL  5

#define mavlink_change_operator_control_t  fmav_change_operator_control_t

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN  28
#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN  28
#define MAVLINK_MSG_ID_5_LEN  28
#define MAVLINK_MSG_ID_5_MIN_LEN  28

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC  217
#define MAVLINK_MSG_ID_5_CRC  217

#define MAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FIELD_PASSKEY_LEN 25


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_change_operator_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t control_request, uint8_t version, const char* passkey)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_change_operator_control_pack(
        _msg, sysid, compid,
        target_system, control_request, version, passkey,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_change_operator_control_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_change_operator_control_t* _payload)
{
    return mavlink_msg_change_operator_control_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->control_request, _payload->version, _payload->passkey);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_change_operator_control_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t control_request, uint8_t version, const char* passkey)
{
    return fmav_msg_change_operator_control_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, control_request, version, passkey,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_change_operator_control_decode(const mavlink_message_t* msg, mavlink_change_operator_control_t* payload)
{
    fmav_msg_change_operator_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_H
