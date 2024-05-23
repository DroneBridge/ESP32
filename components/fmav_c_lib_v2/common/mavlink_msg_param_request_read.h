//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_REQUEST_READ_H
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_H


//----------------------------------------
//-- Message PARAM_REQUEST_READ
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_request_read_t {
    int16_t param_index;
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
}) fmav_param_request_read_t;


#define FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ  20

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA  214

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_TARGET_COMPONENT_OFS  3

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FRAME_LEN_MAX  45

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_INDEX_OFS  0
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_TARGET_COMPONENT_OFS  3
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_OFS  4


//----------------------------------------
//-- Message PARAM_REQUEST_READ pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_request_read_t* _payload = (fmav_param_request_read_t*)_msg->payload;

    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_request_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_request_read_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_request_read_t* _payload = (fmav_param_request_read_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_request_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_request_read_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_request_read_t _payload;

    _payload.param_index = param_index;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.param_id), param_id, sizeof(char)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_request_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_REQUEST_READ decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_request_read_decode(fmav_param_request_read_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_param_request_read_get_field_param_index(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_request_read_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_request_read_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_param_request_read_get_field_param_id_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[4]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_param_request_read_get_field_param_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_NUM) return 0;
    return ((char*)&(msg->payload[4]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ  20

#define mavlink_param_request_read_t  fmav_param_request_read_t

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN  20
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_MIN_LEN  20
#define MAVLINK_MSG_ID_20_LEN  20
#define MAVLINK_MSG_ID_20_MIN_LEN  20

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_CRC  214
#define MAVLINK_MSG_ID_20_CRC  214

#define MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_request_read_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_request_read_pack(
        _msg, sysid, compid,
        target_system, target_component, param_id, param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_request_read_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_param_request_read_t* _payload)
{
    return mavlink_msg_param_request_read_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_request_read_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index)
{
    return fmav_msg_param_request_read_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, param_id, param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_request_read_decode(const mavlink_message_t* msg, mavlink_param_request_read_t* payload)
{
    fmav_msg_param_request_read_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_REQUEST_READ_H
