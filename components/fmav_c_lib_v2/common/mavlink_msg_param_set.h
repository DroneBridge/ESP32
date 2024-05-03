//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_SET_H
#define FASTMAVLINK_MSG_PARAM_SET_H


//----------------------------------------
//-- Message PARAM_SET
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_set_t {
    float param_value;
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
    uint8_t param_type;
}) fmav_param_set_t;


#define FASTMAVLINK_MSG_ID_PARAM_SET  23

#define FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX  23
#define FASTMAVLINK_MSG_PARAM_SET_CRCEXTRA  168

#define FASTMAVLINK_MSG_PARAM_SET_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_SET_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_PARAM_SET_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_PARAM_SET_FRAME_LEN_MAX  48

#define FASTMAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_PARAM_SET_FIELD_PARAM_VALUE_OFS  0
#define FASTMAVLINK_MSG_PARAM_SET_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_PARAM_SET_FIELD_TARGET_COMPONENT_OFS  5
#define FASTMAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_OFS  6
#define FASTMAVLINK_MSG_PARAM_SET_FIELD_PARAM_TYPE_OFS  22


//----------------------------------------
//-- Message PARAM_SET pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_set_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type,
    fmav_status_t* _status)
{
    fmav_param_set_t* _payload = (fmav_param_set_t*)_msg->payload;

    _payload->param_value = param_value;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_PARAM_SET;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_PARAM_SET_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_set_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_set_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_set_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_value, _payload->param_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_set_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type,
    fmav_status_t* _status)
{
    fmav_param_set_t* _payload = (fmav_param_set_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_value = param_value;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_SET;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_SET >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_SET >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_SET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_set_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_set_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_set_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_value, _payload->param_type,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_set_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type,
    fmav_status_t* _status)
{
    fmav_param_set_t _payload;

    _payload.param_value = param_value;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.param_type = param_type;
    memcpy(&(_payload.param_id), param_id, sizeof(char)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_SET,
        FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_SET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_set_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_set_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_SET,
        FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_SET_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_SET decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_set_decode(fmav_param_set_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_SET_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_param_set_get_field_param_value(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_set_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_set_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_set_get_field_param_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_param_set_get_field_param_id_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[6]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_param_set_get_field_param_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_NUM) return 0;
    return ((char*)&(msg->payload[6]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_SET  23

#define mavlink_param_set_t  fmav_param_set_t

#define MAVLINK_MSG_ID_PARAM_SET_LEN  23
#define MAVLINK_MSG_ID_PARAM_SET_MIN_LEN  23
#define MAVLINK_MSG_ID_23_LEN  23
#define MAVLINK_MSG_ID_23_MIN_LEN  23

#define MAVLINK_MSG_ID_PARAM_SET_CRC  168
#define MAVLINK_MSG_ID_23_CRC  168

#define MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_set_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_set_pack(
        _msg, sysid, compid,
        target_system, target_component, param_id, param_value, param_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_set_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_param_set_t* _payload)
{
    return mavlink_msg_param_set_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_value, _payload->param_type);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_set_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type)
{
    return fmav_msg_param_set_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, param_id, param_value, param_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_set_decode(const mavlink_message_t* msg, mavlink_param_set_t* payload)
{
    fmav_msg_param_set_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_SET_H
