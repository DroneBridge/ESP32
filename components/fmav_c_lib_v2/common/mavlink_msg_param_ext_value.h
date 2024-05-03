//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_EXT_VALUE_H
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_H


//----------------------------------------
//-- Message PARAM_EXT_VALUE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_ext_value_t {
    uint16_t param_count;
    uint16_t param_index;
    char param_id[16];
    char param_value[128];
    uint8_t param_type;
}) fmav_param_ext_value_t;


#define FASTMAVLINK_MSG_ID_PARAM_EXT_VALUE  322

#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX  149
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_CRCEXTRA  243

#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FLAGS  0
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FRAME_LEN_MAX  174

#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_ID_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_ID_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_NUM  128 // number of elements in array
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_LEN  128 // length of array = number of bytes

#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_COUNT_OFS  0
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_INDEX_OFS  2
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_ID_OFS  4
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_OFS  20
#define FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_TYPE_OFS  148


//----------------------------------------
//-- Message PARAM_EXT_VALUE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_value_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, const char* param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_ext_value_t* _payload = (fmav_param_ext_value_t*)_msg->payload;

    _payload->param_count = param_count;
    _payload->param_index = param_index;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload->param_value), param_value, sizeof(char)*128);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_PARAM_EXT_VALUE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_VALUE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_value_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_value_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_value_pack(
        _msg, sysid, compid,
        _payload->param_id, _payload->param_value, _payload->param_type, _payload->param_count, _payload->param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_value_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, const char* param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_ext_value_t* _payload = (fmav_param_ext_value_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_count = param_count;
    _payload->param_index = param_index;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload->param_value), param_value, sizeof(char)*128);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_EXT_VALUE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_VALUE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_VALUE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_VALUE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_value_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_value_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_value_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->param_id, _payload->param_value, _payload->param_type, _payload->param_count, _payload->param_index,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_value_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, const char* param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_ext_value_t _payload;

    _payload.param_count = param_count;
    _payload.param_index = param_index;
    _payload.param_type = param_type;
    memcpy(&(_payload.param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload.param_value), param_value, sizeof(char)*128);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_EXT_VALUE,
        FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_VALUE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_value_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_value_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_EXT_VALUE,
        FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_VALUE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_EXT_VALUE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_ext_value_decode(fmav_param_ext_value_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_EXT_VALUE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_value_get_field_param_count(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_value_get_field_param_index(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_ext_value_get_field_param_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[148]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_param_ext_value_get_field_param_id_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[4]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_param_ext_value_get_field_param_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_ID_NUM) return 0;
    return ((char*)&(msg->payload[4]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_param_ext_value_get_field_param_value_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_param_ext_value_get_field_param_value(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_NUM) return 0;
    return ((char*)&(msg->payload[20]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_EXT_VALUE  322

#define mavlink_param_ext_value_t  fmav_param_ext_value_t

#define MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN  149
#define MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN  149
#define MAVLINK_MSG_ID_322_LEN  149
#define MAVLINK_MSG_ID_322_MIN_LEN  149

#define MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC  243
#define MAVLINK_MSG_ID_322_CRC  243

#define MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_ID_LEN 16
#define MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_LEN 128


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_value_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const char* param_id, const char* param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_ext_value_pack(
        _msg, sysid, compid,
        param_id, param_value, param_type, param_count, param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_value_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_param_ext_value_t* _payload)
{
    return mavlink_msg_param_ext_value_pack(
        sysid,
        compid,
        _msg,
        _payload->param_id, _payload->param_value, _payload->param_type, _payload->param_count, _payload->param_index);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_value_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, const char* param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
    return fmav_msg_param_ext_value_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        param_id, param_value, param_type, param_count, param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_ext_value_decode(const mavlink_message_t* msg, mavlink_param_ext_value_t* payload)
{
    fmav_msg_param_ext_value_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_EXT_VALUE_H
