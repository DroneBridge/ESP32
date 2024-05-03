//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_MAP_RC_H
#define FASTMAVLINK_MSG_PARAM_MAP_RC_H


//----------------------------------------
//-- Message PARAM_MAP_RC
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_map_rc_t {
    float param_value0;
    float scale;
    float param_value_min;
    float param_value_max;
    int16_t param_index;
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
    uint8_t parameter_rc_channel_index;
}) fmav_param_map_rc_t;


#define FASTMAVLINK_MSG_ID_PARAM_MAP_RC  50

#define FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX  37
#define FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA  78

#define FASTMAVLINK_MSG_PARAM_MAP_RC_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_MAP_RC_TARGET_SYSTEM_OFS  18
#define FASTMAVLINK_MSG_PARAM_MAP_RC_TARGET_COMPONENT_OFS  19

#define FASTMAVLINK_MSG_PARAM_MAP_RC_FRAME_LEN_MAX  62

#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_ID_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_ID_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_VALUE0_OFS  0
#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_SCALE_OFS  4
#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_VALUE_MIN_OFS  8
#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_VALUE_MAX_OFS  12
#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_INDEX_OFS  16
#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_TARGET_SYSTEM_OFS  18
#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_TARGET_COMPONENT_OFS  19
#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_ID_OFS  20
#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAMETER_RC_CHANNEL_INDEX_OFS  36


//----------------------------------------
//-- Message PARAM_MAP_RC pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max,
    fmav_status_t* _status)
{
    fmav_param_map_rc_t* _payload = (fmav_param_map_rc_t*)_msg->payload;

    _payload->param_value0 = param_value0;
    _payload->scale = scale;
    _payload->param_value_min = param_value_min;
    _payload->param_value_max = param_value_max;
    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->parameter_rc_channel_index = parameter_rc_channel_index;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_PARAM_MAP_RC;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_map_rc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_map_rc_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index, _payload->parameter_rc_channel_index, _payload->param_value0, _payload->scale, _payload->param_value_min, _payload->param_value_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max,
    fmav_status_t* _status)
{
    fmav_param_map_rc_t* _payload = (fmav_param_map_rc_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_value0 = param_value0;
    _payload->scale = scale;
    _payload->param_value_min = param_value_min;
    _payload->param_value_max = param_value_max;
    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->parameter_rc_channel_index = parameter_rc_channel_index;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_MAP_RC;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_MAP_RC >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_MAP_RC >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_map_rc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_map_rc_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index, _payload->parameter_rc_channel_index, _payload->param_value0, _payload->scale, _payload->param_value_min, _payload->param_value_max,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max,
    fmav_status_t* _status)
{
    fmav_param_map_rc_t _payload;

    _payload.param_value0 = param_value0;
    _payload.scale = scale;
    _payload.param_value_min = param_value_min;
    _payload.param_value_max = param_value_max;
    _payload.param_index = param_index;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.parameter_rc_channel_index = parameter_rc_channel_index;
    memcpy(&(_payload.param_id), param_id, sizeof(char)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_MAP_RC,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_map_rc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_MAP_RC,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_MAP_RC decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_map_rc_decode(fmav_param_map_rc_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_param_map_rc_get_field_param_value0(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_param_map_rc_get_field_scale(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_param_map_rc_get_field_param_value_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_param_map_rc_get_field_param_value_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_param_map_rc_get_field_param_index(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_map_rc_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_map_rc_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[19]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_map_rc_get_field_parameter_rc_channel_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_param_map_rc_get_field_param_id_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_param_map_rc_get_field_param_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_ID_NUM) return 0;
    return ((char*)&(msg->payload[20]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_MAP_RC  50

#define mavlink_param_map_rc_t  fmav_param_map_rc_t

#define MAVLINK_MSG_ID_PARAM_MAP_RC_LEN  37
#define MAVLINK_MSG_ID_PARAM_MAP_RC_MIN_LEN  37
#define MAVLINK_MSG_ID_50_LEN  37
#define MAVLINK_MSG_ID_50_MIN_LEN  37

#define MAVLINK_MSG_ID_PARAM_MAP_RC_CRC  78
#define MAVLINK_MSG_ID_50_CRC  78

#define MAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_map_rc_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_map_rc_pack(
        _msg, sysid, compid,
        target_system, target_component, param_id, param_index, parameter_rc_channel_index, param_value0, scale, param_value_min, param_value_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_map_rc_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_param_map_rc_t* _payload)
{
    return mavlink_msg_param_map_rc_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index, _payload->parameter_rc_channel_index, _payload->param_value0, _payload->scale, _payload->param_value_min, _payload->param_value_max);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_map_rc_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max)
{
    return fmav_msg_param_map_rc_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, param_id, param_index, parameter_rc_channel_index, param_value0, scale, param_value_min, param_value_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_map_rc_decode(const mavlink_message_t* msg, mavlink_param_map_rc_t* payload)
{
    fmav_msg_param_map_rc_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_MAP_RC_H
