//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_H
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_H


//----------------------------------------
//-- Message GIMBAL_MANAGER_SET_PITCHYAW
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_manager_set_pitchyaw_t {
    uint32_t flags;
    float pitch;
    float yaw;
    float pitch_rate;
    float yaw_rate;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t gimbal_device_id;
}) fmav_gimbal_manager_set_pitchyaw_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW  287

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX  23
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_CRCEXTRA  1

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FLAGS  3
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_TARGET_SYSTEM_OFS  20
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_TARGET_COMPONENT_OFS  21

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FRAME_LEN_MAX  48



#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FIELD_FLAGS_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FIELD_PITCH_OFS  4
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FIELD_YAW_OFS  8
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FIELD_PITCH_RATE_OFS  12
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FIELD_YAW_RATE_OFS  16
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FIELD_TARGET_SYSTEM_OFS  20
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FIELD_TARGET_COMPONENT_OFS  21
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_FIELD_GIMBAL_DEVICE_ID_OFS  22


//----------------------------------------
//-- Message GIMBAL_MANAGER_SET_PITCHYAW pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_set_pitchyaw_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, float pitch, float yaw, float pitch_rate, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_set_pitchyaw_t* _payload = (fmav_gimbal_manager_set_pitchyaw_t*)_msg->payload;

    _payload->flags = flags;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->pitch_rate = pitch_rate;
    _payload->yaw_rate = yaw_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->gimbal_device_id = gimbal_device_id;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_set_pitchyaw_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_set_pitchyaw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_set_pitchyaw_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->flags, _payload->gimbal_device_id, _payload->pitch, _payload->yaw, _payload->pitch_rate, _payload->yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_set_pitchyaw_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, float pitch, float yaw, float pitch_rate, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_set_pitchyaw_t* _payload = (fmav_gimbal_manager_set_pitchyaw_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->flags = flags;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->pitch_rate = pitch_rate;
    _payload->yaw_rate = yaw_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->gimbal_device_id = gimbal_device_id;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_set_pitchyaw_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_set_pitchyaw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_set_pitchyaw_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->flags, _payload->gimbal_device_id, _payload->pitch, _payload->yaw, _payload->pitch_rate, _payload->yaw_rate,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_set_pitchyaw_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, float pitch, float yaw, float pitch_rate, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_set_pitchyaw_t _payload;

    _payload.flags = flags;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.pitch_rate = pitch_rate;
    _payload.yaw_rate = yaw_rate;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.gimbal_device_id = gimbal_device_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_set_pitchyaw_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_set_pitchyaw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_MANAGER_SET_PITCHYAW decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_manager_set_pitchyaw_decode(fmav_gimbal_manager_set_pitchyaw_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_manager_set_pitchyaw_get_field_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_set_pitchyaw_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_set_pitchyaw_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_set_pitchyaw_get_field_pitch_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_set_pitchyaw_get_field_yaw_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_set_pitchyaw_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_set_pitchyaw_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[21]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_set_pitchyaw_get_field_gimbal_device_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW  287

#define mavlink_gimbal_manager_set_pitchyaw_t  fmav_gimbal_manager_set_pitchyaw_t

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN  23
#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN  23
#define MAVLINK_MSG_ID_287_LEN  23
#define MAVLINK_MSG_ID_287_MIN_LEN  23

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC  1
#define MAVLINK_MSG_ID_287_CRC  1




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_set_pitchyaw_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_manager_set_pitchyaw_pack(
        _msg, sysid, compid,
        target_system, target_component, flags, gimbal_device_id, pitch, yaw, pitch_rate, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_set_pitchyaw_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gimbal_manager_set_pitchyaw_t* _payload)
{
    return mavlink_msg_gimbal_manager_set_pitchyaw_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->flags, _payload->gimbal_device_id, _payload->pitch, _payload->yaw, _payload->pitch_rate, _payload->yaw_rate);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_set_pitchyaw_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
    return fmav_msg_gimbal_manager_set_pitchyaw_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, flags, gimbal_device_id, pitch, yaw, pitch_rate, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_manager_set_pitchyaw_decode(const mavlink_message_t* msg, mavlink_gimbal_manager_set_pitchyaw_t* payload)
{
    fmav_msg_gimbal_manager_set_pitchyaw_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_MANAGER_SET_PITCHYAW_H
