//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_H
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_H


//----------------------------------------
//-- Message SET_ATTITUDE_TARGET
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_attitude_target_t {
    uint32_t time_boot_ms;
    float q[4];
    float body_roll_rate;
    float body_pitch_rate;
    float body_yaw_rate;
    float thrust;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t type_mask;
    float thrust_body[3];
}) fmav_set_attitude_target_t;


#define FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET  82

#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX  51
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA  49

#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FLAGS  3
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_TARGET_SYSTEM_OFS  36
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_TARGET_COMPONENT_OFS  37

#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FRAME_LEN_MAX  76

#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_Q_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_THRUST_BODY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_THRUST_BODY_LEN  12 // length of array = number of bytes

#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_Q_OFS  4
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_BODY_ROLL_RATE_OFS  20
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_BODY_PITCH_RATE_OFS  24
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_BODY_YAW_RATE_OFS  28
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_THRUST_OFS  32
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_TARGET_SYSTEM_OFS  36
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_TARGET_COMPONENT_OFS  37
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_TYPE_MASK_OFS  38
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_THRUST_BODY_OFS  39


//----------------------------------------
//-- Message SET_ATTITUDE_TARGET pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust, const float* thrust_body,
    fmav_status_t* _status)
{
    fmav_set_attitude_target_t* _payload = (fmav_set_attitude_target_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->body_roll_rate = body_roll_rate;
    _payload->body_pitch_rate = body_pitch_rate;
    _payload->body_yaw_rate = body_yaw_rate;
    _payload->thrust = thrust;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->type_mask = type_mask;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->thrust_body), thrust_body, sizeof(float)*3);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_attitude_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_attitude_target_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->type_mask, _payload->q, _payload->body_roll_rate, _payload->body_pitch_rate, _payload->body_yaw_rate, _payload->thrust, _payload->thrust_body,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust, const float* thrust_body,
    fmav_status_t* _status)
{
    fmav_set_attitude_target_t* _payload = (fmav_set_attitude_target_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->body_roll_rate = body_roll_rate;
    _payload->body_pitch_rate = body_pitch_rate;
    _payload->body_yaw_rate = body_yaw_rate;
    _payload->thrust = thrust;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->type_mask = type_mask;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->thrust_body), thrust_body, sizeof(float)*3);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_attitude_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_attitude_target_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->type_mask, _payload->q, _payload->body_roll_rate, _payload->body_pitch_rate, _payload->body_yaw_rate, _payload->thrust, _payload->thrust_body,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust, const float* thrust_body,
    fmav_status_t* _status)
{
    fmav_set_attitude_target_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.body_roll_rate = body_roll_rate;
    _payload.body_pitch_rate = body_pitch_rate;
    _payload.body_yaw_rate = body_yaw_rate;
    _payload.thrust = thrust;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.type_mask = type_mask;
    memcpy(&(_payload.q), q, sizeof(float)*4);
    memcpy(&(_payload.thrust_body), thrust_body, sizeof(float)*3);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_attitude_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SET_ATTITUDE_TARGET decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_attitude_target_decode(fmav_set_attitude_target_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_set_attitude_target_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_attitude_target_get_field_body_roll_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_attitude_target_get_field_body_pitch_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_attitude_target_get_field_body_yaw_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_attitude_target_get_field_thrust(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_attitude_target_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_attitude_target_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_attitude_target_get_field_type_mask(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_set_attitude_target_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[4]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_attitude_target_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[4]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_set_attitude_target_get_field_thrust_body_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[39]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_attitude_target_get_field_thrust_body(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_THRUST_BODY_NUM) return 0;
    return ((float*)&(msg->payload[39]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET  82

#define mavlink_set_attitude_target_t  fmav_set_attitude_target_t

#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_LEN  51
#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_MIN_LEN  39
#define MAVLINK_MSG_ID_82_LEN  51
#define MAVLINK_MSG_ID_82_MIN_LEN  39

#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_CRC  49
#define MAVLINK_MSG_ID_82_CRC  49

#define MAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_Q_LEN 4
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_THRUST_BODY_LEN 3


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_attitude_target_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust, const float* thrust_body)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_attitude_target_pack(
        _msg, sysid, compid,
        time_boot_ms, target_system, target_component, type_mask, q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, thrust_body,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_attitude_target_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_set_attitude_target_t* _payload)
{
    return mavlink_msg_set_attitude_target_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->type_mask, _payload->q, _payload->body_roll_rate, _payload->body_pitch_rate, _payload->body_yaw_rate, _payload->thrust, _payload->thrust_body);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_attitude_target_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust, const float* thrust_body)
{
    return fmav_msg_set_attitude_target_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, target_system, target_component, type_mask, q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, thrust_body,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_attitude_target_decode(const mavlink_message_t* msg, mavlink_set_attitude_target_t* payload)
{
    fmav_msg_set_attitude_target_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_H
