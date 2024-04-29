//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_H
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_H


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_autopilot_state_for_gimbal_device_t {
    uint64_t time_boot_us;
    float q[4];
    uint32_t q_estimated_delay_us;
    float vx;
    float vy;
    float vz;
    uint32_t v_estimated_delay_us;
    float feed_forward_angular_velocity_z;
    uint16_t estimator_status;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t landed_state;
    float angular_velocity_z;
}) fmav_autopilot_state_for_gimbal_device_t;


#define FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE  286

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX  57
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA  210

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FLAGS  3
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_TARGET_SYSTEM_OFS  50
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_TARGET_COMPONENT_OFS  51

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FRAME_LEN_MAX  82

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_TIME_BOOT_US_OFS  0
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_OFS  8
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_ESTIMATED_DELAY_US_OFS  24
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_VX_OFS  28
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_VY_OFS  32
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_VZ_OFS  36
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_V_ESTIMATED_DELAY_US_OFS  40
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_FEED_FORWARD_ANGULAR_VELOCITY_Z_OFS  44
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_ESTIMATOR_STATUS_OFS  48
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_TARGET_SYSTEM_OFS  50
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_TARGET_COMPONENT_OFS  51
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_LANDED_STATE_OFS  52
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_ANGULAR_VELOCITY_Z_OFS  53


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_autopilot_state_for_gimbal_device_t* _payload = (fmav_autopilot_state_for_gimbal_device_t*)_msg->payload;

    _payload->time_boot_us = time_boot_us;
    _payload->q_estimated_delay_us = q_estimated_delay_us;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->v_estimated_delay_us = v_estimated_delay_us;
    _payload->feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    _payload->estimator_status = estimator_status;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->landed_state = landed_state;
    _payload->angular_velocity_z = angular_velocity_z;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_state_for_gimbal_device_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_state_for_gimbal_device_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->time_boot_us, _payload->q, _payload->q_estimated_delay_us, _payload->vx, _payload->vy, _payload->vz, _payload->v_estimated_delay_us, _payload->feed_forward_angular_velocity_z, _payload->estimator_status, _payload->landed_state, _payload->angular_velocity_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_autopilot_state_for_gimbal_device_t* _payload = (fmav_autopilot_state_for_gimbal_device_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_us = time_boot_us;
    _payload->q_estimated_delay_us = q_estimated_delay_us;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->v_estimated_delay_us = v_estimated_delay_us;
    _payload->feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    _payload->estimator_status = estimator_status;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->landed_state = landed_state;
    _payload->angular_velocity_z = angular_velocity_z;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_state_for_gimbal_device_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_state_for_gimbal_device_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->time_boot_us, _payload->q, _payload->q_estimated_delay_us, _payload->vx, _payload->vy, _payload->vz, _payload->v_estimated_delay_us, _payload->feed_forward_angular_velocity_z, _payload->estimator_status, _payload->landed_state, _payload->angular_velocity_z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_autopilot_state_for_gimbal_device_t _payload;

    _payload.time_boot_us = time_boot_us;
    _payload.q_estimated_delay_us = q_estimated_delay_us;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.v_estimated_delay_us = v_estimated_delay_us;
    _payload.feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    _payload.estimator_status = estimator_status;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.landed_state = landed_state;
    _payload.angular_velocity_z = angular_velocity_z;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_state_for_gimbal_device_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_autopilot_state_for_gimbal_device_decode(fmav_autopilot_state_for_gimbal_device_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_autopilot_state_for_gimbal_device_get_field_time_boot_us(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_autopilot_state_for_gimbal_device_get_field_q_estimated_delay_us(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_vx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_vy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_vz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_autopilot_state_for_gimbal_device_get_field_v_estimated_delay_us(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_feed_forward_angular_velocity_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_get_field_estimator_status(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_state_for_gimbal_device_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_state_for_gimbal_device_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[51]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_state_for_gimbal_device_get_field_landed_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[52]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_angular_velocity_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[53]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_autopilot_state_for_gimbal_device_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[8]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE  286

#define mavlink_autopilot_state_for_gimbal_device_t  fmav_autopilot_state_for_gimbal_device_t

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN  57
#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN  53
#define MAVLINK_MSG_ID_286_LEN  57
#define MAVLINK_MSG_ID_286_MIN_LEN  53

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC  210
#define MAVLINK_MSG_ID_286_CRC  210

#define MAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_state_for_gimbal_device_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state, float angular_velocity_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_autopilot_state_for_gimbal_device_pack(
        _msg, sysid, compid,
        target_system, target_component, time_boot_us, q, q_estimated_delay_us, vx, vy, vz, v_estimated_delay_us, feed_forward_angular_velocity_z, estimator_status, landed_state, angular_velocity_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_state_for_gimbal_device_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_autopilot_state_for_gimbal_device_t* _payload)
{
    return mavlink_msg_autopilot_state_for_gimbal_device_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->time_boot_us, _payload->q, _payload->q_estimated_delay_us, _payload->vx, _payload->vy, _payload->vz, _payload->v_estimated_delay_us, _payload->feed_forward_angular_velocity_z, _payload->estimator_status, _payload->landed_state, _payload->angular_velocity_z);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_state_for_gimbal_device_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state, float angular_velocity_z)
{
    return fmav_msg_autopilot_state_for_gimbal_device_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, time_boot_us, q, q_estimated_delay_us, vx, vy, vz, v_estimated_delay_us, feed_forward_angular_velocity_z, estimator_status, landed_state, angular_velocity_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_autopilot_state_for_gimbal_device_decode(const mavlink_message_t* msg, mavlink_autopilot_state_for_gimbal_device_t* payload)
{
    fmav_msg_autopilot_state_for_gimbal_device_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_H
