//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_H
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_H


//----------------------------------------
//-- Message CONTROL_SYSTEM_STATE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_control_system_state_t {
    uint64_t time_usec;
    float x_acc;
    float y_acc;
    float z_acc;
    float x_vel;
    float y_vel;
    float z_vel;
    float x_pos;
    float y_pos;
    float z_pos;
    float airspeed;
    float vel_variance[3];
    float pos_variance[3];
    float q[4];
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
}) fmav_control_system_state_t;


#define FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE  146

#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX  100
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_CRCEXTRA  103

#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FLAGS  0
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FRAME_LEN_MAX  125

#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_VEL_VARIANCE_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_VEL_VARIANCE_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_POS_VARIANCE_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_POS_VARIANCE_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_X_ACC_OFS  8
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Y_ACC_OFS  12
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Z_ACC_OFS  16
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_X_VEL_OFS  20
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Y_VEL_OFS  24
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Z_VEL_OFS  28
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_X_POS_OFS  32
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Y_POS_OFS  36
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Z_POS_OFS  40
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_AIRSPEED_OFS  44
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_VEL_VARIANCE_OFS  48
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_POS_VARIANCE_OFS  60
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Q_OFS  72
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_ROLL_RATE_OFS  88
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_PITCH_RATE_OFS  92
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_YAW_RATE_OFS  96


//----------------------------------------
//-- Message CONTROL_SYSTEM_STATE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float x_acc, float y_acc, float z_acc, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float airspeed, const float* vel_variance, const float* pos_variance, const float* q, float roll_rate, float pitch_rate, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_control_system_state_t* _payload = (fmav_control_system_state_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->x_acc = x_acc;
    _payload->y_acc = y_acc;
    _payload->z_acc = z_acc;
    _payload->x_vel = x_vel;
    _payload->y_vel = y_vel;
    _payload->z_vel = z_vel;
    _payload->x_pos = x_pos;
    _payload->y_pos = y_pos;
    _payload->z_pos = z_pos;
    _payload->airspeed = airspeed;
    _payload->roll_rate = roll_rate;
    _payload->pitch_rate = pitch_rate;
    _payload->yaw_rate = yaw_rate;
    memcpy(&(_payload->vel_variance), vel_variance, sizeof(float)*3);
    memcpy(&(_payload->pos_variance), pos_variance, sizeof(float)*3);
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_control_system_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_control_system_state_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->x_acc, _payload->y_acc, _payload->z_acc, _payload->x_vel, _payload->y_vel, _payload->z_vel, _payload->x_pos, _payload->y_pos, _payload->z_pos, _payload->airspeed, _payload->vel_variance, _payload->pos_variance, _payload->q, _payload->roll_rate, _payload->pitch_rate, _payload->yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float x_acc, float y_acc, float z_acc, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float airspeed, const float* vel_variance, const float* pos_variance, const float* q, float roll_rate, float pitch_rate, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_control_system_state_t* _payload = (fmav_control_system_state_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->x_acc = x_acc;
    _payload->y_acc = y_acc;
    _payload->z_acc = z_acc;
    _payload->x_vel = x_vel;
    _payload->y_vel = y_vel;
    _payload->z_vel = z_vel;
    _payload->x_pos = x_pos;
    _payload->y_pos = y_pos;
    _payload->z_pos = z_pos;
    _payload->airspeed = airspeed;
    _payload->roll_rate = roll_rate;
    _payload->pitch_rate = pitch_rate;
    _payload->yaw_rate = yaw_rate;
    memcpy(&(_payload->vel_variance), vel_variance, sizeof(float)*3);
    memcpy(&(_payload->pos_variance), pos_variance, sizeof(float)*3);
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_control_system_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_control_system_state_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->x_acc, _payload->y_acc, _payload->z_acc, _payload->x_vel, _payload->y_vel, _payload->z_vel, _payload->x_pos, _payload->y_pos, _payload->z_pos, _payload->airspeed, _payload->vel_variance, _payload->pos_variance, _payload->q, _payload->roll_rate, _payload->pitch_rate, _payload->yaw_rate,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float x_acc, float y_acc, float z_acc, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float airspeed, const float* vel_variance, const float* pos_variance, const float* q, float roll_rate, float pitch_rate, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_control_system_state_t _payload;

    _payload.time_usec = time_usec;
    _payload.x_acc = x_acc;
    _payload.y_acc = y_acc;
    _payload.z_acc = z_acc;
    _payload.x_vel = x_vel;
    _payload.y_vel = y_vel;
    _payload.z_vel = z_vel;
    _payload.x_pos = x_pos;
    _payload.y_pos = y_pos;
    _payload.z_pos = z_pos;
    _payload.airspeed = airspeed;
    _payload.roll_rate = roll_rate;
    _payload.pitch_rate = pitch_rate;
    _payload.yaw_rate = yaw_rate;
    memcpy(&(_payload.vel_variance), vel_variance, sizeof(float)*3);
    memcpy(&(_payload.pos_variance), pos_variance, sizeof(float)*3);
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_control_system_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CONTROL_SYSTEM_STATE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_control_system_state_decode(fmav_control_system_state_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_control_system_state_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_x_acc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_y_acc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_z_acc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_x_vel(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_y_vel(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_z_vel(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_x_pos(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_y_pos(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_z_pos(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_airspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_roll_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[88]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_pitch_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[92]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_yaw_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[96]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_control_system_state_get_field_vel_variance_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[48]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_vel_variance(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_VEL_VARIANCE_NUM) return 0;
    return ((float*)&(msg->payload[48]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_control_system_state_get_field_pos_variance_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[60]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_pos_variance(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_POS_VARIANCE_NUM) return 0;
    return ((float*)&(msg->payload[60]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_control_system_state_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[72]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_system_state_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[72]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE  146

#define mavlink_control_system_state_t  fmav_control_system_state_t

#define MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE_LEN  100
#define MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE_MIN_LEN  100
#define MAVLINK_MSG_ID_146_LEN  100
#define MAVLINK_MSG_ID_146_MIN_LEN  100

#define MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE_CRC  103
#define MAVLINK_MSG_ID_146_CRC  103

#define MAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_VEL_VARIANCE_LEN 3
#define MAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_POS_VARIANCE_LEN 3
#define MAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_control_system_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, float x_acc, float y_acc, float z_acc, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float airspeed, const float* vel_variance, const float* pos_variance, const float* q, float roll_rate, float pitch_rate, float yaw_rate)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_control_system_state_pack(
        _msg, sysid, compid,
        time_usec, x_acc, y_acc, z_acc, x_vel, y_vel, z_vel, x_pos, y_pos, z_pos, airspeed, vel_variance, pos_variance, q, roll_rate, pitch_rate, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_control_system_state_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_control_system_state_t* _payload)
{
    return mavlink_msg_control_system_state_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->x_acc, _payload->y_acc, _payload->z_acc, _payload->x_vel, _payload->y_vel, _payload->z_vel, _payload->x_pos, _payload->y_pos, _payload->z_pos, _payload->airspeed, _payload->vel_variance, _payload->pos_variance, _payload->q, _payload->roll_rate, _payload->pitch_rate, _payload->yaw_rate);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_control_system_state_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float x_acc, float y_acc, float z_acc, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float airspeed, const float* vel_variance, const float* pos_variance, const float* q, float roll_rate, float pitch_rate, float yaw_rate)
{
    return fmav_msg_control_system_state_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, x_acc, y_acc, z_acc, x_vel, y_vel, z_vel, x_pos, y_pos, z_pos, airspeed, vel_variance, pos_variance, q, roll_rate, pitch_rate, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_control_system_state_decode(const mavlink_message_t* msg, mavlink_control_system_state_t* payload)
{
    fmav_msg_control_system_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_H
