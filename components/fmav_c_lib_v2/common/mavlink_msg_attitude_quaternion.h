//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ATTITUDE_QUATERNION_H
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_H


//----------------------------------------
//-- Message ATTITUDE_QUATERNION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_attitude_quaternion_t {
    uint32_t time_boot_ms;
    float q1;
    float q2;
    float q3;
    float q4;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    float repr_offset_q[4];
}) fmav_attitude_quaternion_t;


#define FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION  31

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX  48
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_CRCEXTRA  246

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FLAGS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FRAME_LEN_MAX  73

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_REPR_OFFSET_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_REPR_OFFSET_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_Q1_OFS  4
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_Q2_OFS  8
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_Q3_OFS  12
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_Q4_OFS  16
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_ROLLSPEED_OFS  20
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_PITCHSPEED_OFS  24
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_YAWSPEED_OFS  28
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_REPR_OFFSET_Q_OFS  32


//----------------------------------------
//-- Message ATTITUDE_QUATERNION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float* repr_offset_q,
    fmav_status_t* _status)
{
    fmav_attitude_quaternion_t* _payload = (fmav_attitude_quaternion_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->q1 = q1;
    _payload->q2 = q2;
    _payload->q3 = q3;
    _payload->q4 = q4;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    memcpy(&(_payload->repr_offset_q), repr_offset_q, sizeof(float)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ATTITUDE_QUATERNION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_quaternion_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_quaternion_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->repr_offset_q,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float* repr_offset_q,
    fmav_status_t* _status)
{
    fmav_attitude_quaternion_t* _payload = (fmav_attitude_quaternion_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->q1 = q1;
    _payload->q2 = q2;
    _payload->q3 = q3;
    _payload->q4 = q4;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    memcpy(&(_payload->repr_offset_q), repr_offset_q, sizeof(float)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_quaternion_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_quaternion_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->repr_offset_q,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float* repr_offset_q,
    fmav_status_t* _status)
{
    fmav_attitude_quaternion_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.q1 = q1;
    _payload.q2 = q2;
    _payload.q3 = q3;
    _payload.q4 = q4;
    _payload.rollspeed = rollspeed;
    _payload.pitchspeed = pitchspeed;
    _payload.yawspeed = yawspeed;
    memcpy(&(_payload.repr_offset_q), repr_offset_q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_quaternion_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ATTITUDE_QUATERNION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_attitude_quaternion_decode(fmav_attitude_quaternion_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_attitude_quaternion_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_get_field_q1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_get_field_q2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_get_field_q3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_get_field_q4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_get_field_rollspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_get_field_pitchspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_get_field_yawspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_attitude_quaternion_get_field_repr_offset_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[32]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_get_field_repr_offset_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_REPR_OFFSET_Q_NUM) return 0;
    return ((float*)&(msg->payload[32]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION  31

#define mavlink_attitude_quaternion_t  fmav_attitude_quaternion_t

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN  48
#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN  32
#define MAVLINK_MSG_ID_31_LEN  48
#define MAVLINK_MSG_ID_31_MIN_LEN  32

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC  246
#define MAVLINK_MSG_ID_31_CRC  246

#define MAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_REPR_OFFSET_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_quaternion_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float* repr_offset_q)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_attitude_quaternion_pack(
        _msg, sysid, compid,
        time_boot_ms, q1, q2, q3, q4, rollspeed, pitchspeed, yawspeed, repr_offset_q,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_quaternion_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_attitude_quaternion_t* _payload)
{
    return mavlink_msg_attitude_quaternion_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->repr_offset_q);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_quaternion_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float* repr_offset_q)
{
    return fmav_msg_attitude_quaternion_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, q1, q2, q3, q4, rollspeed, pitchspeed, yawspeed, repr_offset_q,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_attitude_quaternion_decode(const mavlink_message_t* msg, mavlink_attitude_quaternion_t* payload)
{
    fmav_msg_attitude_quaternion_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ATTITUDE_QUATERNION_H
