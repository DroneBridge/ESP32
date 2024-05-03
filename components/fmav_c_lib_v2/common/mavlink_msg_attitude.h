//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ATTITUDE_H
#define FASTMAVLINK_MSG_ATTITUDE_H


//----------------------------------------
//-- Message ATTITUDE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_attitude_t {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
}) fmav_attitude_t;


#define FASTMAVLINK_MSG_ID_ATTITUDE  30

#define FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_ATTITUDE_CRCEXTRA  39

#define FASTMAVLINK_MSG_ATTITUDE_FLAGS  0
#define FASTMAVLINK_MSG_ATTITUDE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ATTITUDE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ATTITUDE_FRAME_LEN_MAX  53



#define FASTMAVLINK_MSG_ATTITUDE_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_ATTITUDE_FIELD_ROLL_OFS  4
#define FASTMAVLINK_MSG_ATTITUDE_FIELD_PITCH_OFS  8
#define FASTMAVLINK_MSG_ATTITUDE_FIELD_YAW_OFS  12
#define FASTMAVLINK_MSG_ATTITUDE_FIELD_ROLLSPEED_OFS  16
#define FASTMAVLINK_MSG_ATTITUDE_FIELD_PITCHSPEED_OFS  20
#define FASTMAVLINK_MSG_ATTITUDE_FIELD_YAWSPEED_OFS  24


//----------------------------------------
//-- Message ATTITUDE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed,
    fmav_status_t* _status)
{
    fmav_attitude_t* _payload = (fmav_attitude_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ATTITUDE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ATTITUDE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed,
    fmav_status_t* _status)
{
    fmav_attitude_t* _payload = (fmav_attitude_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ATTITUDE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed,
    fmav_status_t* _status)
{
    fmav_attitude_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.rollspeed = rollspeed;
    _payload.pitchspeed = pitchspeed;
    _payload.yawspeed = yawspeed;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ATTITUDE,
        FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ATTITUDE,
        FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ATTITUDE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_attitude_decode(fmav_attitude_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_attitude_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_get_field_rollspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_get_field_pitchspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_get_field_yawspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ATTITUDE  30

#define mavlink_attitude_t  fmav_attitude_t

#define MAVLINK_MSG_ID_ATTITUDE_LEN  28
#define MAVLINK_MSG_ID_ATTITUDE_MIN_LEN  28
#define MAVLINK_MSG_ID_30_LEN  28
#define MAVLINK_MSG_ID_30_MIN_LEN  28

#define MAVLINK_MSG_ID_ATTITUDE_CRC  39
#define MAVLINK_MSG_ID_30_CRC  39




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_attitude_pack(
        _msg, sysid, compid,
        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_attitude_t* _payload)
{
    return mavlink_msg_attitude_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
    return fmav_msg_attitude_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* payload)
{
    fmav_msg_attitude_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ATTITUDE_H
