//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_H
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_H


//----------------------------------------
//-- Message GIMBAL_MANAGER_INFORMATION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_manager_information_t {
    uint32_t time_boot_ms;
    uint32_t cap_flags;
    float roll_min;
    float roll_max;
    float pitch_min;
    float pitch_max;
    float yaw_min;
    float yaw_max;
    uint8_t gimbal_device_id;
}) fmav_gimbal_manager_information_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION  280

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA  70

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FRAME_LEN_MAX  58



#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FIELD_CAP_FLAGS_OFS  4
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FIELD_ROLL_MIN_OFS  8
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FIELD_ROLL_MAX_OFS  12
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FIELD_PITCH_MIN_OFS  16
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FIELD_PITCH_MAX_OFS  20
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FIELD_YAW_MIN_OFS  24
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FIELD_YAW_MAX_OFS  28
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FIELD_GIMBAL_DEVICE_ID_OFS  32


//----------------------------------------
//-- Message GIMBAL_MANAGER_INFORMATION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_information_t* _payload = (fmav_gimbal_manager_information_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->cap_flags = cap_flags;
    _payload->roll_min = roll_min;
    _payload->roll_max = roll_max;
    _payload->pitch_min = pitch_min;
    _payload->pitch_max = pitch_max;
    _payload->yaw_min = yaw_min;
    _payload->yaw_max = yaw_max;
    _payload->gimbal_device_id = gimbal_device_id;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_information_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->cap_flags, _payload->gimbal_device_id, _payload->roll_min, _payload->roll_max, _payload->pitch_min, _payload->pitch_max, _payload->yaw_min, _payload->yaw_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_information_t* _payload = (fmav_gimbal_manager_information_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->cap_flags = cap_flags;
    _payload->roll_min = roll_min;
    _payload->roll_max = roll_max;
    _payload->pitch_min = pitch_min;
    _payload->pitch_max = pitch_max;
    _payload->yaw_min = yaw_min;
    _payload->yaw_max = yaw_max;
    _payload->gimbal_device_id = gimbal_device_id;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_information_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->cap_flags, _payload->gimbal_device_id, _payload->roll_min, _payload->roll_max, _payload->pitch_min, _payload->pitch_max, _payload->yaw_min, _payload->yaw_max,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_information_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.cap_flags = cap_flags;
    _payload.roll_min = roll_min;
    _payload.roll_max = roll_max;
    _payload.pitch_min = pitch_min;
    _payload.pitch_max = pitch_max;
    _payload.yaw_min = yaw_min;
    _payload.yaw_max = yaw_max;
    _payload.gimbal_device_id = gimbal_device_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_MANAGER_INFORMATION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_manager_information_decode(fmav_gimbal_manager_information_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_manager_information_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_manager_information_get_field_cap_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_information_get_field_roll_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_information_get_field_roll_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_information_get_field_pitch_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_information_get_field_pitch_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_information_get_field_yaw_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_manager_information_get_field_yaw_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_information_get_field_gimbal_device_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION  280

#define mavlink_gimbal_manager_information_t  fmav_gimbal_manager_information_t

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_LEN  33
#define MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_MIN_LEN  33
#define MAVLINK_MSG_ID_280_LEN  33
#define MAVLINK_MSG_ID_280_MIN_LEN  33

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_CRC  70
#define MAVLINK_MSG_ID_280_CRC  70




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_manager_information_pack(
        _msg, sysid, compid,
        time_boot_ms, cap_flags, gimbal_device_id, roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_information_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gimbal_manager_information_t* _payload)
{
    return mavlink_msg_gimbal_manager_information_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->cap_flags, _payload->gimbal_device_id, _payload->roll_min, _payload->roll_max, _payload->pitch_min, _payload->pitch_max, _payload->yaw_min, _payload->yaw_max);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_information_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
    return fmav_msg_gimbal_manager_information_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, cap_flags, gimbal_device_id, roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_manager_information_decode(const mavlink_message_t* msg, mavlink_gimbal_manager_information_t* payload)
{
    fmav_msg_gimbal_manager_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_H
