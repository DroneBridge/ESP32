//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_H
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_H


//----------------------------------------
//-- Message SET_POSITION_TARGET_LOCAL_NED
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_position_target_local_ned_t {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float afx;
    float afy;
    float afz;
    float yaw;
    float yaw_rate;
    uint16_t type_mask;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t coordinate_frame;
}) fmav_set_position_target_local_ned_t;


#define FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED  84

#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX  53
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA  143

#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FLAGS  3
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TARGET_SYSTEM_OFS  50
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TARGET_COMPONENT_OFS  51

#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FRAME_LEN_MAX  78



#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_X_OFS  4
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_Y_OFS  8
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_Z_OFS  12
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_VX_OFS  16
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_VY_OFS  20
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_VZ_OFS  24
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_AFX_OFS  28
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_AFY_OFS  32
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_AFZ_OFS  36
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_YAW_OFS  40
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_YAW_RATE_OFS  44
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_TYPE_MASK_OFS  48
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_TARGET_SYSTEM_OFS  50
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_TARGET_COMPONENT_OFS  51
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FIELD_COORDINATE_FRAME_OFS  52


//----------------------------------------
//-- Message SET_POSITION_TARGET_LOCAL_NED pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_set_position_target_local_ned_t* _payload = (fmav_set_position_target_local_ned_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->afx = afx;
    _payload->afy = afy;
    _payload->afz = afz;
    _payload->yaw = yaw;
    _payload->yaw_rate = yaw_rate;
    _payload->type_mask = type_mask;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->coordinate_frame = coordinate_frame;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_position_target_local_ned_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_position_target_local_ned_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->coordinate_frame, _payload->type_mask, _payload->x, _payload->y, _payload->z, _payload->vx, _payload->vy, _payload->vz, _payload->afx, _payload->afy, _payload->afz, _payload->yaw, _payload->yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_set_position_target_local_ned_t* _payload = (fmav_set_position_target_local_ned_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->afx = afx;
    _payload->afy = afy;
    _payload->afz = afz;
    _payload->yaw = yaw;
    _payload->yaw_rate = yaw_rate;
    _payload->type_mask = type_mask;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->coordinate_frame = coordinate_frame;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_position_target_local_ned_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_position_target_local_ned_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->coordinate_frame, _payload->type_mask, _payload->x, _payload->y, _payload->z, _payload->vx, _payload->vy, _payload->vz, _payload->afx, _payload->afy, _payload->afz, _payload->yaw, _payload->yaw_rate,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_set_position_target_local_ned_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.afx = afx;
    _payload.afy = afy;
    _payload.afz = afz;
    _payload.yaw = yaw;
    _payload.yaw_rate = yaw_rate;
    _payload.type_mask = type_mask;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.coordinate_frame = coordinate_frame;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_position_target_local_ned_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SET_POSITION_TARGET_LOCAL_NED decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_position_target_local_ned_decode(fmav_set_position_target_local_ned_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_set_position_target_local_ned_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_vx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_vy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_vz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_afx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_afy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_afz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_position_target_local_ned_get_field_yaw_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_get_field_type_mask(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_position_target_local_ned_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_position_target_local_ned_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[51]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_position_target_local_ned_get_field_coordinate_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[52]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED  84

#define mavlink_set_position_target_local_ned_t  fmav_set_position_target_local_ned_t

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN  53
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN  53
#define MAVLINK_MSG_ID_84_LEN  53
#define MAVLINK_MSG_ID_84_MIN_LEN  53

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC  143
#define MAVLINK_MSG_ID_84_CRC  143




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_position_target_local_ned_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_position_target_local_ned_pack(
        _msg, sysid, compid,
        time_boot_ms, target_system, target_component, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_position_target_local_ned_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_set_position_target_local_ned_t* _payload)
{
    return mavlink_msg_set_position_target_local_ned_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->coordinate_frame, _payload->type_mask, _payload->x, _payload->y, _payload->z, _payload->vx, _payload->vy, _payload->vz, _payload->afx, _payload->afy, _payload->afz, _payload->yaw, _payload->yaw_rate);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_position_target_local_ned_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
    return fmav_msg_set_position_target_local_ned_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, target_system, target_component, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_position_target_local_ned_decode(const mavlink_message_t* msg, mavlink_set_position_target_local_ned_t* payload)
{
    fmav_msg_set_position_target_local_ned_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_H
