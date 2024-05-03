//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_HOME_POSITION_H
#define FASTMAVLINK_MSG_SET_HOME_POSITION_H


//----------------------------------------
//-- Message SET_HOME_POSITION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_home_position_t {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    float x;
    float y;
    float z;
    float q[4];
    float approach_x;
    float approach_y;
    float approach_z;
    uint8_t target_system;
    uint64_t time_usec;
}) fmav_set_home_position_t;


#define FASTMAVLINK_MSG_ID_SET_HOME_POSITION  243

#define FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX  61
#define FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA  85

#define FASTMAVLINK_MSG_SET_HOME_POSITION_FLAGS  1
#define FASTMAVLINK_MSG_SET_HOME_POSITION_TARGET_SYSTEM_OFS  52
#define FASTMAVLINK_MSG_SET_HOME_POSITION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SET_HOME_POSITION_FRAME_LEN_MAX  86

#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_LATITUDE_OFS  0
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_LONGITUDE_OFS  4
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_ALTITUDE_OFS  8
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_X_OFS  12
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_Y_OFS  16
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_Z_OFS  20
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_Q_OFS  24
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_APPROACH_X_OFS  40
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_APPROACH_Y_OFS  44
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_APPROACH_Z_OFS  48
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_TARGET_SYSTEM_OFS  52
#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_TIME_USEC_OFS  53


//----------------------------------------
//-- Message SET_HOME_POSITION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec,
    fmav_status_t* _status)
{
    fmav_set_home_position_t* _payload = (fmav_set_home_position_t*)_msg->payload;

    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->altitude = altitude;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->approach_x = approach_x;
    _payload->approach_y = approach_y;
    _payload->approach_z = approach_z;
    _payload->target_system = target_system;
    _payload->time_usec = time_usec;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SET_HOME_POSITION;
    _msg->target_sysid = target_system;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_home_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_home_position_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->latitude, _payload->longitude, _payload->altitude, _payload->x, _payload->y, _payload->z, _payload->q, _payload->approach_x, _payload->approach_y, _payload->approach_z, _payload->time_usec,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec,
    fmav_status_t* _status)
{
    fmav_set_home_position_t* _payload = (fmav_set_home_position_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->altitude = altitude;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->approach_x = approach_x;
    _payload->approach_y = approach_y;
    _payload->approach_z = approach_z;
    _payload->target_system = target_system;
    _payload->time_usec = time_usec;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_HOME_POSITION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_HOME_POSITION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_HOME_POSITION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_home_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_home_position_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->latitude, _payload->longitude, _payload->altitude, _payload->x, _payload->y, _payload->z, _payload->q, _payload->approach_x, _payload->approach_y, _payload->approach_z, _payload->time_usec,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec,
    fmav_status_t* _status)
{
    fmav_set_home_position_t _payload;

    _payload.latitude = latitude;
    _payload.longitude = longitude;
    _payload.altitude = altitude;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.approach_x = approach_x;
    _payload.approach_y = approach_y;
    _payload.approach_z = approach_z;
    _payload.target_system = target_system;
    _payload.time_usec = time_usec;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SET_HOME_POSITION,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_home_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SET_HOME_POSITION,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SET_HOME_POSITION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_home_position_decode(fmav_set_home_position_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_set_home_position_get_field_latitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_set_home_position_get_field_longitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_set_home_position_get_field_altitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_home_position_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_home_position_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_home_position_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_home_position_get_field_approach_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_home_position_get_field_approach_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_home_position_get_field_approach_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_home_position_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[52]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_set_home_position_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[53]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_set_home_position_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[24]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_home_position_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[24]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_HOME_POSITION  243

#define mavlink_set_home_position_t  fmav_set_home_position_t

#define MAVLINK_MSG_ID_SET_HOME_POSITION_LEN  61
#define MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN  53
#define MAVLINK_MSG_ID_243_LEN  61
#define MAVLINK_MSG_ID_243_MIN_LEN  53

#define MAVLINK_MSG_ID_SET_HOME_POSITION_CRC  85
#define MAVLINK_MSG_ID_243_CRC  85

#define MAVLINK_MSG_SET_HOME_POSITION_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_home_position_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_home_position_pack(
        _msg, sysid, compid,
        target_system, latitude, longitude, altitude, x, y, z, q, approach_x, approach_y, approach_z, time_usec,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_home_position_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_set_home_position_t* _payload)
{
    return mavlink_msg_set_home_position_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->latitude, _payload->longitude, _payload->altitude, _payload->x, _payload->y, _payload->z, _payload->q, _payload->approach_x, _payload->approach_y, _payload->approach_z, _payload->time_usec);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_home_position_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec)
{
    return fmav_msg_set_home_position_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, latitude, longitude, altitude, x, y, z, q, approach_x, approach_y, approach_z, time_usec,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_home_position_decode(const mavlink_message_t* msg, mavlink_set_home_position_t* payload)
{
    fmav_msg_set_home_position_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_HOME_POSITION_H
