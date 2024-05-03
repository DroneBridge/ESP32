//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LANDING_TARGET_H
#define FASTMAVLINK_MSG_LANDING_TARGET_H


//----------------------------------------
//-- Message LANDING_TARGET
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_landing_target_t {
    uint64_t time_usec;
    float angle_x;
    float angle_y;
    float distance;
    float size_x;
    float size_y;
    uint8_t target_num;
    uint8_t frame;
    float x;
    float y;
    float z;
    float q[4];
    uint8_t type;
    uint8_t position_valid;
}) fmav_landing_target_t;


#define FASTMAVLINK_MSG_ID_LANDING_TARGET  149

#define FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX  60
#define FASTMAVLINK_MSG_LANDING_TARGET_CRCEXTRA  200

#define FASTMAVLINK_MSG_LANDING_TARGET_FLAGS  0
#define FASTMAVLINK_MSG_LANDING_TARGET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LANDING_TARGET_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_LANDING_TARGET_FRAME_LEN_MAX  85

#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_ANGLE_X_OFS  8
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_ANGLE_Y_OFS  12
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_DISTANCE_OFS  16
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_SIZE_X_OFS  20
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_SIZE_Y_OFS  24
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_TARGET_NUM_OFS  28
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_FRAME_OFS  29
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_X_OFS  30
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_Y_OFS  34
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_Z_OFS  38
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_Q_OFS  42
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_TYPE_OFS  58
#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_POSITION_VALID_OFS  59


//----------------------------------------
//-- Message LANDING_TARGET pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float* q, uint8_t type, uint8_t position_valid,
    fmav_status_t* _status)
{
    fmav_landing_target_t* _payload = (fmav_landing_target_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->angle_x = angle_x;
    _payload->angle_y = angle_y;
    _payload->distance = distance;
    _payload->size_x = size_x;
    _payload->size_y = size_y;
    _payload->target_num = target_num;
    _payload->frame = frame;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->type = type;
    _payload->position_valid = position_valid;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_LANDING_TARGET;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_LANDING_TARGET_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_landing_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_landing_target_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->target_num, _payload->frame, _payload->angle_x, _payload->angle_y, _payload->distance, _payload->size_x, _payload->size_y, _payload->x, _payload->y, _payload->z, _payload->q, _payload->type, _payload->position_valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float* q, uint8_t type, uint8_t position_valid,
    fmav_status_t* _status)
{
    fmav_landing_target_t* _payload = (fmav_landing_target_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->angle_x = angle_x;
    _payload->angle_y = angle_y;
    _payload->distance = distance;
    _payload->size_x = size_x;
    _payload->size_y = size_y;
    _payload->target_num = target_num;
    _payload->frame = frame;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->type = type;
    _payload->position_valid = position_valid;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LANDING_TARGET;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LANDING_TARGET >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LANDING_TARGET >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LANDING_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_landing_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_landing_target_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->target_num, _payload->frame, _payload->angle_x, _payload->angle_y, _payload->distance, _payload->size_x, _payload->size_y, _payload->x, _payload->y, _payload->z, _payload->q, _payload->type, _payload->position_valid,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float* q, uint8_t type, uint8_t position_valid,
    fmav_status_t* _status)
{
    fmav_landing_target_t _payload;

    _payload.time_usec = time_usec;
    _payload.angle_x = angle_x;
    _payload.angle_y = angle_y;
    _payload.distance = distance;
    _payload.size_x = size_x;
    _payload.size_y = size_y;
    _payload.target_num = target_num;
    _payload.frame = frame;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.type = type;
    _payload.position_valid = position_valid;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LANDING_TARGET,
        FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LANDING_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_landing_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LANDING_TARGET,
        FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LANDING_TARGET_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LANDING_TARGET decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_landing_target_decode(fmav_landing_target_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_landing_target_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_landing_target_get_field_angle_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_landing_target_get_field_angle_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_landing_target_get_field_distance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_landing_target_get_field_size_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_landing_target_get_field_size_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_landing_target_get_field_target_num(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_landing_target_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_landing_target_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[30]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_landing_target_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[34]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_landing_target_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[38]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_landing_target_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[58]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_landing_target_get_field_position_valid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[59]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_landing_target_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[42]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_landing_target_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_LANDING_TARGET_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[42]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LANDING_TARGET  149

#define mavlink_landing_target_t  fmav_landing_target_t

#define MAVLINK_MSG_ID_LANDING_TARGET_LEN  60
#define MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN  30
#define MAVLINK_MSG_ID_149_LEN  60
#define MAVLINK_MSG_ID_149_MIN_LEN  30

#define MAVLINK_MSG_ID_LANDING_TARGET_CRC  200
#define MAVLINK_MSG_ID_149_CRC  200

#define MAVLINK_MSG_LANDING_TARGET_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_landing_target_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float* q, uint8_t type, uint8_t position_valid)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_landing_target_pack(
        _msg, sysid, compid,
        time_usec, target_num, frame, angle_x, angle_y, distance, size_x, size_y, x, y, z, q, type, position_valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_landing_target_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_landing_target_t* _payload)
{
    return mavlink_msg_landing_target_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->target_num, _payload->frame, _payload->angle_x, _payload->angle_y, _payload->distance, _payload->size_x, _payload->size_y, _payload->x, _payload->y, _payload->z, _payload->q, _payload->type, _payload->position_valid);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_landing_target_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float* q, uint8_t type, uint8_t position_valid)
{
    return fmav_msg_landing_target_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, target_num, frame, angle_x, angle_y, distance, size_x, size_y, x, y, z, q, type, position_valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_landing_target_decode(const mavlink_message_t* msg, mavlink_landing_target_t* payload)
{
    fmav_msg_landing_target_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LANDING_TARGET_H
