//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_H
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_H


//----------------------------------------
//-- Message TRAJECTORY_REPRESENTATION_BEZIER
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_trajectory_representation_bezier_t {
    uint64_t time_usec;
    float pos_x[5];
    float pos_y[5];
    float pos_z[5];
    float delta[5];
    float pos_yaw[5];
    uint8_t valid_points;
}) fmav_trajectory_representation_bezier_t;


#define FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER  333

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX  109
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA  231

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FLAGS  0
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FRAME_LEN_MAX  134

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_LEN  20 // length of array = number of bytes

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_OFS  8
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_OFS  28
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_OFS  48
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_OFS  68
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_OFS  88
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_VALID_POINTS_OFS  108


//----------------------------------------
//-- Message TRAJECTORY_REPRESENTATION_BEZIER pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw,
    fmav_status_t* _status)
{
    fmav_trajectory_representation_bezier_t* _payload = (fmav_trajectory_representation_bezier_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->valid_points = valid_points;
    memcpy(&(_payload->pos_x), pos_x, sizeof(float)*5);
    memcpy(&(_payload->pos_y), pos_y, sizeof(float)*5);
    memcpy(&(_payload->pos_z), pos_z, sizeof(float)*5);
    memcpy(&(_payload->delta), delta, sizeof(float)*5);
    memcpy(&(_payload->pos_yaw), pos_yaw, sizeof(float)*5);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_trajectory_representation_bezier_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_trajectory_representation_bezier_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->valid_points, _payload->pos_x, _payload->pos_y, _payload->pos_z, _payload->delta, _payload->pos_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw,
    fmav_status_t* _status)
{
    fmav_trajectory_representation_bezier_t* _payload = (fmav_trajectory_representation_bezier_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->valid_points = valid_points;
    memcpy(&(_payload->pos_x), pos_x, sizeof(float)*5);
    memcpy(&(_payload->pos_y), pos_y, sizeof(float)*5);
    memcpy(&(_payload->pos_z), pos_z, sizeof(float)*5);
    memcpy(&(_payload->delta), delta, sizeof(float)*5);
    memcpy(&(_payload->pos_yaw), pos_yaw, sizeof(float)*5);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_trajectory_representation_bezier_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_trajectory_representation_bezier_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->valid_points, _payload->pos_x, _payload->pos_y, _payload->pos_z, _payload->delta, _payload->pos_yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw,
    fmav_status_t* _status)
{
    fmav_trajectory_representation_bezier_t _payload;

    _payload.time_usec = time_usec;
    _payload.valid_points = valid_points;
    memcpy(&(_payload.pos_x), pos_x, sizeof(float)*5);
    memcpy(&(_payload.pos_y), pos_y, sizeof(float)*5);
    memcpy(&(_payload.pos_z), pos_z, sizeof(float)*5);
    memcpy(&(_payload.delta), delta, sizeof(float)*5);
    memcpy(&(_payload.pos_yaw), pos_yaw, sizeof(float)*5);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_trajectory_representation_bezier_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TRAJECTORY_REPRESENTATION_BEZIER decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_trajectory_representation_bezier_decode(fmav_trajectory_representation_bezier_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_trajectory_representation_bezier_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_trajectory_representation_bezier_get_field_valid_points(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[108]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_pos_x_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_pos_x(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_NUM) return 0;
    return ((float*)&(msg->payload[8]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_pos_y_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[28]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_pos_y(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_NUM) return 0;
    return ((float*)&(msg->payload[28]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_pos_z_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[48]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_pos_z(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_NUM) return 0;
    return ((float*)&(msg->payload[48]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_delta_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[68]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_delta(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_NUM) return 0;
    return ((float*)&(msg->payload[68]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_pos_yaw_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[88]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_pos_yaw(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_NUM) return 0;
    return ((float*)&(msg->payload[88]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER  333

#define mavlink_trajectory_representation_bezier_t  fmav_trajectory_representation_bezier_t

#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER_LEN  109
#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER_MIN_LEN  109
#define MAVLINK_MSG_ID_333_LEN  109
#define MAVLINK_MSG_ID_333_MIN_LEN  109

#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER_CRC  231
#define MAVLINK_MSG_ID_333_CRC  231

#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_LEN 5


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_trajectory_representation_bezier_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_trajectory_representation_bezier_pack(
        _msg, sysid, compid,
        time_usec, valid_points, pos_x, pos_y, pos_z, delta, pos_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_trajectory_representation_bezier_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_trajectory_representation_bezier_t* _payload)
{
    return mavlink_msg_trajectory_representation_bezier_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->valid_points, _payload->pos_x, _payload->pos_y, _payload->pos_z, _payload->delta, _payload->pos_yaw);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_trajectory_representation_bezier_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw)
{
    return fmav_msg_trajectory_representation_bezier_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, valid_points, pos_x, pos_y, pos_z, delta, pos_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_trajectory_representation_bezier_decode(const mavlink_message_t* msg, mavlink_trajectory_representation_bezier_t* payload)
{
    fmav_msg_trajectory_representation_bezier_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_H
