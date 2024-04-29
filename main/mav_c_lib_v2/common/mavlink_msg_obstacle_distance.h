//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OBSTACLE_DISTANCE_H
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_H


//----------------------------------------
//-- Message OBSTACLE_DISTANCE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_obstacle_distance_t {
    uint64_t time_usec;
    uint16_t distances[72];
    uint16_t min_distance;
    uint16_t max_distance;
    uint8_t sensor_type;
    uint8_t increment;
    float increment_f;
    float angle_offset;
    uint8_t frame;
}) fmav_obstacle_distance_t;


#define FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE  330

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX  167
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA  23

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FLAGS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FRAME_LEN_MAX  192

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_NUM  72 // number of elements in array
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN  144 // length of array = number of bytes

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_OFS  8
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_MIN_DISTANCE_OFS  152
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_MAX_DISTANCE_OFS  154
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_SENSOR_TYPE_OFS  156
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_INCREMENT_OFS  157
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_INCREMENT_F_OFS  158
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_ANGLE_OFFSET_OFS  162
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_FRAME_OFS  166


//----------------------------------------
//-- Message OBSTACLE_DISTANCE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame,
    fmav_status_t* _status)
{
    fmav_obstacle_distance_t* _payload = (fmav_obstacle_distance_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->min_distance = min_distance;
    _payload->max_distance = max_distance;
    _payload->sensor_type = sensor_type;
    _payload->increment = increment;
    _payload->increment_f = increment_f;
    _payload->angle_offset = angle_offset;
    _payload->frame = frame;
    memcpy(&(_payload->distances), distances, sizeof(uint16_t)*72);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_obstacle_distance_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_obstacle_distance_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->sensor_type, _payload->distances, _payload->increment, _payload->min_distance, _payload->max_distance, _payload->increment_f, _payload->angle_offset, _payload->frame,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame,
    fmav_status_t* _status)
{
    fmav_obstacle_distance_t* _payload = (fmav_obstacle_distance_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->min_distance = min_distance;
    _payload->max_distance = max_distance;
    _payload->sensor_type = sensor_type;
    _payload->increment = increment;
    _payload->increment_f = increment_f;
    _payload->angle_offset = angle_offset;
    _payload->frame = frame;
    memcpy(&(_payload->distances), distances, sizeof(uint16_t)*72);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_obstacle_distance_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_obstacle_distance_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->sensor_type, _payload->distances, _payload->increment, _payload->min_distance, _payload->max_distance, _payload->increment_f, _payload->angle_offset, _payload->frame,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame,
    fmav_status_t* _status)
{
    fmav_obstacle_distance_t _payload;

    _payload.time_usec = time_usec;
    _payload.min_distance = min_distance;
    _payload.max_distance = max_distance;
    _payload.sensor_type = sensor_type;
    _payload.increment = increment;
    _payload.increment_f = increment_f;
    _payload.angle_offset = angle_offset;
    _payload.frame = frame;
    memcpy(&(_payload.distances), distances, sizeof(uint16_t)*72);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_obstacle_distance_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OBSTACLE_DISTANCE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_obstacle_distance_decode(fmav_obstacle_distance_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_obstacle_distance_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_get_field_min_distance(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[152]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_get_field_max_distance(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[154]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_obstacle_distance_get_field_sensor_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[156]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_obstacle_distance_get_field_increment(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[157]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_obstacle_distance_get_field_increment_f(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[158]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_obstacle_distance_get_field_angle_offset(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[162]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_obstacle_distance_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[166]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_obstacle_distance_get_field_distances_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_get_field_distances(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_NUM) return 0;
    return ((uint16_t*)&(msg->payload[8]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE  330

#define mavlink_obstacle_distance_t  fmav_obstacle_distance_t

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_LEN  167
#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_MIN_LEN  158
#define MAVLINK_MSG_ID_330_LEN  167
#define MAVLINK_MSG_ID_330_MIN_LEN  158

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_CRC  23
#define MAVLINK_MSG_ID_330_CRC  23

#define MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN 72


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_obstacle_distance_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_obstacle_distance_pack(
        _msg, sysid, compid,
        time_usec, sensor_type, distances, increment, min_distance, max_distance, increment_f, angle_offset, frame,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_obstacle_distance_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_obstacle_distance_t* _payload)
{
    return mavlink_msg_obstacle_distance_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->sensor_type, _payload->distances, _payload->increment, _payload->min_distance, _payload->max_distance, _payload->increment_f, _payload->angle_offset, _payload->frame);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_obstacle_distance_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame)
{
    return fmav_msg_obstacle_distance_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, sensor_type, distances, increment, min_distance, max_distance, increment_f, angle_offset, frame,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_obstacle_distance_decode(const mavlink_message_t* msg, mavlink_obstacle_distance_t* payload)
{
    fmav_msg_obstacle_distance_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OBSTACLE_DISTANCE_H
