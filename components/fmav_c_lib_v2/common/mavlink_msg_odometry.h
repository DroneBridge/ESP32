//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ODOMETRY_H
#define FASTMAVLINK_MSG_ODOMETRY_H


//----------------------------------------
//-- Message ODOMETRY
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_odometry_t {
    uint64_t time_usec;
    float x;
    float y;
    float z;
    float q[4];
    float vx;
    float vy;
    float vz;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    float pose_covariance[21];
    float velocity_covariance[21];
    uint8_t frame_id;
    uint8_t child_frame_id;
    uint8_t reset_counter;
    uint8_t estimator_type;
    int8_t quality;
}) fmav_odometry_t;


#define FASTMAVLINK_MSG_ID_ODOMETRY  331

#define FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX  233
#define FASTMAVLINK_MSG_ODOMETRY_CRCEXTRA  91

#define FASTMAVLINK_MSG_ODOMETRY_FLAGS  0
#define FASTMAVLINK_MSG_ODOMETRY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ODOMETRY_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ODOMETRY_FRAME_LEN_MAX  258

#define FASTMAVLINK_MSG_ODOMETRY_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_Q_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_POSE_COVARIANCE_NUM  21 // number of elements in array
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_POSE_COVARIANCE_LEN  84 // length of array = number of bytes
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_VELOCITY_COVARIANCE_NUM  21 // number of elements in array
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_VELOCITY_COVARIANCE_LEN  84 // length of array = number of bytes

#define FASTMAVLINK_MSG_ODOMETRY_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_X_OFS  8
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_Y_OFS  12
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_Z_OFS  16
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_Q_OFS  20
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_VX_OFS  36
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_VY_OFS  40
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_VZ_OFS  44
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_ROLLSPEED_OFS  48
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_PITCHSPEED_OFS  52
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_YAWSPEED_OFS  56
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_POSE_COVARIANCE_OFS  60
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_VELOCITY_COVARIANCE_OFS  144
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_FRAME_ID_OFS  228
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_CHILD_FRAME_ID_OFS  229
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_RESET_COUNTER_OFS  230
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_ESTIMATOR_TYPE_OFS  231
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_QUALITY_OFS  232


//----------------------------------------
//-- Message ODOMETRY pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float* q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float* pose_covariance, const float* velocity_covariance, uint8_t reset_counter, uint8_t estimator_type, int8_t quality,
    fmav_status_t* _status)
{
    fmav_odometry_t* _payload = (fmav_odometry_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->frame_id = frame_id;
    _payload->child_frame_id = child_frame_id;
    _payload->reset_counter = reset_counter;
    _payload->estimator_type = estimator_type;
    _payload->quality = quality;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->pose_covariance), pose_covariance, sizeof(float)*21);
    memcpy(&(_payload->velocity_covariance), velocity_covariance, sizeof(float)*21);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ODOMETRY;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ODOMETRY_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_odometry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_odometry_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->frame_id, _payload->child_frame_id, _payload->x, _payload->y, _payload->z, _payload->q, _payload->vx, _payload->vy, _payload->vz, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->pose_covariance, _payload->velocity_covariance, _payload->reset_counter, _payload->estimator_type, _payload->quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float* q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float* pose_covariance, const float* velocity_covariance, uint8_t reset_counter, uint8_t estimator_type, int8_t quality,
    fmav_status_t* _status)
{
    fmav_odometry_t* _payload = (fmav_odometry_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->frame_id = frame_id;
    _payload->child_frame_id = child_frame_id;
    _payload->reset_counter = reset_counter;
    _payload->estimator_type = estimator_type;
    _payload->quality = quality;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->pose_covariance), pose_covariance, sizeof(float)*21);
    memcpy(&(_payload->velocity_covariance), velocity_covariance, sizeof(float)*21);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ODOMETRY;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ODOMETRY >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ODOMETRY >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ODOMETRY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_odometry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_odometry_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->frame_id, _payload->child_frame_id, _payload->x, _payload->y, _payload->z, _payload->q, _payload->vx, _payload->vy, _payload->vz, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->pose_covariance, _payload->velocity_covariance, _payload->reset_counter, _payload->estimator_type, _payload->quality,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float* q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float* pose_covariance, const float* velocity_covariance, uint8_t reset_counter, uint8_t estimator_type, int8_t quality,
    fmav_status_t* _status)
{
    fmav_odometry_t _payload;

    _payload.time_usec = time_usec;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.rollspeed = rollspeed;
    _payload.pitchspeed = pitchspeed;
    _payload.yawspeed = yawspeed;
    _payload.frame_id = frame_id;
    _payload.child_frame_id = child_frame_id;
    _payload.reset_counter = reset_counter;
    _payload.estimator_type = estimator_type;
    _payload.quality = quality;
    memcpy(&(_payload.q), q, sizeof(float)*4);
    memcpy(&(_payload.pose_covariance), pose_covariance, sizeof(float)*21);
    memcpy(&(_payload.velocity_covariance), velocity_covariance, sizeof(float)*21);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ODOMETRY,
        FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ODOMETRY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_odometry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ODOMETRY,
        FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ODOMETRY_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ODOMETRY decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_odometry_decode(fmav_odometry_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_odometry_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_vx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_vy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_vz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_rollspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_pitchspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[52]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_yawspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[56]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_odometry_get_field_frame_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[228]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_odometry_get_field_child_frame_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[229]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_odometry_get_field_reset_counter(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[230]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_odometry_get_field_estimator_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[231]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_odometry_get_field_quality(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[232]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_odometry_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ODOMETRY_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[20]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_odometry_get_field_pose_covariance_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[60]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_pose_covariance(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ODOMETRY_FIELD_POSE_COVARIANCE_NUM) return 0;
    return ((float*)&(msg->payload[60]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_odometry_get_field_velocity_covariance_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[144]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_odometry_get_field_velocity_covariance(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ODOMETRY_FIELD_VELOCITY_COVARIANCE_NUM) return 0;
    return ((float*)&(msg->payload[144]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ODOMETRY  331

#define mavlink_odometry_t  fmav_odometry_t

#define MAVLINK_MSG_ID_ODOMETRY_LEN  233
#define MAVLINK_MSG_ID_ODOMETRY_MIN_LEN  230
#define MAVLINK_MSG_ID_331_LEN  233
#define MAVLINK_MSG_ID_331_MIN_LEN  230

#define MAVLINK_MSG_ID_ODOMETRY_CRC  91
#define MAVLINK_MSG_ID_331_CRC  91

#define MAVLINK_MSG_ODOMETRY_FIELD_Q_LEN 4
#define MAVLINK_MSG_ODOMETRY_FIELD_POSE_COVARIANCE_LEN 21
#define MAVLINK_MSG_ODOMETRY_FIELD_VELOCITY_COVARIANCE_LEN 21


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_odometry_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float* q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float* pose_covariance, const float* velocity_covariance, uint8_t reset_counter, uint8_t estimator_type, int8_t quality)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_odometry_pack(
        _msg, sysid, compid,
        time_usec, frame_id, child_frame_id, x, y, z, q, vx, vy, vz, rollspeed, pitchspeed, yawspeed, pose_covariance, velocity_covariance, reset_counter, estimator_type, quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_odometry_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_odometry_t* _payload)
{
    return mavlink_msg_odometry_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->frame_id, _payload->child_frame_id, _payload->x, _payload->y, _payload->z, _payload->q, _payload->vx, _payload->vy, _payload->vz, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->pose_covariance, _payload->velocity_covariance, _payload->reset_counter, _payload->estimator_type, _payload->quality);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_odometry_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float* q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float* pose_covariance, const float* velocity_covariance, uint8_t reset_counter, uint8_t estimator_type, int8_t quality)
{
    return fmav_msg_odometry_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, frame_id, child_frame_id, x, y, z, q, vx, vy, vz, rollspeed, pitchspeed, yawspeed, pose_covariance, velocity_covariance, reset_counter, estimator_type, quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_odometry_decode(const mavlink_message_t* msg, mavlink_odometry_t* payload)
{
    fmav_msg_odometry_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ODOMETRY_H
