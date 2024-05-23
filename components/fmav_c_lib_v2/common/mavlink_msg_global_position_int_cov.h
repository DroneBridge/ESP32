//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_H
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_H


//----------------------------------------
//-- Message GLOBAL_POSITION_INT_COV
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_global_position_int_cov_t {
    uint64_t time_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    float vx;
    float vy;
    float vz;
    float covariance[36];
    uint8_t estimator_type;
}) fmav_global_position_int_cov_t;


#define FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV  63

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX  181
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_CRCEXTRA  119

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FLAGS  0
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FRAME_LEN_MAX  206

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_COVARIANCE_NUM  36 // number of elements in array
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_COVARIANCE_LEN  144 // length of array = number of bytes

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_LON_OFS  12
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_ALT_OFS  16
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_RELATIVE_ALT_OFS  20
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_VX_OFS  24
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_VY_OFS  28
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_VZ_OFS  32
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_COVARIANCE_OFS  36
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_ESTIMATOR_TYPE_OFS  180


//----------------------------------------
//-- Message GLOBAL_POSITION_INT_COV pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float* covariance,
    fmav_status_t* _status)
{
    fmav_global_position_int_cov_t* _payload = (fmav_global_position_int_cov_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->estimator_type = estimator_type;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*36);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_position_int_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_global_position_int_cov_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->estimator_type, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float* covariance,
    fmav_status_t* _status)
{
    fmav_global_position_int_cov_t* _payload = (fmav_global_position_int_cov_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->estimator_type = estimator_type;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*36);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_position_int_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_global_position_int_cov_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->estimator_type, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->covariance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float* covariance,
    fmav_status_t* _status)
{
    fmav_global_position_int_cov_t _payload;

    _payload.time_usec = time_usec;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.relative_alt = relative_alt;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.estimator_type = estimator_type;
    memcpy(&(_payload.covariance), covariance, sizeof(float)*36);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_position_int_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GLOBAL_POSITION_INT_COV decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_global_position_int_cov_decode(fmav_global_position_int_cov_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_global_position_int_cov_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_global_position_int_cov_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_global_position_int_cov_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_global_position_int_cov_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_global_position_int_cov_get_field_relative_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_position_int_cov_get_field_vx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_position_int_cov_get_field_vy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_position_int_cov_get_field_vz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_global_position_int_cov_get_field_estimator_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[180]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_global_position_int_cov_get_field_covariance_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[36]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_position_int_cov_get_field_covariance(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_COVARIANCE_NUM) return 0;
    return ((float*)&(msg->payload[36]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV  63

#define mavlink_global_position_int_cov_t  fmav_global_position_int_cov_t

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN  181
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN  181
#define MAVLINK_MSG_ID_63_LEN  181
#define MAVLINK_MSG_ID_63_MIN_LEN  181

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC  119
#define MAVLINK_MSG_ID_63_CRC  119

#define MAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_COVARIANCE_LEN 36


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_global_position_int_cov_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float* covariance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_global_position_int_cov_pack(
        _msg, sysid, compid,
        time_usec, estimator_type, lat, lon, alt, relative_alt, vx, vy, vz, covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_global_position_int_cov_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_global_position_int_cov_t* _payload)
{
    return mavlink_msg_global_position_int_cov_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->estimator_type, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->covariance);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_global_position_int_cov_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float* covariance)
{
    return fmav_msg_global_position_int_cov_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, estimator_type, lat, lon, alt, relative_alt, vx, vy, vz, covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_global_position_int_cov_decode(const mavlink_message_t* msg, mavlink_global_position_int_cov_t* payload)
{
    fmav_msg_global_position_int_cov_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_H
