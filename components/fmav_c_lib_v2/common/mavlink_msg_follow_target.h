//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FOLLOW_TARGET_H
#define FASTMAVLINK_MSG_FOLLOW_TARGET_H


//----------------------------------------
//-- Message FOLLOW_TARGET
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_follow_target_t {
    uint64_t timestamp;
    uint64_t custom_state;
    int32_t lat;
    int32_t lon;
    float alt;
    float vel[3];
    float acc[3];
    float attitude_q[4];
    float rates[3];
    float position_cov[3];
    uint8_t est_capabilities;
}) fmav_follow_target_t;


#define FASTMAVLINK_MSG_ID_FOLLOW_TARGET  144

#define FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX  93
#define FASTMAVLINK_MSG_FOLLOW_TARGET_CRCEXTRA  127

#define FASTMAVLINK_MSG_FOLLOW_TARGET_FLAGS  0
#define FASTMAVLINK_MSG_FOLLOW_TARGET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_FOLLOW_TARGET_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_FOLLOW_TARGET_FRAME_LEN_MAX  118

#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_VEL_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_VEL_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ACC_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ACC_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ATTITUDE_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ATTITUDE_Q_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_RATES_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_RATES_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_POSITION_COV_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_POSITION_COV_LEN  12 // length of array = number of bytes

#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_CUSTOM_STATE_OFS  8
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_LAT_OFS  16
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_LON_OFS  20
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ALT_OFS  24
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_VEL_OFS  28
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ACC_OFS  40
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ATTITUDE_Q_OFS  52
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_RATES_OFS  68
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_POSITION_COV_OFS  80
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_EST_CAPABILITIES_OFS  92


//----------------------------------------
//-- Message FOLLOW_TARGET pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t est_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* attitude_q, const float* rates, const float* position_cov, uint64_t custom_state,
    fmav_status_t* _status)
{
    fmav_follow_target_t* _payload = (fmav_follow_target_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->custom_state = custom_state;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->est_capabilities = est_capabilities;
    memcpy(&(_payload->vel), vel, sizeof(float)*3);
    memcpy(&(_payload->acc), acc, sizeof(float)*3);
    memcpy(&(_payload->attitude_q), attitude_q, sizeof(float)*4);
    memcpy(&(_payload->rates), rates, sizeof(float)*3);
    memcpy(&(_payload->position_cov), position_cov, sizeof(float)*3);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_FOLLOW_TARGET;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_FOLLOW_TARGET_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_follow_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_follow_target_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->est_capabilities, _payload->lat, _payload->lon, _payload->alt, _payload->vel, _payload->acc, _payload->attitude_q, _payload->rates, _payload->position_cov, _payload->custom_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t est_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* attitude_q, const float* rates, const float* position_cov, uint64_t custom_state,
    fmav_status_t* _status)
{
    fmav_follow_target_t* _payload = (fmav_follow_target_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->custom_state = custom_state;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->est_capabilities = est_capabilities;
    memcpy(&(_payload->vel), vel, sizeof(float)*3);
    memcpy(&(_payload->acc), acc, sizeof(float)*3);
    memcpy(&(_payload->attitude_q), attitude_q, sizeof(float)*4);
    memcpy(&(_payload->rates), rates, sizeof(float)*3);
    memcpy(&(_payload->position_cov), position_cov, sizeof(float)*3);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FOLLOW_TARGET;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FOLLOW_TARGET >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FOLLOW_TARGET >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FOLLOW_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_follow_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_follow_target_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->est_capabilities, _payload->lat, _payload->lon, _payload->alt, _payload->vel, _payload->acc, _payload->attitude_q, _payload->rates, _payload->position_cov, _payload->custom_state,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t est_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* attitude_q, const float* rates, const float* position_cov, uint64_t custom_state,
    fmav_status_t* _status)
{
    fmav_follow_target_t _payload;

    _payload.timestamp = timestamp;
    _payload.custom_state = custom_state;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.est_capabilities = est_capabilities;
    memcpy(&(_payload.vel), vel, sizeof(float)*3);
    memcpy(&(_payload.acc), acc, sizeof(float)*3);
    memcpy(&(_payload.attitude_q), attitude_q, sizeof(float)*4);
    memcpy(&(_payload.rates), rates, sizeof(float)*3);
    memcpy(&(_payload.position_cov), position_cov, sizeof(float)*3);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_FOLLOW_TARGET,
        FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FOLLOW_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_follow_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_FOLLOW_TARGET,
        FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FOLLOW_TARGET_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message FOLLOW_TARGET decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_follow_target_decode(fmav_follow_target_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_follow_target_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_follow_target_get_field_custom_state(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_follow_target_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_follow_target_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_follow_target_get_field_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_follow_target_get_field_est_capabilities(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[92]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_follow_target_get_field_vel_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[28]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_follow_target_get_field_vel(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_VEL_NUM) return 0;
    return ((float*)&(msg->payload[28]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_follow_target_get_field_acc_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[40]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_follow_target_get_field_acc(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ACC_NUM) return 0;
    return ((float*)&(msg->payload[40]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_follow_target_get_field_attitude_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[52]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_follow_target_get_field_attitude_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ATTITUDE_Q_NUM) return 0;
    return ((float*)&(msg->payload[52]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_follow_target_get_field_rates_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[68]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_follow_target_get_field_rates(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_RATES_NUM) return 0;
    return ((float*)&(msg->payload[68]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_follow_target_get_field_position_cov_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[80]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_follow_target_get_field_position_cov(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_POSITION_COV_NUM) return 0;
    return ((float*)&(msg->payload[80]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FOLLOW_TARGET  144

#define mavlink_follow_target_t  fmav_follow_target_t

#define MAVLINK_MSG_ID_FOLLOW_TARGET_LEN  93
#define MAVLINK_MSG_ID_FOLLOW_TARGET_MIN_LEN  93
#define MAVLINK_MSG_ID_144_LEN  93
#define MAVLINK_MSG_ID_144_MIN_LEN  93

#define MAVLINK_MSG_ID_FOLLOW_TARGET_CRC  127
#define MAVLINK_MSG_ID_144_CRC  127

#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_VEL_LEN 3
#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_ACC_LEN 3
#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_ATTITUDE_Q_LEN 4
#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_RATES_LEN 3
#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_POSITION_COV_LEN 3


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_follow_target_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, uint8_t est_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* attitude_q, const float* rates, const float* position_cov, uint64_t custom_state)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_follow_target_pack(
        _msg, sysid, compid,
        timestamp, est_capabilities, lat, lon, alt, vel, acc, attitude_q, rates, position_cov, custom_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_follow_target_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_follow_target_t* _payload)
{
    return mavlink_msg_follow_target_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->est_capabilities, _payload->lat, _payload->lon, _payload->alt, _payload->vel, _payload->acc, _payload->attitude_q, _payload->rates, _payload->position_cov, _payload->custom_state);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_follow_target_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t est_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* attitude_q, const float* rates, const float* position_cov, uint64_t custom_state)
{
    return fmav_msg_follow_target_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, est_capabilities, lat, lon, alt, vel, acc, attitude_q, rates, position_cov, custom_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_follow_target_decode(const mavlink_message_t* msg, mavlink_follow_target_t* payload)
{
    fmav_msg_follow_target_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FOLLOW_TARGET_H
