//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ATT_POS_MOCAP_H
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_H


//----------------------------------------
//-- Message ATT_POS_MOCAP
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_att_pos_mocap_t {
    uint64_t time_usec;
    float q[4];
    float x;
    float y;
    float z;
    float covariance[21];
}) fmav_att_pos_mocap_t;


#define FASTMAVLINK_MSG_ID_ATT_POS_MOCAP  138

#define FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX  120
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_CRCEXTRA  109

#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FLAGS  0
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FRAME_LEN_MAX  145

#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_Q_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_COVARIANCE_NUM  21 // number of elements in array
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_COVARIANCE_LEN  84 // length of array = number of bytes

#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_Q_OFS  8
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_X_OFS  24
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_Y_OFS  28
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_Z_OFS  32
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_COVARIANCE_OFS  36


//----------------------------------------
//-- Message ATT_POS_MOCAP pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float x, float y, float z, const float* covariance,
    fmav_status_t* _status)
{
    fmav_att_pos_mocap_t* _payload = (fmav_att_pos_mocap_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->covariance), covariance, sizeof(float)*21);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ATT_POS_MOCAP;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ATT_POS_MOCAP_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_att_pos_mocap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_att_pos_mocap_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->q, _payload->x, _payload->y, _payload->z, _payload->covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float x, float y, float z, const float* covariance,
    fmav_status_t* _status)
{
    fmav_att_pos_mocap_t* _payload = (fmav_att_pos_mocap_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->covariance), covariance, sizeof(float)*21);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ATT_POS_MOCAP;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ATT_POS_MOCAP >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ATT_POS_MOCAP >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_att_pos_mocap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_att_pos_mocap_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->q, _payload->x, _payload->y, _payload->z, _payload->covariance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float x, float y, float z, const float* covariance,
    fmav_status_t* _status)
{
    fmav_att_pos_mocap_t _payload;

    _payload.time_usec = time_usec;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    memcpy(&(_payload.q), q, sizeof(float)*4);
    memcpy(&(_payload.covariance), covariance, sizeof(float)*21);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ATT_POS_MOCAP,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_att_pos_mocap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ATT_POS_MOCAP,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ATT_POS_MOCAP decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_att_pos_mocap_decode(fmav_att_pos_mocap_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_att_pos_mocap_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_att_pos_mocap_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_att_pos_mocap_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_att_pos_mocap_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_att_pos_mocap_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_att_pos_mocap_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[8]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_att_pos_mocap_get_field_covariance_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[36]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_att_pos_mocap_get_field_covariance(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_COVARIANCE_NUM) return 0;
    return ((float*)&(msg->payload[36]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ATT_POS_MOCAP  138

#define mavlink_att_pos_mocap_t  fmav_att_pos_mocap_t

#define MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN  120
#define MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN  36
#define MAVLINK_MSG_ID_138_LEN  120
#define MAVLINK_MSG_ID_138_MIN_LEN  36

#define MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC  109
#define MAVLINK_MSG_ID_138_CRC  109

#define MAVLINK_MSG_ATT_POS_MOCAP_FIELD_Q_LEN 4
#define MAVLINK_MSG_ATT_POS_MOCAP_FIELD_COVARIANCE_LEN 21


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_att_pos_mocap_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, const float* q, float x, float y, float z, const float* covariance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_att_pos_mocap_pack(
        _msg, sysid, compid,
        time_usec, q, x, y, z, covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_att_pos_mocap_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_att_pos_mocap_t* _payload)
{
    return mavlink_msg_att_pos_mocap_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->q, _payload->x, _payload->y, _payload->z, _payload->covariance);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_att_pos_mocap_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float x, float y, float z, const float* covariance)
{
    return fmav_msg_att_pos_mocap_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, q, x, y, z, covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_att_pos_mocap_decode(const mavlink_message_t* msg, mavlink_att_pos_mocap_t* payload)
{
    fmav_msg_att_pos_mocap_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ATT_POS_MOCAP_H
