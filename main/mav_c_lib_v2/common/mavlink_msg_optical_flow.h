//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OPTICAL_FLOW_H
#define FASTMAVLINK_MSG_OPTICAL_FLOW_H


//----------------------------------------
//-- Message OPTICAL_FLOW
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_optical_flow_t {
    uint64_t time_usec;
    float flow_comp_m_x;
    float flow_comp_m_y;
    float ground_distance;
    int16_t flow_x;
    int16_t flow_y;
    uint8_t sensor_id;
    uint8_t quality;
    float flow_rate_x;
    float flow_rate_y;
}) fmav_optical_flow_t;


#define FASTMAVLINK_MSG_ID_OPTICAL_FLOW  100

#define FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX  34
#define FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA  175

#define FASTMAVLINK_MSG_OPTICAL_FLOW_FLAGS  0
#define FASTMAVLINK_MSG_OPTICAL_FLOW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_OPTICAL_FLOW_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_OPTICAL_FLOW_FRAME_LEN_MAX  59



#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_COMP_M_X_OFS  8
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_COMP_M_Y_OFS  12
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_GROUND_DISTANCE_OFS  16
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_X_OFS  20
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_Y_OFS  22
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_SENSOR_ID_OFS  24
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_QUALITY_OFS  25
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_RATE_X_OFS  26
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_RATE_Y_OFS  30


//----------------------------------------
//-- Message OPTICAL_FLOW pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y,
    fmav_status_t* _status)
{
    fmav_optical_flow_t* _payload = (fmav_optical_flow_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->flow_comp_m_x = flow_comp_m_x;
    _payload->flow_comp_m_y = flow_comp_m_y;
    _payload->ground_distance = ground_distance;
    _payload->flow_x = flow_x;
    _payload->flow_y = flow_y;
    _payload->sensor_id = sensor_id;
    _payload->quality = quality;
    _payload->flow_rate_x = flow_rate_x;
    _payload->flow_rate_y = flow_rate_y;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_OPTICAL_FLOW;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_optical_flow_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_optical_flow_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->sensor_id, _payload->flow_x, _payload->flow_y, _payload->flow_comp_m_x, _payload->flow_comp_m_y, _payload->quality, _payload->ground_distance, _payload->flow_rate_x, _payload->flow_rate_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y,
    fmav_status_t* _status)
{
    fmav_optical_flow_t* _payload = (fmav_optical_flow_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->flow_comp_m_x = flow_comp_m_x;
    _payload->flow_comp_m_y = flow_comp_m_y;
    _payload->ground_distance = ground_distance;
    _payload->flow_x = flow_x;
    _payload->flow_y = flow_y;
    _payload->sensor_id = sensor_id;
    _payload->quality = quality;
    _payload->flow_rate_x = flow_rate_x;
    _payload->flow_rate_y = flow_rate_y;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OPTICAL_FLOW;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OPTICAL_FLOW >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OPTICAL_FLOW >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_optical_flow_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_optical_flow_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->sensor_id, _payload->flow_x, _payload->flow_y, _payload->flow_comp_m_x, _payload->flow_comp_m_y, _payload->quality, _payload->ground_distance, _payload->flow_rate_x, _payload->flow_rate_y,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y,
    fmav_status_t* _status)
{
    fmav_optical_flow_t _payload;

    _payload.time_usec = time_usec;
    _payload.flow_comp_m_x = flow_comp_m_x;
    _payload.flow_comp_m_y = flow_comp_m_y;
    _payload.ground_distance = ground_distance;
    _payload.flow_x = flow_x;
    _payload.flow_y = flow_y;
    _payload.sensor_id = sensor_id;
    _payload.quality = quality;
    _payload.flow_rate_x = flow_rate_x;
    _payload.flow_rate_y = flow_rate_y;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OPTICAL_FLOW,
        FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_optical_flow_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OPTICAL_FLOW,
        FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OPTICAL_FLOW decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_optical_flow_decode(fmav_optical_flow_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_optical_flow_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_flow_comp_m_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_flow_comp_m_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_ground_distance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_optical_flow_get_field_flow_x(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_optical_flow_get_field_flow_y(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_optical_flow_get_field_sensor_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_optical_flow_get_field_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[25]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_flow_rate_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[26]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_flow_rate_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[30]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OPTICAL_FLOW  100

#define mavlink_optical_flow_t  fmav_optical_flow_t

#define MAVLINK_MSG_ID_OPTICAL_FLOW_LEN  34
#define MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN  26
#define MAVLINK_MSG_ID_100_LEN  34
#define MAVLINK_MSG_ID_100_MIN_LEN  26

#define MAVLINK_MSG_ID_OPTICAL_FLOW_CRC  175
#define MAVLINK_MSG_ID_100_CRC  175




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_optical_flow_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_optical_flow_pack(
        _msg, sysid, compid,
        time_usec, sensor_id, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, ground_distance, flow_rate_x, flow_rate_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_optical_flow_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_optical_flow_t* _payload)
{
    return mavlink_msg_optical_flow_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->sensor_id, _payload->flow_x, _payload->flow_y, _payload->flow_comp_m_x, _payload->flow_comp_m_y, _payload->quality, _payload->ground_distance, _payload->flow_rate_x, _payload->flow_rate_y);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_optical_flow_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y)
{
    return fmav_msg_optical_flow_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, sensor_id, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, ground_distance, flow_rate_x, flow_rate_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_optical_flow_decode(const mavlink_message_t* msg, mavlink_optical_flow_t* payload)
{
    fmav_msg_optical_flow_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OPTICAL_FLOW_H
