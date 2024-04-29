//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ALTITUDE_H
#define FASTMAVLINK_MSG_ALTITUDE_H


//----------------------------------------
//-- Message ALTITUDE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_altitude_t {
    uint64_t time_usec;
    float altitude_monotonic;
    float altitude_amsl;
    float altitude_local;
    float altitude_relative;
    float altitude_terrain;
    float bottom_clearance;
}) fmav_altitude_t;


#define FASTMAVLINK_MSG_ID_ALTITUDE  141

#define FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_ALTITUDE_CRCEXTRA  47

#define FASTMAVLINK_MSG_ALTITUDE_FLAGS  0
#define FASTMAVLINK_MSG_ALTITUDE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ALTITUDE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ALTITUDE_FRAME_LEN_MAX  57



#define FASTMAVLINK_MSG_ALTITUDE_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ALTITUDE_FIELD_ALTITUDE_MONOTONIC_OFS  8
#define FASTMAVLINK_MSG_ALTITUDE_FIELD_ALTITUDE_AMSL_OFS  12
#define FASTMAVLINK_MSG_ALTITUDE_FIELD_ALTITUDE_LOCAL_OFS  16
#define FASTMAVLINK_MSG_ALTITUDE_FIELD_ALTITUDE_RELATIVE_OFS  20
#define FASTMAVLINK_MSG_ALTITUDE_FIELD_ALTITUDE_TERRAIN_OFS  24
#define FASTMAVLINK_MSG_ALTITUDE_FIELD_BOTTOM_CLEARANCE_OFS  28


//----------------------------------------
//-- Message ALTITUDE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_altitude_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float altitude_monotonic, float altitude_amsl, float altitude_local, float altitude_relative, float altitude_terrain, float bottom_clearance,
    fmav_status_t* _status)
{
    fmav_altitude_t* _payload = (fmav_altitude_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->altitude_monotonic = altitude_monotonic;
    _payload->altitude_amsl = altitude_amsl;
    _payload->altitude_local = altitude_local;
    _payload->altitude_relative = altitude_relative;
    _payload->altitude_terrain = altitude_terrain;
    _payload->bottom_clearance = bottom_clearance;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ALTITUDE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ALTITUDE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_altitude_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_altitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_altitude_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->altitude_monotonic, _payload->altitude_amsl, _payload->altitude_local, _payload->altitude_relative, _payload->altitude_terrain, _payload->bottom_clearance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_altitude_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float altitude_monotonic, float altitude_amsl, float altitude_local, float altitude_relative, float altitude_terrain, float bottom_clearance,
    fmav_status_t* _status)
{
    fmav_altitude_t* _payload = (fmav_altitude_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->altitude_monotonic = altitude_monotonic;
    _payload->altitude_amsl = altitude_amsl;
    _payload->altitude_local = altitude_local;
    _payload->altitude_relative = altitude_relative;
    _payload->altitude_terrain = altitude_terrain;
    _payload->bottom_clearance = bottom_clearance;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ALTITUDE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ALTITUDE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ALTITUDE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ALTITUDE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_altitude_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_altitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_altitude_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->altitude_monotonic, _payload->altitude_amsl, _payload->altitude_local, _payload->altitude_relative, _payload->altitude_terrain, _payload->bottom_clearance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_altitude_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float altitude_monotonic, float altitude_amsl, float altitude_local, float altitude_relative, float altitude_terrain, float bottom_clearance,
    fmav_status_t* _status)
{
    fmav_altitude_t _payload;

    _payload.time_usec = time_usec;
    _payload.altitude_monotonic = altitude_monotonic;
    _payload.altitude_amsl = altitude_amsl;
    _payload.altitude_local = altitude_local;
    _payload.altitude_relative = altitude_relative;
    _payload.altitude_terrain = altitude_terrain;
    _payload.bottom_clearance = bottom_clearance;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ALTITUDE,
        FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ALTITUDE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_altitude_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_altitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ALTITUDE,
        FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ALTITUDE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ALTITUDE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_altitude_decode(fmav_altitude_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ALTITUDE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_altitude_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_altitude_get_field_altitude_monotonic(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_altitude_get_field_altitude_amsl(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_altitude_get_field_altitude_local(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_altitude_get_field_altitude_relative(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_altitude_get_field_altitude_terrain(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_altitude_get_field_bottom_clearance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ALTITUDE  141

#define mavlink_altitude_t  fmav_altitude_t

#define MAVLINK_MSG_ID_ALTITUDE_LEN  32
#define MAVLINK_MSG_ID_ALTITUDE_MIN_LEN  32
#define MAVLINK_MSG_ID_141_LEN  32
#define MAVLINK_MSG_ID_141_MIN_LEN  32

#define MAVLINK_MSG_ID_ALTITUDE_CRC  47
#define MAVLINK_MSG_ID_141_CRC  47




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_altitude_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, float altitude_monotonic, float altitude_amsl, float altitude_local, float altitude_relative, float altitude_terrain, float bottom_clearance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_altitude_pack(
        _msg, sysid, compid,
        time_usec, altitude_monotonic, altitude_amsl, altitude_local, altitude_relative, altitude_terrain, bottom_clearance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_altitude_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_altitude_t* _payload)
{
    return mavlink_msg_altitude_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->altitude_monotonic, _payload->altitude_amsl, _payload->altitude_local, _payload->altitude_relative, _payload->altitude_terrain, _payload->bottom_clearance);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_altitude_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float altitude_monotonic, float altitude_amsl, float altitude_local, float altitude_relative, float altitude_terrain, float bottom_clearance)
{
    return fmav_msg_altitude_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, altitude_monotonic, altitude_amsl, altitude_local, altitude_relative, altitude_terrain, bottom_clearance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_altitude_decode(const mavlink_message_t* msg, mavlink_altitude_t* payload)
{
    fmav_msg_altitude_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ALTITUDE_H
