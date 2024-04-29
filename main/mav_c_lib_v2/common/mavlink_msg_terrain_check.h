//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TERRAIN_CHECK_H
#define FASTMAVLINK_MSG_TERRAIN_CHECK_H


//----------------------------------------
//-- Message TERRAIN_CHECK
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_terrain_check_t {
    int32_t lat;
    int32_t lon;
}) fmav_terrain_check_t;


#define FASTMAVLINK_MSG_ID_TERRAIN_CHECK  135

#define FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX  8
#define FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA  203

#define FASTMAVLINK_MSG_TERRAIN_CHECK_FLAGS  0
#define FASTMAVLINK_MSG_TERRAIN_CHECK_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_CHECK_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TERRAIN_CHECK_FRAME_LEN_MAX  33



#define FASTMAVLINK_MSG_TERRAIN_CHECK_FIELD_LAT_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_CHECK_FIELD_LON_OFS  4


//----------------------------------------
//-- Message TERRAIN_CHECK pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon,
    fmav_status_t* _status)
{
    fmav_terrain_check_t* _payload = (fmav_terrain_check_t*)_msg->payload;

    _payload->lat = lat;
    _payload->lon = lon;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_TERRAIN_CHECK;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_check_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_check_pack(
        _msg, sysid, compid,
        _payload->lat, _payload->lon,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon,
    fmav_status_t* _status)
{
    fmav_terrain_check_t* _payload = (fmav_terrain_check_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->lat = lat;
    _payload->lon = lon;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TERRAIN_CHECK;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_CHECK >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_CHECK >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_check_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_check_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->lat, _payload->lon,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon,
    fmav_status_t* _status)
{
    fmav_terrain_check_t _payload;

    _payload.lat = lat;
    _payload.lon = lon;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_CHECK,
        FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_check_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_CHECK,
        FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TERRAIN_CHECK decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_terrain_check_decode(fmav_terrain_check_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_terrain_check_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_terrain_check_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TERRAIN_CHECK  135

#define mavlink_terrain_check_t  fmav_terrain_check_t

#define MAVLINK_MSG_ID_TERRAIN_CHECK_LEN  8
#define MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN  8
#define MAVLINK_MSG_ID_135_LEN  8
#define MAVLINK_MSG_ID_135_MIN_LEN  8

#define MAVLINK_MSG_ID_TERRAIN_CHECK_CRC  203
#define MAVLINK_MSG_ID_135_CRC  203




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_check_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    int32_t lat, int32_t lon)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_terrain_check_pack(
        _msg, sysid, compid,
        lat, lon,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_check_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_terrain_check_t* _payload)
{
    return mavlink_msg_terrain_check_pack(
        sysid,
        compid,
        _msg,
        _payload->lat, _payload->lon);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_check_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon)
{
    return fmav_msg_terrain_check_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        lat, lon,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_terrain_check_decode(const mavlink_message_t* msg, mavlink_terrain_check_t* payload)
{
    fmav_msg_terrain_check_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TERRAIN_CHECK_H
