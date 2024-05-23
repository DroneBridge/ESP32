//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TERRAIN_REQUEST_H
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_H


//----------------------------------------
//-- Message TERRAIN_REQUEST
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_terrain_request_t {
    uint64_t mask;
    int32_t lat;
    int32_t lon;
    uint16_t grid_spacing;
}) fmav_terrain_request_t;


#define FASTMAVLINK_MSG_ID_TERRAIN_REQUEST  133

#define FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX  18
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA  6

#define FASTMAVLINK_MSG_TERRAIN_REQUEST_FLAGS  0
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TERRAIN_REQUEST_FRAME_LEN_MAX  43



#define FASTMAVLINK_MSG_TERRAIN_REQUEST_FIELD_MASK_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_FIELD_LON_OFS  12
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_FIELD_GRID_SPACING_OFS  16


//----------------------------------------
//-- Message TERRAIN_REQUEST pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask,
    fmav_status_t* _status)
{
    fmav_terrain_request_t* _payload = (fmav_terrain_request_t*)_msg->payload;

    _payload->mask = mask;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->grid_spacing = grid_spacing;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_TERRAIN_REQUEST;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_request_pack(
        _msg, sysid, compid,
        _payload->lat, _payload->lon, _payload->grid_spacing, _payload->mask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask,
    fmav_status_t* _status)
{
    fmav_terrain_request_t* _payload = (fmav_terrain_request_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mask = mask;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->grid_spacing = grid_spacing;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TERRAIN_REQUEST;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_REQUEST >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_REQUEST >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_request_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->lat, _payload->lon, _payload->grid_spacing, _payload->mask,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask,
    fmav_status_t* _status)
{
    fmav_terrain_request_t _payload;

    _payload.mask = mask;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.grid_spacing = grid_spacing;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_REQUEST,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_REQUEST,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TERRAIN_REQUEST decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_terrain_request_decode(fmav_terrain_request_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_terrain_request_get_field_mask(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_terrain_request_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_terrain_request_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_get_field_grid_spacing(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TERRAIN_REQUEST  133

#define mavlink_terrain_request_t  fmav_terrain_request_t

#define MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN  18
#define MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN  18
#define MAVLINK_MSG_ID_133_LEN  18
#define MAVLINK_MSG_ID_133_MIN_LEN  18

#define MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC  6
#define MAVLINK_MSG_ID_133_CRC  6




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_request_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_terrain_request_pack(
        _msg, sysid, compid,
        lat, lon, grid_spacing, mask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_request_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_terrain_request_t* _payload)
{
    return mavlink_msg_terrain_request_pack(
        sysid,
        compid,
        _msg,
        _payload->lat, _payload->lon, _payload->grid_spacing, _payload->mask);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_request_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask)
{
    return fmav_msg_terrain_request_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        lat, lon, grid_spacing, mask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_terrain_request_decode(const mavlink_message_t* msg, mavlink_terrain_request_t* payload)
{
    fmav_msg_terrain_request_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TERRAIN_REQUEST_H
