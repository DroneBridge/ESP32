//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VFR_HUD_H
#define FASTMAVLINK_MSG_VFR_HUD_H


//----------------------------------------
//-- Message VFR_HUD
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_vfr_hud_t {
    float airspeed;
    float groundspeed;
    float alt;
    float climb;
    int16_t heading;
    uint16_t throttle;
}) fmav_vfr_hud_t;


#define FASTMAVLINK_MSG_ID_VFR_HUD  74

#define FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA  20

#define FASTMAVLINK_MSG_VFR_HUD_FLAGS  0
#define FASTMAVLINK_MSG_VFR_HUD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VFR_HUD_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_VFR_HUD_FRAME_LEN_MAX  45



#define FASTMAVLINK_MSG_VFR_HUD_FIELD_AIRSPEED_OFS  0
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_GROUNDSPEED_OFS  4
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_ALT_OFS  8
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_CLIMB_OFS  12
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_HEADING_OFS  16
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_THROTTLE_OFS  18


//----------------------------------------
//-- Message VFR_HUD pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb,
    fmav_status_t* _status)
{
    fmav_vfr_hud_t* _payload = (fmav_vfr_hud_t*)_msg->payload;

    _payload->airspeed = airspeed;
    _payload->groundspeed = groundspeed;
    _payload->alt = alt;
    _payload->climb = climb;
    _payload->heading = heading;
    _payload->throttle = throttle;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_VFR_HUD;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vfr_hud_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vfr_hud_pack(
        _msg, sysid, compid,
        _payload->airspeed, _payload->groundspeed, _payload->heading, _payload->throttle, _payload->alt, _payload->climb,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb,
    fmav_status_t* _status)
{
    fmav_vfr_hud_t* _payload = (fmav_vfr_hud_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->airspeed = airspeed;
    _payload->groundspeed = groundspeed;
    _payload->alt = alt;
    _payload->climb = climb;
    _payload->heading = heading;
    _payload->throttle = throttle;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VFR_HUD;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VFR_HUD >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VFR_HUD >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vfr_hud_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vfr_hud_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->airspeed, _payload->groundspeed, _payload->heading, _payload->throttle, _payload->alt, _payload->climb,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb,
    fmav_status_t* _status)
{
    fmav_vfr_hud_t _payload;

    _payload.airspeed = airspeed;
    _payload.groundspeed = groundspeed;
    _payload.alt = alt;
    _payload.climb = climb;
    _payload.heading = heading;
    _payload.throttle = throttle;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_VFR_HUD,
        FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_vfr_hud_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_VFR_HUD,
        FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message VFR_HUD decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_vfr_hud_decode(fmav_vfr_hud_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vfr_hud_get_field_airspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vfr_hud_get_field_groundspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vfr_hud_get_field_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vfr_hud_get_field_climb(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_vfr_hud_get_field_heading(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_get_field_throttle(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VFR_HUD  74

#define mavlink_vfr_hud_t  fmav_vfr_hud_t

#define MAVLINK_MSG_ID_VFR_HUD_LEN  20
#define MAVLINK_MSG_ID_VFR_HUD_MIN_LEN  20
#define MAVLINK_MSG_ID_74_LEN  20
#define MAVLINK_MSG_ID_74_MIN_LEN  20

#define MAVLINK_MSG_ID_VFR_HUD_CRC  20
#define MAVLINK_MSG_ID_74_CRC  20




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vfr_hud_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_vfr_hud_pack(
        _msg, sysid, compid,
        airspeed, groundspeed, heading, throttle, alt, climb,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vfr_hud_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_vfr_hud_t* _payload)
{
    return mavlink_msg_vfr_hud_pack(
        sysid,
        compid,
        _msg,
        _payload->airspeed, _payload->groundspeed, _payload->heading, _payload->throttle, _payload->alt, _payload->climb);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vfr_hud_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
{
    return fmav_msg_vfr_hud_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        airspeed, groundspeed, heading, throttle, alt, climb,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_vfr_hud_decode(const mavlink_message_t* msg, mavlink_vfr_hud_t* payload)
{
    fmav_msg_vfr_hud_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VFR_HUD_H
