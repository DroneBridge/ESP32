//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_H
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_H


//----------------------------------------
//-- Message SAFETY_ALLOWED_AREA
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_safety_allowed_area_t {
    float p1x;
    float p1y;
    float p1z;
    float p2x;
    float p2y;
    float p2z;
    uint8_t frame;
}) fmav_safety_allowed_area_t;


#define FASTMAVLINK_MSG_ID_SAFETY_ALLOWED_AREA  55

#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX  25
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_CRCEXTRA  3

#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_FLAGS  0
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_FRAME_LEN_MAX  50



#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_FIELD_P1X_OFS  0
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_FIELD_P1Y_OFS  4
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_FIELD_P1Z_OFS  8
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_FIELD_P2X_OFS  12
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_FIELD_P2Y_OFS  16
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_FIELD_P2Z_OFS  20
#define FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_FIELD_FRAME_OFS  24


//----------------------------------------
//-- Message SAFETY_ALLOWED_AREA pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_allowed_area_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z,
    fmav_status_t* _status)
{
    fmav_safety_allowed_area_t* _payload = (fmav_safety_allowed_area_t*)_msg->payload;

    _payload->p1x = p1x;
    _payload->p1y = p1y;
    _payload->p1z = p1z;
    _payload->p2x = p2x;
    _payload->p2y = p2y;
    _payload->p2z = p2z;
    _payload->frame = frame;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SAFETY_ALLOWED_AREA;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_allowed_area_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_safety_allowed_area_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_safety_allowed_area_pack(
        _msg, sysid, compid,
        _payload->frame, _payload->p1x, _payload->p1y, _payload->p1z, _payload->p2x, _payload->p2y, _payload->p2z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_allowed_area_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z,
    fmav_status_t* _status)
{
    fmav_safety_allowed_area_t* _payload = (fmav_safety_allowed_area_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->p1x = p1x;
    _payload->p1y = p1y;
    _payload->p1z = p1z;
    _payload->p2x = p2x;
    _payload->p2y = p2y;
    _payload->p2z = p2z;
    _payload->frame = frame;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SAFETY_ALLOWED_AREA;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SAFETY_ALLOWED_AREA >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SAFETY_ALLOWED_AREA >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_allowed_area_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_safety_allowed_area_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_safety_allowed_area_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->frame, _payload->p1x, _payload->p1y, _payload->p1z, _payload->p2x, _payload->p2y, _payload->p2z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_allowed_area_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z,
    fmav_status_t* _status)
{
    fmav_safety_allowed_area_t _payload;

    _payload.p1x = p1x;
    _payload.p1y = p1y;
    _payload.p1z = p1z;
    _payload.p2x = p2x;
    _payload.p2y = p2y;
    _payload.p2z = p2z;
    _payload.frame = frame;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SAFETY_ALLOWED_AREA,
        FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_allowed_area_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_safety_allowed_area_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SAFETY_ALLOWED_AREA,
        FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SAFETY_ALLOWED_AREA decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_safety_allowed_area_decode(fmav_safety_allowed_area_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_allowed_area_get_field_p1x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_allowed_area_get_field_p1y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_allowed_area_get_field_p1z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_allowed_area_get_field_p2x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_allowed_area_get_field_p2y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_allowed_area_get_field_p2z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_safety_allowed_area_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA  55

#define mavlink_safety_allowed_area_t  fmav_safety_allowed_area_t

#define MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA_LEN  25
#define MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA_MIN_LEN  25
#define MAVLINK_MSG_ID_55_LEN  25
#define MAVLINK_MSG_ID_55_MIN_LEN  25

#define MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA_CRC  3
#define MAVLINK_MSG_ID_55_CRC  3




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_safety_allowed_area_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_safety_allowed_area_pack(
        _msg, sysid, compid,
        frame, p1x, p1y, p1z, p2x, p2y, p2z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_safety_allowed_area_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_safety_allowed_area_t* _payload)
{
    return mavlink_msg_safety_allowed_area_pack(
        sysid,
        compid,
        _msg,
        _payload->frame, _payload->p1x, _payload->p1y, _payload->p1z, _payload->p2x, _payload->p2y, _payload->p2z);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_safety_allowed_area_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
    return fmav_msg_safety_allowed_area_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        frame, p1x, p1y, p1z, p2x, p2y, p2z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_safety_allowed_area_decode(const mavlink_message_t* msg, mavlink_safety_allowed_area_t* payload)
{
    fmav_msg_safety_allowed_area_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SAFETY_ALLOWED_AREA_H
