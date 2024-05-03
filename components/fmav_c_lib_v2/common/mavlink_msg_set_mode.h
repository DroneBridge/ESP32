//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_MODE_H
#define FASTMAVLINK_MSG_SET_MODE_H


//----------------------------------------
//-- Message SET_MODE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_mode_t {
    uint32_t custom_mode;
    uint8_t target_system;
    uint8_t base_mode;
}) fmav_set_mode_t;


#define FASTMAVLINK_MSG_ID_SET_MODE  11

#define FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_SET_MODE_CRCEXTRA  89

#define FASTMAVLINK_MSG_SET_MODE_FLAGS  1
#define FASTMAVLINK_MSG_SET_MODE_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_SET_MODE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SET_MODE_FRAME_LEN_MAX  31



#define FASTMAVLINK_MSG_SET_MODE_FIELD_CUSTOM_MODE_OFS  0
#define FASTMAVLINK_MSG_SET_MODE_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_SET_MODE_FIELD_BASE_MODE_OFS  5


//----------------------------------------
//-- Message SET_MODE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t base_mode, uint32_t custom_mode,
    fmav_status_t* _status)
{
    fmav_set_mode_t* _payload = (fmav_set_mode_t*)_msg->payload;

    _payload->custom_mode = custom_mode;
    _payload->target_system = target_system;
    _payload->base_mode = base_mode;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SET_MODE;
    _msg->target_sysid = target_system;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SET_MODE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_mode_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->base_mode, _payload->custom_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t base_mode, uint32_t custom_mode,
    fmav_status_t* _status)
{
    fmav_set_mode_t* _payload = (fmav_set_mode_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->custom_mode = custom_mode;
    _payload->target_system = target_system;
    _payload->base_mode = base_mode;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_MODE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_MODE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_MODE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_MODE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_mode_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->base_mode, _payload->custom_mode,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t base_mode, uint32_t custom_mode,
    fmav_status_t* _status)
{
    fmav_set_mode_t _payload;

    _payload.custom_mode = custom_mode;
    _payload.target_system = target_system;
    _payload.base_mode = base_mode;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SET_MODE,
        FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_MODE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SET_MODE,
        FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_MODE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SET_MODE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_mode_decode(fmav_set_mode_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_set_mode_get_field_custom_mode(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_mode_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_mode_get_field_base_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_MODE  11

#define mavlink_set_mode_t  fmav_set_mode_t

#define MAVLINK_MSG_ID_SET_MODE_LEN  6
#define MAVLINK_MSG_ID_SET_MODE_MIN_LEN  6
#define MAVLINK_MSG_ID_11_LEN  6
#define MAVLINK_MSG_ID_11_MIN_LEN  6

#define MAVLINK_MSG_ID_SET_MODE_CRC  89
#define MAVLINK_MSG_ID_11_CRC  89




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_mode_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t base_mode, uint32_t custom_mode)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_mode_pack(
        _msg, sysid, compid,
        target_system, base_mode, custom_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_mode_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_set_mode_t* _payload)
{
    return mavlink_msg_set_mode_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->base_mode, _payload->custom_mode);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_mode_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t base_mode, uint32_t custom_mode)
{
    return fmav_msg_set_mode_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, base_mode, custom_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_mode_decode(const mavlink_message_t* msg, mavlink_set_mode_t* payload)
{
    fmav_msg_set_mode_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_MODE_H
