//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MESSAGE_INTERVAL_H
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_H


//----------------------------------------
//-- Message MESSAGE_INTERVAL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_message_interval_t {
    int32_t interval_us;
    uint16_t message_id;
}) fmav_message_interval_t;


#define FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL  244

#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA  95

#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FLAGS  0
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FRAME_LEN_MAX  31



#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FIELD_INTERVAL_US_OFS  0
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FIELD_MESSAGE_ID_OFS  4


//----------------------------------------
//-- Message MESSAGE_INTERVAL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t message_id, int32_t interval_us,
    fmav_status_t* _status)
{
    fmav_message_interval_t* _payload = (fmav_message_interval_t*)_msg->payload;

    _payload->interval_us = interval_us;
    _payload->message_id = message_id;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_message_interval_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_message_interval_pack(
        _msg, sysid, compid,
        _payload->message_id, _payload->interval_us,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t message_id, int32_t interval_us,
    fmav_status_t* _status)
{
    fmav_message_interval_t* _payload = (fmav_message_interval_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->interval_us = interval_us;
    _payload->message_id = message_id;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_message_interval_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_message_interval_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->message_id, _payload->interval_us,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t message_id, int32_t interval_us,
    fmav_status_t* _status)
{
    fmav_message_interval_t _payload;

    _payload.interval_us = interval_us;
    _payload.message_id = message_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_message_interval_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MESSAGE_INTERVAL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_message_interval_decode(fmav_message_interval_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_message_interval_get_field_interval_us(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_get_field_message_id(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MESSAGE_INTERVAL  244

#define mavlink_message_interval_t  fmav_message_interval_t

#define MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN  6
#define MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN  6
#define MAVLINK_MSG_ID_244_LEN  6
#define MAVLINK_MSG_ID_244_MIN_LEN  6

#define MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC  95
#define MAVLINK_MSG_ID_244_CRC  95




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_message_interval_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint16_t message_id, int32_t interval_us)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_message_interval_pack(
        _msg, sysid, compid,
        message_id, interval_us,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_message_interval_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_message_interval_t* _payload)
{
    return mavlink_msg_message_interval_pack(
        sysid,
        compid,
        _msg,
        _payload->message_id, _payload->interval_us);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_message_interval_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t message_id, int32_t interval_us)
{
    return fmav_msg_message_interval_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        message_id, interval_us,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_message_interval_decode(const mavlink_message_t* msg, mavlink_message_interval_t* payload)
{
    fmav_msg_message_interval_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MESSAGE_INTERVAL_H
