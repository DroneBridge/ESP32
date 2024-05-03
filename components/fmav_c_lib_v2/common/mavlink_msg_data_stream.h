//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DATA_STREAM_H
#define FASTMAVLINK_MSG_DATA_STREAM_H


//----------------------------------------
//-- Message DATA_STREAM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_data_stream_t {
    uint16_t message_rate;
    uint8_t stream_id;
    uint8_t on_off;
}) fmav_data_stream_t;


#define FASTMAVLINK_MSG_ID_DATA_STREAM  67

#define FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX  4
#define FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA  21

#define FASTMAVLINK_MSG_DATA_STREAM_FLAGS  0
#define FASTMAVLINK_MSG_DATA_STREAM_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DATA_STREAM_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DATA_STREAM_FRAME_LEN_MAX  29



#define FASTMAVLINK_MSG_DATA_STREAM_FIELD_MESSAGE_RATE_OFS  0
#define FASTMAVLINK_MSG_DATA_STREAM_FIELD_STREAM_ID_OFS  2
#define FASTMAVLINK_MSG_DATA_STREAM_FIELD_ON_OFF_OFS  3


//----------------------------------------
//-- Message DATA_STREAM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off,
    fmav_status_t* _status)
{
    fmav_data_stream_t* _payload = (fmav_data_stream_t*)_msg->payload;

    _payload->message_rate = message_rate;
    _payload->stream_id = stream_id;
    _payload->on_off = on_off;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_DATA_STREAM;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data_stream_pack(
        _msg, sysid, compid,
        _payload->stream_id, _payload->message_rate, _payload->on_off,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off,
    fmav_status_t* _status)
{
    fmav_data_stream_t* _payload = (fmav_data_stream_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->message_rate = message_rate;
    _payload->stream_id = stream_id;
    _payload->on_off = on_off;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DATA_STREAM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA_STREAM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA_STREAM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data_stream_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->stream_id, _payload->message_rate, _payload->on_off,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off,
    fmav_status_t* _status)
{
    fmav_data_stream_t _payload;

    _payload.message_rate = message_rate;
    _payload.stream_id = stream_id;
    _payload.on_off = on_off;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DATA_STREAM,
        FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DATA_STREAM,
        FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DATA_STREAM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data_stream_decode(fmav_data_stream_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_get_field_message_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_stream_get_field_stream_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_stream_get_field_on_off(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DATA_STREAM  67

#define mavlink_data_stream_t  fmav_data_stream_t

#define MAVLINK_MSG_ID_DATA_STREAM_LEN  4
#define MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN  4
#define MAVLINK_MSG_ID_67_LEN  4
#define MAVLINK_MSG_ID_67_MIN_LEN  4

#define MAVLINK_MSG_ID_DATA_STREAM_CRC  21
#define MAVLINK_MSG_ID_67_CRC  21




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_stream_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_data_stream_pack(
        _msg, sysid, compid,
        stream_id, message_rate, on_off,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_stream_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_data_stream_t* _payload)
{
    return mavlink_msg_data_stream_pack(
        sysid,
        compid,
        _msg,
        _payload->stream_id, _payload->message_rate, _payload->on_off);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_stream_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off)
{
    return fmav_msg_data_stream_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        stream_id, message_rate, on_off,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_data_stream_decode(const mavlink_message_t* msg, mavlink_data_stream_t* payload)
{
    fmav_msg_data_stream_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DATA_STREAM_H
