//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_REQUEST_DATA_STREAM_H
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_H


//----------------------------------------
//-- Message REQUEST_DATA_STREAM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_request_data_stream_t {
    uint16_t req_message_rate;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t req_stream_id;
    uint8_t start_stop;
}) fmav_request_data_stream_t;


#define FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM  66

#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA  148

#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FLAGS  3
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_TARGET_COMPONENT_OFS  3

#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FRAME_LEN_MAX  31



#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_REQ_MESSAGE_RATE_OFS  0
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_TARGET_COMPONENT_OFS  3
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_REQ_STREAM_ID_OFS  4
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_START_STOP_OFS  5


//----------------------------------------
//-- Message REQUEST_DATA_STREAM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop,
    fmav_status_t* _status)
{
    fmav_request_data_stream_t* _payload = (fmav_request_data_stream_t*)_msg->payload;

    _payload->req_message_rate = req_message_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->req_stream_id = req_stream_id;
    _payload->start_stop = start_stop;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_request_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_request_data_stream_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->req_stream_id, _payload->req_message_rate, _payload->start_stop,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop,
    fmav_status_t* _status)
{
    fmav_request_data_stream_t* _payload = (fmav_request_data_stream_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->req_message_rate = req_message_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->req_stream_id = req_stream_id;
    _payload->start_stop = start_stop;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_request_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_request_data_stream_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->req_stream_id, _payload->req_message_rate, _payload->start_stop,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop,
    fmav_status_t* _status)
{
    fmav_request_data_stream_t _payload;

    _payload.req_message_rate = req_message_rate;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.req_stream_id = req_stream_id;
    _payload.start_stop = start_stop;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_request_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message REQUEST_DATA_STREAM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_request_data_stream_decode(fmav_request_data_stream_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_get_field_req_message_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_request_data_stream_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_request_data_stream_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_request_data_stream_get_field_req_stream_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_request_data_stream_get_field_start_stop(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM  66

#define mavlink_request_data_stream_t  fmav_request_data_stream_t

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN  6
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MIN_LEN  6
#define MAVLINK_MSG_ID_66_LEN  6
#define MAVLINK_MSG_ID_66_MIN_LEN  6

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC  148
#define MAVLINK_MSG_ID_66_CRC  148




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_request_data_stream_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_request_data_stream_pack(
        _msg, sysid, compid,
        target_system, target_component, req_stream_id, req_message_rate, start_stop,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_request_data_stream_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_request_data_stream_t* _payload)
{
    return mavlink_msg_request_data_stream_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->req_stream_id, _payload->req_message_rate, _payload->start_stop);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_request_data_stream_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
    return fmav_msg_request_data_stream_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, req_stream_id, req_message_rate, start_stop,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_request_data_stream_decode(const mavlink_message_t* msg, mavlink_request_data_stream_t* payload)
{
    fmav_msg_request_data_stream_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_REQUEST_DATA_STREAM_H
