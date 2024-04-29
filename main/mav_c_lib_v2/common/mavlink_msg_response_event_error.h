//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_H
#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_H


//----------------------------------------
//-- Message RESPONSE_EVENT_ERROR
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_response_event_error_t {
    uint16_t sequence;
    uint16_t sequence_oldest_available;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t reason;
}) fmav_response_event_error_t;


#define FASTMAVLINK_MSG_ID_RESPONSE_EVENT_ERROR  413

#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX  7
#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_CRCEXTRA  77

#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_FLAGS  3
#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_FRAME_LEN_MAX  32



#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_FIELD_SEQUENCE_OFS  0
#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_FIELD_SEQUENCE_OLDEST_AVAILABLE_OFS  2
#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_FIELD_TARGET_COMPONENT_OFS  5
#define FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_FIELD_REASON_OFS  6


//----------------------------------------
//-- Message RESPONSE_EVENT_ERROR pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_response_event_error_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint16_t sequence_oldest_available, uint8_t reason,
    fmav_status_t* _status)
{
    fmav_response_event_error_t* _payload = (fmav_response_event_error_t*)_msg->payload;

    _payload->sequence = sequence;
    _payload->sequence_oldest_available = sequence_oldest_available;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->reason = reason;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RESPONSE_EVENT_ERROR;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_response_event_error_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_response_event_error_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_response_event_error_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->sequence, _payload->sequence_oldest_available, _payload->reason,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_response_event_error_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint16_t sequence_oldest_available, uint8_t reason,
    fmav_status_t* _status)
{
    fmav_response_event_error_t* _payload = (fmav_response_event_error_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->sequence = sequence;
    _payload->sequence_oldest_available = sequence_oldest_available;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->reason = reason;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RESPONSE_EVENT_ERROR;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RESPONSE_EVENT_ERROR >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RESPONSE_EVENT_ERROR >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_response_event_error_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_response_event_error_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_response_event_error_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->sequence, _payload->sequence_oldest_available, _payload->reason,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_response_event_error_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint16_t sequence_oldest_available, uint8_t reason,
    fmav_status_t* _status)
{
    fmav_response_event_error_t _payload;

    _payload.sequence = sequence;
    _payload.sequence_oldest_available = sequence_oldest_available;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.reason = reason;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RESPONSE_EVENT_ERROR,
        FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_response_event_error_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_response_event_error_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RESPONSE_EVENT_ERROR,
        FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RESPONSE_EVENT_ERROR decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_response_event_error_decode(fmav_response_event_error_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_response_event_error_get_field_sequence(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_response_event_error_get_field_sequence_oldest_available(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_response_event_error_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_response_event_error_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_response_event_error_get_field_reason(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR  413

#define mavlink_response_event_error_t  fmav_response_event_error_t

#define MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN  7
#define MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_MIN_LEN  7
#define MAVLINK_MSG_ID_413_LEN  7
#define MAVLINK_MSG_ID_413_MIN_LEN  7

#define MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_CRC  77
#define MAVLINK_MSG_ID_413_CRC  77




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_response_event_error_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint16_t sequence_oldest_available, uint8_t reason)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_response_event_error_pack(
        _msg, sysid, compid,
        target_system, target_component, sequence, sequence_oldest_available, reason,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_response_event_error_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_response_event_error_t* _payload)
{
    return mavlink_msg_response_event_error_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->sequence, _payload->sequence_oldest_available, _payload->reason);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_response_event_error_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint16_t sequence_oldest_available, uint8_t reason)
{
    return fmav_msg_response_event_error_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, sequence, sequence_oldest_available, reason,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_response_event_error_decode(const mavlink_message_t* msg, mavlink_response_event_error_t* payload)
{
    fmav_msg_response_event_error_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RESPONSE_EVENT_ERROR_H
