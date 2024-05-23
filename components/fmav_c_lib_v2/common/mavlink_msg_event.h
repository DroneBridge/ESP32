//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_EVENT_H
#define FASTMAVLINK_MSG_EVENT_H


//----------------------------------------
//-- Message EVENT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_event_t {
    uint32_t id;
    uint32_t event_time_boot_ms;
    uint16_t sequence;
    uint8_t destination_component;
    uint8_t destination_system;
    uint8_t log_levels;
    uint8_t arguments[40];
}) fmav_event_t;


#define FASTMAVLINK_MSG_ID_EVENT  410

#define FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX  53
#define FASTMAVLINK_MSG_EVENT_CRCEXTRA  160

#define FASTMAVLINK_MSG_EVENT_FLAGS  0
#define FASTMAVLINK_MSG_EVENT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_EVENT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_EVENT_FRAME_LEN_MAX  78

#define FASTMAVLINK_MSG_EVENT_FIELD_ARGUMENTS_NUM  40 // number of elements in array
#define FASTMAVLINK_MSG_EVENT_FIELD_ARGUMENTS_LEN  40 // length of array = number of bytes

#define FASTMAVLINK_MSG_EVENT_FIELD_ID_OFS  0
#define FASTMAVLINK_MSG_EVENT_FIELD_EVENT_TIME_BOOT_MS_OFS  4
#define FASTMAVLINK_MSG_EVENT_FIELD_SEQUENCE_OFS  8
#define FASTMAVLINK_MSG_EVENT_FIELD_DESTINATION_COMPONENT_OFS  10
#define FASTMAVLINK_MSG_EVENT_FIELD_DESTINATION_SYSTEM_OFS  11
#define FASTMAVLINK_MSG_EVENT_FIELD_LOG_LEVELS_OFS  12
#define FASTMAVLINK_MSG_EVENT_FIELD_ARGUMENTS_OFS  13


//----------------------------------------
//-- Message EVENT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_event_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t destination_component, uint8_t destination_system, uint32_t id, uint32_t event_time_boot_ms, uint16_t sequence, uint8_t log_levels, const uint8_t* arguments,
    fmav_status_t* _status)
{
    fmav_event_t* _payload = (fmav_event_t*)_msg->payload;

    _payload->id = id;
    _payload->event_time_boot_ms = event_time_boot_ms;
    _payload->sequence = sequence;
    _payload->destination_component = destination_component;
    _payload->destination_system = destination_system;
    _payload->log_levels = log_levels;
    memcpy(&(_payload->arguments), arguments, sizeof(uint8_t)*40);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_EVENT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_EVENT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_event_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_event_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_event_pack(
        _msg, sysid, compid,
        _payload->destination_component, _payload->destination_system, _payload->id, _payload->event_time_boot_ms, _payload->sequence, _payload->log_levels, _payload->arguments,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_event_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t destination_component, uint8_t destination_system, uint32_t id, uint32_t event_time_boot_ms, uint16_t sequence, uint8_t log_levels, const uint8_t* arguments,
    fmav_status_t* _status)
{
    fmav_event_t* _payload = (fmav_event_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->id = id;
    _payload->event_time_boot_ms = event_time_boot_ms;
    _payload->sequence = sequence;
    _payload->destination_component = destination_component;
    _payload->destination_system = destination_system;
    _payload->log_levels = log_levels;
    memcpy(&(_payload->arguments), arguments, sizeof(uint8_t)*40);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_EVENT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_EVENT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_EVENT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EVENT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_event_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_event_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_event_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->destination_component, _payload->destination_system, _payload->id, _payload->event_time_boot_ms, _payload->sequence, _payload->log_levels, _payload->arguments,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_event_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t destination_component, uint8_t destination_system, uint32_t id, uint32_t event_time_boot_ms, uint16_t sequence, uint8_t log_levels, const uint8_t* arguments,
    fmav_status_t* _status)
{
    fmav_event_t _payload;

    _payload.id = id;
    _payload.event_time_boot_ms = event_time_boot_ms;
    _payload.sequence = sequence;
    _payload.destination_component = destination_component;
    _payload.destination_system = destination_system;
    _payload.log_levels = log_levels;
    memcpy(&(_payload.arguments), arguments, sizeof(uint8_t)*40);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_EVENT,
        FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EVENT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_event_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_event_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_EVENT,
        FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EVENT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message EVENT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_event_decode(fmav_event_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_EVENT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_event_get_field_id(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_event_get_field_event_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_event_get_field_sequence(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_event_get_field_destination_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_event_get_field_destination_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_event_get_field_log_levels(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_event_get_field_arguments_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[13]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_event_get_field_arguments(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_EVENT_FIELD_ARGUMENTS_NUM) return 0;
    return ((uint8_t*)&(msg->payload[13]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_EVENT  410

#define mavlink_event_t  fmav_event_t

#define MAVLINK_MSG_ID_EVENT_LEN  53
#define MAVLINK_MSG_ID_EVENT_MIN_LEN  53
#define MAVLINK_MSG_ID_410_LEN  53
#define MAVLINK_MSG_ID_410_MIN_LEN  53

#define MAVLINK_MSG_ID_EVENT_CRC  160
#define MAVLINK_MSG_ID_410_CRC  160

#define MAVLINK_MSG_EVENT_FIELD_ARGUMENTS_LEN 40


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_event_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t destination_component, uint8_t destination_system, uint32_t id, uint32_t event_time_boot_ms, uint16_t sequence, uint8_t log_levels, const uint8_t* arguments)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_event_pack(
        _msg, sysid, compid,
        destination_component, destination_system, id, event_time_boot_ms, sequence, log_levels, arguments,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_event_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_event_t* _payload)
{
    return mavlink_msg_event_pack(
        sysid,
        compid,
        _msg,
        _payload->destination_component, _payload->destination_system, _payload->id, _payload->event_time_boot_ms, _payload->sequence, _payload->log_levels, _payload->arguments);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_event_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t destination_component, uint8_t destination_system, uint32_t id, uint32_t event_time_boot_ms, uint16_t sequence, uint8_t log_levels, const uint8_t* arguments)
{
    return fmav_msg_event_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        destination_component, destination_system, id, event_time_boot_ms, sequence, log_levels, arguments,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_event_decode(const mavlink_message_t* msg, mavlink_event_t* payload)
{
    fmav_msg_event_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_EVENT_H
