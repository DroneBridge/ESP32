//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SERIAL_CONTROL_H
#define FASTMAVLINK_MSG_SERIAL_CONTROL_H


//----------------------------------------
//-- Message SERIAL_CONTROL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_serial_control_t {
    uint32_t baudrate;
    uint16_t timeout;
    uint8_t device;
    uint8_t flags;
    uint8_t count;
    uint8_t data[70];
    uint8_t target_system;
    uint8_t target_component;
}) fmav_serial_control_t;


#define FASTMAVLINK_MSG_ID_SERIAL_CONTROL  126

#define FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX  81
#define FASTMAVLINK_MSG_SERIAL_CONTROL_CRCEXTRA  220

#define FASTMAVLINK_MSG_SERIAL_CONTROL_FLAGS  3
#define FASTMAVLINK_MSG_SERIAL_CONTROL_TARGET_SYSTEM_OFS  79
#define FASTMAVLINK_MSG_SERIAL_CONTROL_TARGET_COMPONENT_OFS  80

#define FASTMAVLINK_MSG_SERIAL_CONTROL_FRAME_LEN_MAX  106

#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_NUM  70 // number of elements in array
#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN  70 // length of array = number of bytes

#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_BAUDRATE_OFS  0
#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_TIMEOUT_OFS  4
#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_DEVICE_OFS  6
#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_FLAGS_OFS  7
#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_COUNT_OFS  8
#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_OFS  9
#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_TARGET_SYSTEM_OFS  79
#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_TARGET_COMPONENT_OFS  80


//----------------------------------------
//-- Message SERIAL_CONTROL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t* data, uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_serial_control_t* _payload = (fmav_serial_control_t*)_msg->payload;

    _payload->baudrate = baudrate;
    _payload->timeout = timeout;
    _payload->device = device;
    _payload->flags = flags;
    _payload->count = count;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*70);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SERIAL_CONTROL;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_SERIAL_CONTROL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_serial_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_serial_control_pack(
        _msg, sysid, compid,
        _payload->device, _payload->flags, _payload->timeout, _payload->baudrate, _payload->count, _payload->data, _payload->target_system, _payload->target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t* data, uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_serial_control_t* _payload = (fmav_serial_control_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->baudrate = baudrate;
    _payload->timeout = timeout;
    _payload->device = device;
    _payload->flags = flags;
    _payload->count = count;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*70);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SERIAL_CONTROL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SERIAL_CONTROL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SERIAL_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SERIAL_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_serial_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_serial_control_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->device, _payload->flags, _payload->timeout, _payload->baudrate, _payload->count, _payload->data, _payload->target_system, _payload->target_component,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t* data, uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_serial_control_t _payload;

    _payload.baudrate = baudrate;
    _payload.timeout = timeout;
    _payload.device = device;
    _payload.flags = flags;
    _payload.count = count;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*70);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SERIAL_CONTROL,
        FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SERIAL_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_serial_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SERIAL_CONTROL,
        FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SERIAL_CONTROL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SERIAL_CONTROL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_serial_control_decode(fmav_serial_control_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_serial_control_get_field_baudrate(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_get_field_timeout(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_serial_control_get_field_device(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_serial_control_get_field_flags(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_serial_control_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_serial_control_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[79]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_serial_control_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[80]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_serial_control_get_field_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[9]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_serial_control_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[9]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SERIAL_CONTROL  126

#define mavlink_serial_control_t  fmav_serial_control_t

#define MAVLINK_MSG_ID_SERIAL_CONTROL_LEN  81
#define MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN  79
#define MAVLINK_MSG_ID_126_LEN  81
#define MAVLINK_MSG_ID_126_MIN_LEN  79

#define MAVLINK_MSG_ID_SERIAL_CONTROL_CRC  220
#define MAVLINK_MSG_ID_126_CRC  220

#define MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN 70


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_serial_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t* data, uint8_t target_system, uint8_t target_component)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_serial_control_pack(
        _msg, sysid, compid,
        device, flags, timeout, baudrate, count, data, target_system, target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_serial_control_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_serial_control_t* _payload)
{
    return mavlink_msg_serial_control_pack(
        sysid,
        compid,
        _msg,
        _payload->device, _payload->flags, _payload->timeout, _payload->baudrate, _payload->count, _payload->data, _payload->target_system, _payload->target_component);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_serial_control_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t* data, uint8_t target_system, uint8_t target_component)
{
    return fmav_msg_serial_control_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        device, flags, timeout, baudrate, count, data, target_system, target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_serial_control_decode(const mavlink_message_t* msg, mavlink_serial_control_t* payload)
{
    fmav_msg_serial_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SERIAL_CONTROL_H
