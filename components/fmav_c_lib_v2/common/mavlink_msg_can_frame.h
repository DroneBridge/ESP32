//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAN_FRAME_H
#define FASTMAVLINK_MSG_CAN_FRAME_H


//----------------------------------------
//-- Message CAN_FRAME
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_can_frame_t {
    uint32_t id;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t bus;
    uint8_t len;
    uint8_t data[8];
}) fmav_can_frame_t;


#define FASTMAVLINK_MSG_ID_CAN_FRAME  386

#define FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_CAN_FRAME_CRCEXTRA  132

#define FASTMAVLINK_MSG_CAN_FRAME_FLAGS  3
#define FASTMAVLINK_MSG_CAN_FRAME_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_CAN_FRAME_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_CAN_FRAME_FRAME_LEN_MAX  41

#define FASTMAVLINK_MSG_CAN_FRAME_FIELD_DATA_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_CAN_FRAME_FIELD_DATA_LEN  8 // length of array = number of bytes

#define FASTMAVLINK_MSG_CAN_FRAME_FIELD_ID_OFS  0
#define FASTMAVLINK_MSG_CAN_FRAME_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_CAN_FRAME_FIELD_TARGET_COMPONENT_OFS  5
#define FASTMAVLINK_MSG_CAN_FRAME_FIELD_BUS_OFS  6
#define FASTMAVLINK_MSG_CAN_FRAME_FIELD_LEN_OFS  7
#define FASTMAVLINK_MSG_CAN_FRAME_FIELD_DATA_OFS  8


//----------------------------------------
//-- Message CAN_FRAME pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_frame_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_can_frame_t* _payload = (fmav_can_frame_t*)_msg->payload;

    _payload->id = id;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->bus = bus;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*8);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CAN_FRAME;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_CAN_FRAME_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_frame_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_can_frame_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_can_frame_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->bus, _payload->len, _payload->id, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_frame_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_can_frame_t* _payload = (fmav_can_frame_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->id = id;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->bus = bus;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*8);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAN_FRAME;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAN_FRAME >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAN_FRAME >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAN_FRAME_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_frame_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_can_frame_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_can_frame_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->bus, _payload->len, _payload->id, _payload->data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_frame_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_can_frame_t _payload;

    _payload.id = id;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.bus = bus;
    _payload.len = len;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*8);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAN_FRAME,
        FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAN_FRAME_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_frame_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_can_frame_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAN_FRAME,
        FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAN_FRAME_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAN_FRAME decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_can_frame_decode(fmav_can_frame_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAN_FRAME_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_can_frame_get_field_id(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_frame_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_frame_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_frame_get_field_bus(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_frame_get_field_len(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_can_frame_get_field_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_frame_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CAN_FRAME_FIELD_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[8]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAN_FRAME  386

#define mavlink_can_frame_t  fmav_can_frame_t

#define MAVLINK_MSG_ID_CAN_FRAME_LEN  16
#define MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN  16
#define MAVLINK_MSG_ID_386_LEN  16
#define MAVLINK_MSG_ID_386_MIN_LEN  16

#define MAVLINK_MSG_ID_CAN_FRAME_CRC  132
#define MAVLINK_MSG_ID_386_CRC  132

#define MAVLINK_MSG_CAN_FRAME_FIELD_DATA_LEN 8


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_can_frame_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_can_frame_pack(
        _msg, sysid, compid,
        target_system, target_component, bus, len, id, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_can_frame_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_can_frame_t* _payload)
{
    return mavlink_msg_can_frame_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->bus, _payload->len, _payload->id, _payload->data);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_can_frame_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t* data)
{
    return fmav_msg_can_frame_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, bus, len, id, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_can_frame_decode(const mavlink_message_t* msg, mavlink_can_frame_t* payload)
{
    fmav_msg_can_frame_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAN_FRAME_H
