//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAN_FILTER_MODIFY_H
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_H


//----------------------------------------
//-- Message CAN_FILTER_MODIFY
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_can_filter_modify_t {
    uint16_t ids[16];
    uint8_t target_system;
    uint8_t target_component;
    uint8_t bus;
    uint8_t operation;
    uint8_t num_ids;
}) fmav_can_filter_modify_t;


#define FASTMAVLINK_MSG_ID_CAN_FILTER_MODIFY  388

#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX  37
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_CRCEXTRA  8

#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FLAGS  3
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_TARGET_SYSTEM_OFS  32
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_TARGET_COMPONENT_OFS  33

#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FRAME_LEN_MAX  62

#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_IDS_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_IDS_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_IDS_OFS  0
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_TARGET_SYSTEM_OFS  32
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_TARGET_COMPONENT_OFS  33
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_BUS_OFS  34
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_OPERATION_OFS  35
#define FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_NUM_IDS_OFS  36


//----------------------------------------
//-- Message CAN_FILTER_MODIFY pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_filter_modify_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t operation, uint8_t num_ids, const uint16_t* ids,
    fmav_status_t* _status)
{
    fmav_can_filter_modify_t* _payload = (fmav_can_filter_modify_t*)_msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->bus = bus;
    _payload->operation = operation;
    _payload->num_ids = num_ids;
    memcpy(&(_payload->ids), ids, sizeof(uint16_t)*16);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CAN_FILTER_MODIFY;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_CAN_FILTER_MODIFY_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_filter_modify_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_can_filter_modify_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_can_filter_modify_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->bus, _payload->operation, _payload->num_ids, _payload->ids,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_filter_modify_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t operation, uint8_t num_ids, const uint16_t* ids,
    fmav_status_t* _status)
{
    fmav_can_filter_modify_t* _payload = (fmav_can_filter_modify_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->bus = bus;
    _payload->operation = operation;
    _payload->num_ids = num_ids;
    memcpy(&(_payload->ids), ids, sizeof(uint16_t)*16);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAN_FILTER_MODIFY;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAN_FILTER_MODIFY >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAN_FILTER_MODIFY >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAN_FILTER_MODIFY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_filter_modify_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_can_filter_modify_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_can_filter_modify_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->bus, _payload->operation, _payload->num_ids, _payload->ids,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_filter_modify_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t operation, uint8_t num_ids, const uint16_t* ids,
    fmav_status_t* _status)
{
    fmav_can_filter_modify_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.bus = bus;
    _payload.operation = operation;
    _payload.num_ids = num_ids;
    memcpy(&(_payload.ids), ids, sizeof(uint16_t)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAN_FILTER_MODIFY,
        FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAN_FILTER_MODIFY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_filter_modify_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_can_filter_modify_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAN_FILTER_MODIFY,
        FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAN_FILTER_MODIFY_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAN_FILTER_MODIFY decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_can_filter_modify_decode(fmav_can_filter_modify_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAN_FILTER_MODIFY_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_filter_modify_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_filter_modify_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[33]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_filter_modify_get_field_bus(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_filter_modify_get_field_operation(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_can_filter_modify_get_field_num_ids(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_can_filter_modify_get_field_ids_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_can_filter_modify_get_field_ids(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_IDS_NUM) return 0;
    return ((uint16_t*)&(msg->payload[0]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAN_FILTER_MODIFY  388

#define mavlink_can_filter_modify_t  fmav_can_filter_modify_t

#define MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN  37
#define MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN  37
#define MAVLINK_MSG_ID_388_LEN  37
#define MAVLINK_MSG_ID_388_MIN_LEN  37

#define MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC  8
#define MAVLINK_MSG_ID_388_CRC  8

#define MAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_IDS_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_can_filter_modify_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t operation, uint8_t num_ids, const uint16_t* ids)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_can_filter_modify_pack(
        _msg, sysid, compid,
        target_system, target_component, bus, operation, num_ids, ids,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_can_filter_modify_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_can_filter_modify_t* _payload)
{
    return mavlink_msg_can_filter_modify_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->bus, _payload->operation, _payload->num_ids, _payload->ids);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_can_filter_modify_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t operation, uint8_t num_ids, const uint16_t* ids)
{
    return fmav_msg_can_filter_modify_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, bus, operation, num_ids, ids,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_can_filter_modify_decode(const mavlink_message_t* msg, mavlink_can_filter_modify_t* payload)
{
    fmav_msg_can_filter_modify_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAN_FILTER_MODIFY_H
