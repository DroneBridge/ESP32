//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MEMORY_VECT_H
#define FASTMAVLINK_MSG_MEMORY_VECT_H


//----------------------------------------
//-- Message MEMORY_VECT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_memory_vect_t {
    uint16_t address;
    uint8_t ver;
    uint8_t type;
    int8_t value[32];
}) fmav_memory_vect_t;


#define FASTMAVLINK_MSG_ID_MEMORY_VECT  249

#define FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX  36
#define FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA  204

#define FASTMAVLINK_MSG_MEMORY_VECT_FLAGS  0
#define FASTMAVLINK_MSG_MEMORY_VECT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MEMORY_VECT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MEMORY_VECT_FRAME_LEN_MAX  61

#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_ADDRESS_OFS  0
#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VER_OFS  2
#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_TYPE_OFS  3
#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_OFS  4


//----------------------------------------
//-- Message MEMORY_VECT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value,
    fmav_status_t* _status)
{
    fmav_memory_vect_t* _payload = (fmav_memory_vect_t*)_msg->payload;

    _payload->address = address;
    _payload->ver = ver;
    _payload->type = type;
    memcpy(&(_payload->value), value, sizeof(int8_t)*32);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MEMORY_VECT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_memory_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_memory_vect_pack(
        _msg, sysid, compid,
        _payload->address, _payload->ver, _payload->type, _payload->value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value,
    fmav_status_t* _status)
{
    fmav_memory_vect_t* _payload = (fmav_memory_vect_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->address = address;
    _payload->ver = ver;
    _payload->type = type;
    memcpy(&(_payload->value), value, sizeof(int8_t)*32);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MEMORY_VECT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MEMORY_VECT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MEMORY_VECT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_memory_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_memory_vect_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->address, _payload->ver, _payload->type, _payload->value,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value,
    fmav_status_t* _status)
{
    fmav_memory_vect_t _payload;

    _payload.address = address;
    _payload.ver = ver;
    _payload.type = type;
    memcpy(&(_payload.value), value, sizeof(int8_t)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MEMORY_VECT,
        FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_memory_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MEMORY_VECT,
        FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MEMORY_VECT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_memory_vect_decode(fmav_memory_vect_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_get_field_address(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_memory_vect_get_field_ver(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_memory_vect_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t* fmav_msg_memory_vect_get_field_value_ptr(const fmav_message_t* msg)
{
    return (int8_t*)&(msg->payload[4]);
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_memory_vect_get_field_value(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_NUM) return 0;
    return ((int8_t*)&(msg->payload[4]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MEMORY_VECT  249

#define mavlink_memory_vect_t  fmav_memory_vect_t

#define MAVLINK_MSG_ID_MEMORY_VECT_LEN  36
#define MAVLINK_MSG_ID_MEMORY_VECT_MIN_LEN  36
#define MAVLINK_MSG_ID_249_LEN  36
#define MAVLINK_MSG_ID_249_MIN_LEN  36

#define MAVLINK_MSG_ID_MEMORY_VECT_CRC  204
#define MAVLINK_MSG_ID_249_CRC  204

#define MAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_memory_vect_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_memory_vect_pack(
        _msg, sysid, compid,
        address, ver, type, value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_memory_vect_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_memory_vect_t* _payload)
{
    return mavlink_msg_memory_vect_pack(
        sysid,
        compid,
        _msg,
        _payload->address, _payload->ver, _payload->type, _payload->value);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_memory_vect_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value)
{
    return fmav_msg_memory_vect_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        address, ver, type, value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_memory_vect_decode(const mavlink_message_t* msg, mavlink_memory_vect_t* payload)
{
    fmav_msg_memory_vect_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MEMORY_VECT_H
