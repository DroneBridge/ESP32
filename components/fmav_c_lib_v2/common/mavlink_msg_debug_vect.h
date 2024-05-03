//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DEBUG_VECT_H
#define FASTMAVLINK_MSG_DEBUG_VECT_H


//----------------------------------------
//-- Message DEBUG_VECT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_debug_vect_t {
    uint64_t time_usec;
    float x;
    float y;
    float z;
    char name[10];
}) fmav_debug_vect_t;


#define FASTMAVLINK_MSG_ID_DEBUG_VECT  250

#define FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX  30
#define FASTMAVLINK_MSG_DEBUG_VECT_CRCEXTRA  49

#define FASTMAVLINK_MSG_DEBUG_VECT_FLAGS  0
#define FASTMAVLINK_MSG_DEBUG_VECT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DEBUG_VECT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DEBUG_VECT_FRAME_LEN_MAX  55

#define FASTMAVLINK_MSG_DEBUG_VECT_FIELD_NAME_NUM  10 // number of elements in array
#define FASTMAVLINK_MSG_DEBUG_VECT_FIELD_NAME_LEN  10 // length of array = number of bytes

#define FASTMAVLINK_MSG_DEBUG_VECT_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_DEBUG_VECT_FIELD_X_OFS  8
#define FASTMAVLINK_MSG_DEBUG_VECT_FIELD_Y_OFS  12
#define FASTMAVLINK_MSG_DEBUG_VECT_FIELD_Z_OFS  16
#define FASTMAVLINK_MSG_DEBUG_VECT_FIELD_NAME_OFS  20


//----------------------------------------
//-- Message DEBUG_VECT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const char* name, uint64_t time_usec, float x, float y, float z,
    fmav_status_t* _status)
{
    fmav_debug_vect_t* _payload = (fmav_debug_vect_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    memcpy(&(_payload->name), name, sizeof(char)*10);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_DEBUG_VECT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_DEBUG_VECT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_debug_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_debug_vect_pack(
        _msg, sysid, compid,
        _payload->name, _payload->time_usec, _payload->x, _payload->y, _payload->z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const char* name, uint64_t time_usec, float x, float y, float z,
    fmav_status_t* _status)
{
    fmav_debug_vect_t* _payload = (fmav_debug_vect_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    memcpy(&(_payload->name), name, sizeof(char)*10);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DEBUG_VECT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DEBUG_VECT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DEBUG_VECT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEBUG_VECT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_debug_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_debug_vect_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->name, _payload->time_usec, _payload->x, _payload->y, _payload->z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const char* name, uint64_t time_usec, float x, float y, float z,
    fmav_status_t* _status)
{
    fmav_debug_vect_t _payload;

    _payload.time_usec = time_usec;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    memcpy(&(_payload.name), name, sizeof(char)*10);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DEBUG_VECT,
        FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEBUG_VECT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_debug_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DEBUG_VECT,
        FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEBUG_VECT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DEBUG_VECT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_debug_vect_decode(fmav_debug_vect_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_debug_vect_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_debug_vect_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_debug_vect_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_debug_vect_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_debug_vect_get_field_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_debug_vect_get_field_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_DEBUG_VECT_FIELD_NAME_NUM) return 0;
    return ((char*)&(msg->payload[20]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DEBUG_VECT  250

#define mavlink_debug_vect_t  fmav_debug_vect_t

#define MAVLINK_MSG_ID_DEBUG_VECT_LEN  30
#define MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN  30
#define MAVLINK_MSG_ID_250_LEN  30
#define MAVLINK_MSG_ID_250_MIN_LEN  30

#define MAVLINK_MSG_ID_DEBUG_VECT_CRC  49
#define MAVLINK_MSG_ID_250_CRC  49

#define MAVLINK_MSG_DEBUG_VECT_FIELD_NAME_LEN 10


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_debug_vect_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const char* name, uint64_t time_usec, float x, float y, float z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_debug_vect_pack(
        _msg, sysid, compid,
        name, time_usec, x, y, z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_debug_vect_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_debug_vect_t* _payload)
{
    return mavlink_msg_debug_vect_pack(
        sysid,
        compid,
        _msg,
        _payload->name, _payload->time_usec, _payload->x, _payload->y, _payload->z);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_debug_vect_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* name, uint64_t time_usec, float x, float y, float z)
{
    return fmav_msg_debug_vect_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        name, time_usec, x, y, z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_debug_vect_decode(const mavlink_message_t* msg, mavlink_debug_vect_t* payload)
{
    fmav_msg_debug_vect_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DEBUG_VECT_H
