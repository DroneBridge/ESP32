//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_H
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_H


//----------------------------------------
//-- Message HIL_ACTUATOR_CONTROLS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_actuator_controls_t {
    uint64_t time_usec;
    uint64_t flags;
    float controls[16];
    uint8_t mode;
}) fmav_hil_actuator_controls_t;


#define FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS  93

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX  81
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA  47

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FLAGS  0
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FRAME_LEN_MAX  106

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_LEN  64 // length of array = number of bytes

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_FLAGS_OFS  8
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_OFS  16
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_MODE_OFS  80


//----------------------------------------
//-- Message HIL_ACTUATOR_CONTROLS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags,
    fmav_status_t* _status)
{
    fmav_hil_actuator_controls_t* _payload = (fmav_hil_actuator_controls_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->flags = flags;
    _payload->mode = mode;
    memcpy(&(_payload->controls), controls, sizeof(float)*16);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_actuator_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_actuator_controls_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->controls, _payload->mode, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags,
    fmav_status_t* _status)
{
    fmav_hil_actuator_controls_t* _payload = (fmav_hil_actuator_controls_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->flags = flags;
    _payload->mode = mode;
    memcpy(&(_payload->controls), controls, sizeof(float)*16);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_actuator_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_actuator_controls_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->controls, _payload->mode, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags,
    fmav_status_t* _status)
{
    fmav_hil_actuator_controls_t _payload;

    _payload.time_usec = time_usec;
    _payload.flags = flags;
    _payload.mode = mode;
    memcpy(&(_payload.controls), controls, sizeof(float)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_actuator_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIL_ACTUATOR_CONTROLS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_actuator_controls_decode(fmav_hil_actuator_controls_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_actuator_controls_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_actuator_controls_get_field_flags(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_actuator_controls_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[80]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_hil_actuator_controls_get_field_controls_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[16]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_actuator_controls_get_field_controls(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_NUM) return 0;
    return ((float*)&(msg->payload[16]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS  93

#define mavlink_hil_actuator_controls_t  fmav_hil_actuator_controls_t

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN  81
#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN  81
#define MAVLINK_MSG_ID_93_LEN  81
#define MAVLINK_MSG_ID_93_MIN_LEN  81

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC  47
#define MAVLINK_MSG_ID_93_CRC  47

#define MAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_actuator_controls_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_actuator_controls_pack(
        _msg, sysid, compid,
        time_usec, controls, mode, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_actuator_controls_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_hil_actuator_controls_t* _payload)
{
    return mavlink_msg_hil_actuator_controls_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->controls, _payload->mode, _payload->flags);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_actuator_controls_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags)
{
    return fmav_msg_hil_actuator_controls_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, controls, mode, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_actuator_controls_decode(const mavlink_message_t* msg, mavlink_hil_actuator_controls_t* payload)
{
    fmav_msg_hil_actuator_controls_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_H
