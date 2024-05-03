//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HEARTBEAT_H
#define FASTMAVLINK_MSG_HEARTBEAT_H


//----------------------------------------
//-- Message HEARTBEAT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_heartbeat_t {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
}) fmav_heartbeat_t;


#define FASTMAVLINK_MSG_ID_HEARTBEAT  0

#define FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX  9
#define FASTMAVLINK_MSG_HEARTBEAT_CRCEXTRA  50

#define FASTMAVLINK_MSG_HEARTBEAT_FLAGS  0
#define FASTMAVLINK_MSG_HEARTBEAT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HEARTBEAT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HEARTBEAT_FRAME_LEN_MAX  34



#define FASTMAVLINK_MSG_HEARTBEAT_FIELD_CUSTOM_MODE_OFS  0
#define FASTMAVLINK_MSG_HEARTBEAT_FIELD_TYPE_OFS  4
#define FASTMAVLINK_MSG_HEARTBEAT_FIELD_AUTOPILOT_OFS  5
#define FASTMAVLINK_MSG_HEARTBEAT_FIELD_BASE_MODE_OFS  6
#define FASTMAVLINK_MSG_HEARTBEAT_FIELD_SYSTEM_STATUS_OFS  7
#define FASTMAVLINK_MSG_HEARTBEAT_FIELD_MAVLINK_VERSION_OFS  8


//----------------------------------------
//-- Message HEARTBEAT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_heartbeat_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status,
    fmav_status_t* _status)
{
    fmav_heartbeat_t* _payload = (fmav_heartbeat_t*)_msg->payload;

    _payload->custom_mode = custom_mode;
    _payload->type = type;
    _payload->autopilot = autopilot;
    _payload->base_mode = base_mode;
    _payload->system_status = system_status;
    _payload->mavlink_version = FASTMAVLINK_MAVLINK_VERSION;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_HEARTBEAT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_HEARTBEAT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_heartbeat_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_heartbeat_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_heartbeat_pack(
        _msg, sysid, compid,
        _payload->type, _payload->autopilot, _payload->base_mode, _payload->custom_mode, _payload->system_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_heartbeat_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status,
    fmav_status_t* _status)
{
    fmav_heartbeat_t* _payload = (fmav_heartbeat_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->custom_mode = custom_mode;
    _payload->type = type;
    _payload->autopilot = autopilot;
    _payload->base_mode = base_mode;
    _payload->system_status = system_status;
    _payload->mavlink_version = FASTMAVLINK_MAVLINK_VERSION;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HEARTBEAT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HEARTBEAT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HEARTBEAT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HEARTBEAT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_heartbeat_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_heartbeat_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_heartbeat_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->type, _payload->autopilot, _payload->base_mode, _payload->custom_mode, _payload->system_status,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_heartbeat_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status,
    fmav_status_t* _status)
{
    fmav_heartbeat_t _payload;

    _payload.custom_mode = custom_mode;
    _payload.type = type;
    _payload.autopilot = autopilot;
    _payload.base_mode = base_mode;
    _payload.system_status = system_status;
    _payload.mavlink_version = FASTMAVLINK_MAVLINK_VERSION;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HEARTBEAT,
        FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HEARTBEAT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_heartbeat_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_heartbeat_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HEARTBEAT,
        FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HEARTBEAT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HEARTBEAT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_heartbeat_decode(fmav_heartbeat_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_HEARTBEAT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_heartbeat_get_field_custom_mode(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_heartbeat_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_heartbeat_get_field_autopilot(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_heartbeat_get_field_base_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_heartbeat_get_field_system_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_heartbeat_get_field_mavlink_version(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HEARTBEAT  0

#define mavlink_heartbeat_t  fmav_heartbeat_t

#define MAVLINK_MSG_ID_HEARTBEAT_LEN  9
#define MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN  9
#define MAVLINK_MSG_ID_0_LEN  9
#define MAVLINK_MSG_ID_0_MIN_LEN  9

#define MAVLINK_MSG_ID_HEARTBEAT_CRC  50
#define MAVLINK_MSG_ID_0_CRC  50




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_heartbeat_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_heartbeat_pack(
        _msg, sysid, compid,
        type, autopilot, base_mode, custom_mode, system_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_heartbeat_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_heartbeat_t* _payload)
{
    return mavlink_msg_heartbeat_pack(
        sysid,
        compid,
        _msg,
        _payload->type, _payload->autopilot, _payload->base_mode, _payload->custom_mode, _payload->system_status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_heartbeat_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
{
    return fmav_msg_heartbeat_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        type, autopilot, base_mode, custom_mode, system_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* payload)
{
    fmav_msg_heartbeat_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HEARTBEAT_H
