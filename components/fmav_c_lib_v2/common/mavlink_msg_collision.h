//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COLLISION_H
#define FASTMAVLINK_MSG_COLLISION_H


//----------------------------------------
//-- Message COLLISION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_collision_t {
    uint32_t id;
    float time_to_minimum_delta;
    float altitude_minimum_delta;
    float horizontal_minimum_delta;
    uint8_t src;
    uint8_t action;
    uint8_t threat_level;
}) fmav_collision_t;


#define FASTMAVLINK_MSG_ID_COLLISION  247

#define FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX  19
#define FASTMAVLINK_MSG_COLLISION_CRCEXTRA  81

#define FASTMAVLINK_MSG_COLLISION_FLAGS  0
#define FASTMAVLINK_MSG_COLLISION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_COLLISION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_COLLISION_FRAME_LEN_MAX  44



#define FASTMAVLINK_MSG_COLLISION_FIELD_ID_OFS  0
#define FASTMAVLINK_MSG_COLLISION_FIELD_TIME_TO_MINIMUM_DELTA_OFS  4
#define FASTMAVLINK_MSG_COLLISION_FIELD_ALTITUDE_MINIMUM_DELTA_OFS  8
#define FASTMAVLINK_MSG_COLLISION_FIELD_HORIZONTAL_MINIMUM_DELTA_OFS  12
#define FASTMAVLINK_MSG_COLLISION_FIELD_SRC_OFS  16
#define FASTMAVLINK_MSG_COLLISION_FIELD_ACTION_OFS  17
#define FASTMAVLINK_MSG_COLLISION_FIELD_THREAT_LEVEL_OFS  18


//----------------------------------------
//-- Message COLLISION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta,
    fmav_status_t* _status)
{
    fmav_collision_t* _payload = (fmav_collision_t*)_msg->payload;

    _payload->id = id;
    _payload->time_to_minimum_delta = time_to_minimum_delta;
    _payload->altitude_minimum_delta = altitude_minimum_delta;
    _payload->horizontal_minimum_delta = horizontal_minimum_delta;
    _payload->src = src;
    _payload->action = action;
    _payload->threat_level = threat_level;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_COLLISION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_COLLISION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_collision_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_collision_pack(
        _msg, sysid, compid,
        _payload->src, _payload->id, _payload->action, _payload->threat_level, _payload->time_to_minimum_delta, _payload->altitude_minimum_delta, _payload->horizontal_minimum_delta,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta,
    fmav_status_t* _status)
{
    fmav_collision_t* _payload = (fmav_collision_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->id = id;
    _payload->time_to_minimum_delta = time_to_minimum_delta;
    _payload->altitude_minimum_delta = altitude_minimum_delta;
    _payload->horizontal_minimum_delta = horizontal_minimum_delta;
    _payload->src = src;
    _payload->action = action;
    _payload->threat_level = threat_level;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COLLISION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COLLISION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COLLISION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COLLISION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_collision_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_collision_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->src, _payload->id, _payload->action, _payload->threat_level, _payload->time_to_minimum_delta, _payload->altitude_minimum_delta, _payload->horizontal_minimum_delta,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta,
    fmav_status_t* _status)
{
    fmav_collision_t _payload;

    _payload.id = id;
    _payload.time_to_minimum_delta = time_to_minimum_delta;
    _payload.altitude_minimum_delta = altitude_minimum_delta;
    _payload.horizontal_minimum_delta = horizontal_minimum_delta;
    _payload.src = src;
    _payload.action = action;
    _payload.threat_level = threat_level;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_COLLISION,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COLLISION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_collision_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_COLLISION,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COLLISION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message COLLISION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_collision_decode(fmav_collision_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_collision_get_field_id(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_collision_get_field_time_to_minimum_delta(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_collision_get_field_altitude_minimum_delta(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_collision_get_field_horizontal_minimum_delta(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_collision_get_field_src(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_collision_get_field_action(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[17]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_collision_get_field_threat_level(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COLLISION  247

#define mavlink_collision_t  fmav_collision_t

#define MAVLINK_MSG_ID_COLLISION_LEN  19
#define MAVLINK_MSG_ID_COLLISION_MIN_LEN  19
#define MAVLINK_MSG_ID_247_LEN  19
#define MAVLINK_MSG_ID_247_MIN_LEN  19

#define MAVLINK_MSG_ID_COLLISION_CRC  81
#define MAVLINK_MSG_ID_247_CRC  81




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_collision_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_collision_pack(
        _msg, sysid, compid,
        src, id, action, threat_level, time_to_minimum_delta, altitude_minimum_delta, horizontal_minimum_delta,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_collision_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_collision_t* _payload)
{
    return mavlink_msg_collision_pack(
        sysid,
        compid,
        _msg,
        _payload->src, _payload->id, _payload->action, _payload->threat_level, _payload->time_to_minimum_delta, _payload->altitude_minimum_delta, _payload->horizontal_minimum_delta);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_collision_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta)
{
    return fmav_msg_collision_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        src, id, action, threat_level, time_to_minimum_delta, altitude_minimum_delta, horizontal_minimum_delta,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_collision_decode(const mavlink_message_t* msg, mavlink_collision_t* payload)
{
    fmav_msg_collision_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COLLISION_H
