//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MISSION_ITEM_H
#define FASTMAVLINK_MSG_MISSION_ITEM_H


//----------------------------------------
//-- Message MISSION_ITEM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mission_item_t {
    float param1;
    float param2;
    float param3;
    float param4;
    float x;
    float y;
    float z;
    uint16_t seq;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t frame;
    uint8_t current;
    uint8_t autocontinue;
    uint8_t mission_type;
}) fmav_mission_item_t;


#define FASTMAVLINK_MSG_ID_MISSION_ITEM  39

#define FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX  38
#define FASTMAVLINK_MSG_MISSION_ITEM_CRCEXTRA  254

#define FASTMAVLINK_MSG_MISSION_ITEM_FLAGS  3
#define FASTMAVLINK_MSG_MISSION_ITEM_TARGET_SYSTEM_OFS  32
#define FASTMAVLINK_MSG_MISSION_ITEM_TARGET_COMPONENT_OFS  33

#define FASTMAVLINK_MSG_MISSION_ITEM_FRAME_LEN_MAX  63



#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_PARAM1_OFS  0
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_PARAM2_OFS  4
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_PARAM3_OFS  8
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_PARAM4_OFS  12
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_X_OFS  16
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_Y_OFS  20
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_Z_OFS  24
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_SEQ_OFS  28
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_COMMAND_OFS  30
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_TARGET_SYSTEM_OFS  32
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_TARGET_COMPONENT_OFS  33
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_FRAME_OFS  34
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_CURRENT_OFS  35
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_AUTOCONTINUE_OFS  36
#define FASTMAVLINK_MSG_MISSION_ITEM_FIELD_MISSION_TYPE_OFS  37


//----------------------------------------
//-- Message MISSION_ITEM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_item_t* _payload = (fmav_mission_item_t*)_msg->payload;

    _payload->param1 = param1;
    _payload->param2 = param2;
    _payload->param3 = param3;
    _payload->param4 = param4;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->seq = seq;
    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->frame = frame;
    _payload->current = current;
    _payload->autocontinue = autocontinue;
    _payload->mission_type = mission_type;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MISSION_ITEM;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_MISSION_ITEM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_item_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_item_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->seq, _payload->frame, _payload->command, _payload->current, _payload->autocontinue, _payload->param1, _payload->param2, _payload->param3, _payload->param4, _payload->x, _payload->y, _payload->z, _payload->mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_item_t* _payload = (fmav_mission_item_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param1 = param1;
    _payload->param2 = param2;
    _payload->param3 = param3;
    _payload->param4 = param4;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->seq = seq;
    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->frame = frame;
    _payload->current = current;
    _payload->autocontinue = autocontinue;
    _payload->mission_type = mission_type;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_ITEM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_ITEM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_ITEM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_ITEM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_item_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_item_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->seq, _payload->frame, _payload->command, _payload->current, _payload->autocontinue, _payload->param1, _payload->param2, _payload->param3, _payload->param4, _payload->x, _payload->y, _payload->z, _payload->mission_type,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_item_t _payload;

    _payload.param1 = param1;
    _payload.param2 = param2;
    _payload.param3 = param3;
    _payload.param4 = param4;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.seq = seq;
    _payload.command = command;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.frame = frame;
    _payload.current = current;
    _payload.autocontinue = autocontinue;
    _payload.mission_type = mission_type;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MISSION_ITEM,
        FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_ITEM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_item_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MISSION_ITEM,
        FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_ITEM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MISSION_ITEM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_item_decode(fmav_mission_item_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mission_item_get_field_param1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mission_item_get_field_param2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mission_item_get_field_param3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mission_item_get_field_param4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mission_item_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mission_item_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mission_item_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_get_field_seq(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_get_field_command(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_item_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_item_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[33]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_item_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_item_get_field_current(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_item_get_field_autocontinue(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_item_get_field_mission_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MISSION_ITEM  39

#define mavlink_mission_item_t  fmav_mission_item_t

#define MAVLINK_MSG_ID_MISSION_ITEM_LEN  38
#define MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN  37
#define MAVLINK_MSG_ID_39_LEN  38
#define MAVLINK_MSG_ID_39_MIN_LEN  37

#define MAVLINK_MSG_ID_MISSION_ITEM_CRC  254
#define MAVLINK_MSG_ID_39_CRC  254




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_item_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_item_pack(
        _msg, sysid, compid,
        target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_item_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_mission_item_t* _payload)
{
    return mavlink_msg_mission_item_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->seq, _payload->frame, _payload->command, _payload->current, _payload->autocontinue, _payload->param1, _payload->param2, _payload->param3, _payload->param4, _payload->x, _payload->y, _payload->z, _payload->mission_type);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_item_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type)
{
    return fmav_msg_mission_item_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mission_item_decode(const mavlink_message_t* msg, mavlink_mission_item_t* payload)
{
    fmav_msg_mission_item_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MISSION_ITEM_H
