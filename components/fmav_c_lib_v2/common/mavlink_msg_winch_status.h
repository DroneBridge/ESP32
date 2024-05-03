//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WINCH_STATUS_H
#define FASTMAVLINK_MSG_WINCH_STATUS_H


//----------------------------------------
//-- Message WINCH_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_winch_status_t {
    uint64_t time_usec;
    float line_length;
    float speed;
    float tension;
    float voltage;
    float current;
    uint32_t status;
    int16_t temperature;
}) fmav_winch_status_t;


#define FASTMAVLINK_MSG_ID_WINCH_STATUS  9005

#define FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX  34
#define FASTMAVLINK_MSG_WINCH_STATUS_CRCEXTRA  117

#define FASTMAVLINK_MSG_WINCH_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_WINCH_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WINCH_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_WINCH_STATUS_FRAME_LEN_MAX  59



#define FASTMAVLINK_MSG_WINCH_STATUS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_WINCH_STATUS_FIELD_LINE_LENGTH_OFS  8
#define FASTMAVLINK_MSG_WINCH_STATUS_FIELD_SPEED_OFS  12
#define FASTMAVLINK_MSG_WINCH_STATUS_FIELD_TENSION_OFS  16
#define FASTMAVLINK_MSG_WINCH_STATUS_FIELD_VOLTAGE_OFS  20
#define FASTMAVLINK_MSG_WINCH_STATUS_FIELD_CURRENT_OFS  24
#define FASTMAVLINK_MSG_WINCH_STATUS_FIELD_STATUS_OFS  28
#define FASTMAVLINK_MSG_WINCH_STATUS_FIELD_TEMPERATURE_OFS  32


//----------------------------------------
//-- Message WINCH_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status,
    fmav_status_t* _status)
{
    fmav_winch_status_t* _payload = (fmav_winch_status_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->line_length = line_length;
    _payload->speed = speed;
    _payload->tension = tension;
    _payload->voltage = voltage;
    _payload->current = current;
    _payload->status = status;
    _payload->temperature = temperature;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_WINCH_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_WINCH_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_winch_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_winch_status_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->line_length, _payload->speed, _payload->tension, _payload->voltage, _payload->current, _payload->temperature, _payload->status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status,
    fmav_status_t* _status)
{
    fmav_winch_status_t* _payload = (fmav_winch_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->line_length = line_length;
    _payload->speed = speed;
    _payload->tension = tension;
    _payload->voltage = voltage;
    _payload->current = current;
    _payload->status = status;
    _payload->temperature = temperature;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WINCH_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WINCH_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WINCH_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WINCH_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_winch_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_winch_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->line_length, _payload->speed, _payload->tension, _payload->voltage, _payload->current, _payload->temperature, _payload->status,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status,
    fmav_status_t* _status)
{
    fmav_winch_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.line_length = line_length;
    _payload.speed = speed;
    _payload.tension = tension;
    _payload.voltage = voltage;
    _payload.current = current;
    _payload.status = status;
    _payload.temperature = temperature;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_WINCH_STATUS,
        FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WINCH_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_winch_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_WINCH_STATUS,
        FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WINCH_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message WINCH_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_winch_status_decode(fmav_winch_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_winch_status_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_winch_status_get_field_line_length(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_winch_status_get_field_speed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_winch_status_get_field_tension(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_winch_status_get_field_voltage(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_winch_status_get_field_current(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_winch_status_get_field_status(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_winch_status_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WINCH_STATUS  9005

#define mavlink_winch_status_t  fmav_winch_status_t

#define MAVLINK_MSG_ID_WINCH_STATUS_LEN  34
#define MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN  34
#define MAVLINK_MSG_ID_9005_LEN  34
#define MAVLINK_MSG_ID_9005_MIN_LEN  34

#define MAVLINK_MSG_ID_WINCH_STATUS_CRC  117
#define MAVLINK_MSG_ID_9005_CRC  117




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_winch_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_winch_status_pack(
        _msg, sysid, compid,
        time_usec, line_length, speed, tension, voltage, current, temperature, status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_winch_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_winch_status_t* _payload)
{
    return mavlink_msg_winch_status_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->line_length, _payload->speed, _payload->tension, _payload->voltage, _payload->current, _payload->temperature, _payload->status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_winch_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status)
{
    return fmav_msg_winch_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, line_length, speed, tension, voltage, current, temperature, status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_winch_status_decode(const mavlink_message_t* msg, mavlink_winch_status_t* payload)
{
    fmav_msg_winch_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WINCH_STATUS_H
