//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HYGROMETER_SENSOR_H
#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_H


//----------------------------------------
//-- Message HYGROMETER_SENSOR
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hygrometer_sensor_t {
    int16_t temperature;
    uint16_t humidity;
    uint8_t id;
}) fmav_hygrometer_sensor_t;


#define FASTMAVLINK_MSG_ID_HYGROMETER_SENSOR  12920

#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX  5
#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_CRCEXTRA  20

#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_FLAGS  0
#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_FRAME_LEN_MAX  30



#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_FIELD_TEMPERATURE_OFS  0
#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_FIELD_HUMIDITY_OFS  2
#define FASTMAVLINK_MSG_HYGROMETER_SENSOR_FIELD_ID_OFS  4


//----------------------------------------
//-- Message HYGROMETER_SENSOR pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hygrometer_sensor_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t temperature, uint16_t humidity,
    fmav_status_t* _status)
{
    fmav_hygrometer_sensor_t* _payload = (fmav_hygrometer_sensor_t*)_msg->payload;

    _payload->temperature = temperature;
    _payload->humidity = humidity;
    _payload->id = id;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_HYGROMETER_SENSOR;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_HYGROMETER_SENSOR_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hygrometer_sensor_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hygrometer_sensor_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hygrometer_sensor_pack(
        _msg, sysid, compid,
        _payload->id, _payload->temperature, _payload->humidity,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hygrometer_sensor_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t temperature, uint16_t humidity,
    fmav_status_t* _status)
{
    fmav_hygrometer_sensor_t* _payload = (fmav_hygrometer_sensor_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->temperature = temperature;
    _payload->humidity = humidity;
    _payload->id = id;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HYGROMETER_SENSOR;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HYGROMETER_SENSOR >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HYGROMETER_SENSOR >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HYGROMETER_SENSOR_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hygrometer_sensor_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hygrometer_sensor_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hygrometer_sensor_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->temperature, _payload->humidity,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hygrometer_sensor_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t temperature, uint16_t humidity,
    fmav_status_t* _status)
{
    fmav_hygrometer_sensor_t _payload;

    _payload.temperature = temperature;
    _payload.humidity = humidity;
    _payload.id = id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HYGROMETER_SENSOR,
        FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HYGROMETER_SENSOR_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hygrometer_sensor_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hygrometer_sensor_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HYGROMETER_SENSOR,
        FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HYGROMETER_SENSOR_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HYGROMETER_SENSOR decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hygrometer_sensor_decode(fmav_hygrometer_sensor_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_HYGROMETER_SENSOR_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hygrometer_sensor_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hygrometer_sensor_get_field_humidity(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hygrometer_sensor_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HYGROMETER_SENSOR  12920

#define mavlink_hygrometer_sensor_t  fmav_hygrometer_sensor_t

#define MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN  5
#define MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN  5
#define MAVLINK_MSG_ID_12920_LEN  5
#define MAVLINK_MSG_ID_12920_MIN_LEN  5

#define MAVLINK_MSG_ID_HYGROMETER_SENSOR_CRC  20
#define MAVLINK_MSG_ID_12920_CRC  20




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hygrometer_sensor_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, int16_t temperature, uint16_t humidity)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hygrometer_sensor_pack(
        _msg, sysid, compid,
        id, temperature, humidity,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hygrometer_sensor_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_hygrometer_sensor_t* _payload)
{
    return mavlink_msg_hygrometer_sensor_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->temperature, _payload->humidity);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hygrometer_sensor_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t temperature, uint16_t humidity)
{
    return fmav_msg_hygrometer_sensor_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, temperature, humidity,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hygrometer_sensor_decode(const mavlink_message_t* msg, mavlink_hygrometer_sensor_t* payload)
{
    fmav_msg_hygrometer_sensor_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HYGROMETER_SENSOR_H
