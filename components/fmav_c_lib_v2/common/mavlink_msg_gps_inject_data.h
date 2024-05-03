//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS_INJECT_DATA_H
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_H


//----------------------------------------
//-- Message GPS_INJECT_DATA
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps_inject_data_t {
    uint8_t target_system;
    uint8_t target_component;
    uint8_t len;
    uint8_t data[110];
}) fmav_gps_inject_data_t;


#define FASTMAVLINK_MSG_ID_GPS_INJECT_DATA  123

#define FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX  113
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA  250

#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FLAGS  3
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_TARGET_COMPONENT_OFS  1

#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FRAME_LEN_MAX  138

#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_NUM  110 // number of elements in array
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN  110 // length of array = number of bytes

#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FIELD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FIELD_TARGET_COMPONENT_OFS  1
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FIELD_LEN_OFS  2
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_OFS  3


//----------------------------------------
//-- Message GPS_INJECT_DATA pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_gps_inject_data_t* _payload = (fmav_gps_inject_data_t*)_msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*110);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GPS_INJECT_DATA;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_inject_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_inject_data_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->len, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_gps_inject_data_t* _payload = (fmav_gps_inject_data_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*110);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS_INJECT_DATA;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_INJECT_DATA >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_INJECT_DATA >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_inject_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_inject_data_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->len, _payload->data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_gps_inject_data_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.len = len;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*110);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GPS_INJECT_DATA,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_inject_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GPS_INJECT_DATA,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GPS_INJECT_DATA decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps_inject_data_decode(fmav_gps_inject_data_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_inject_data_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_inject_data_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_inject_data_get_field_len(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_gps_inject_data_get_field_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[3]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_inject_data_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[3]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS_INJECT_DATA  123

#define mavlink_gps_inject_data_t  fmav_gps_inject_data_t

#define MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN  113
#define MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN  113
#define MAVLINK_MSG_ID_123_LEN  113
#define MAVLINK_MSG_ID_123_MIN_LEN  113

#define MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC  250
#define MAVLINK_MSG_ID_123_CRC  250

#define MAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN 110


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_inject_data_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps_inject_data_pack(
        _msg, sysid, compid,
        target_system, target_component, len, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_inject_data_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gps_inject_data_t* _payload)
{
    return mavlink_msg_gps_inject_data_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->len, _payload->data);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_inject_data_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data)
{
    return fmav_msg_gps_inject_data_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, len, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps_inject_data_decode(const mavlink_message_t* msg, mavlink_gps_inject_data_t* payload)
{
    fmav_msg_gps_inject_data_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS_INJECT_DATA_H
