//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ESC_STATUS_H
#define FASTMAVLINK_MSG_ESC_STATUS_H


//----------------------------------------
//-- Message ESC_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_esc_status_t {
    uint64_t time_usec;
    int32_t rpm[4];
    float voltage[4];
    float current[4];
    uint8_t index;
}) fmav_esc_status_t;


#define FASTMAVLINK_MSG_ID_ESC_STATUS  291

#define FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX  57
#define FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA  10

#define FASTMAVLINK_MSG_ESC_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ESC_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ESC_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ESC_STATUS_FRAME_LEN_MAX  82

#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_RPM_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_RPM_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_VOLTAGE_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_VOLTAGE_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_CURRENT_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_CURRENT_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_RPM_OFS  8
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_VOLTAGE_OFS  24
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_CURRENT_OFS  40
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_INDEX_OFS  56


//----------------------------------------
//-- Message ESC_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current,
    fmav_status_t* _status)
{
    fmav_esc_status_t* _payload = (fmav_esc_status_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->index = index;
    memcpy(&(_payload->rpm), rpm, sizeof(int32_t)*4);
    memcpy(&(_payload->voltage), voltage, sizeof(float)*4);
    memcpy(&(_payload->current), current, sizeof(float)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ESC_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_status_pack(
        _msg, sysid, compid,
        _payload->index, _payload->time_usec, _payload->rpm, _payload->voltage, _payload->current,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current,
    fmav_status_t* _status)
{
    fmav_esc_status_t* _payload = (fmav_esc_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->index = index;
    memcpy(&(_payload->rpm), rpm, sizeof(int32_t)*4);
    memcpy(&(_payload->voltage), voltage, sizeof(float)*4);
    memcpy(&(_payload->current), current, sizeof(float)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ESC_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->index, _payload->time_usec, _payload->rpm, _payload->voltage, _payload->current,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current,
    fmav_status_t* _status)
{
    fmav_esc_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.index = index;
    memcpy(&(_payload.rpm), rpm, sizeof(int32_t)*4);
    memcpy(&(_payload.voltage), voltage, sizeof(float)*4);
    memcpy(&(_payload.current), current, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ESC_STATUS,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ESC_STATUS,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ESC_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_esc_status_decode(fmav_esc_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_esc_status_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_status_get_field_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[56]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t* fmav_msg_esc_status_get_field_rpm_ptr(const fmav_message_t* msg)
{
    return (int32_t*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_esc_status_get_field_rpm(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_STATUS_FIELD_RPM_NUM) return 0;
    return ((int32_t*)&(msg->payload[8]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_esc_status_get_field_voltage_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[24]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_esc_status_get_field_voltage(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_STATUS_FIELD_VOLTAGE_NUM) return 0;
    return ((float*)&(msg->payload[24]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_esc_status_get_field_current_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[40]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_esc_status_get_field_current(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_STATUS_FIELD_CURRENT_NUM) return 0;
    return ((float*)&(msg->payload[40]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ESC_STATUS  291

#define mavlink_esc_status_t  fmav_esc_status_t

#define MAVLINK_MSG_ID_ESC_STATUS_LEN  57
#define MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN  57
#define MAVLINK_MSG_ID_291_LEN  57
#define MAVLINK_MSG_ID_291_MIN_LEN  57

#define MAVLINK_MSG_ID_ESC_STATUS_CRC  10
#define MAVLINK_MSG_ID_291_CRC  10

#define MAVLINK_MSG_ESC_STATUS_FIELD_RPM_LEN 4
#define MAVLINK_MSG_ESC_STATUS_FIELD_VOLTAGE_LEN 4
#define MAVLINK_MSG_ESC_STATUS_FIELD_CURRENT_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_esc_status_pack(
        _msg, sysid, compid,
        index, time_usec, rpm, voltage, current,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_esc_status_t* _payload)
{
    return mavlink_msg_esc_status_pack(
        sysid,
        compid,
        _msg,
        _payload->index, _payload->time_usec, _payload->rpm, _payload->voltage, _payload->current);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current)
{
    return fmav_msg_esc_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        index, time_usec, rpm, voltage, current,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_esc_status_decode(const mavlink_message_t* msg, mavlink_esc_status_t* payload)
{
    fmav_msg_esc_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ESC_STATUS_H
