//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ESC_INFO_H
#define FASTMAVLINK_MSG_ESC_INFO_H


//----------------------------------------
//-- Message ESC_INFO
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_esc_info_t {
    uint64_t time_usec;
    uint32_t error_count[4];
    uint16_t counter;
    uint16_t failure_flags[4];
    int16_t temperature[4];
    uint8_t index;
    uint8_t count;
    uint8_t connection_type;
    uint8_t info;
}) fmav_esc_info_t;


#define FASTMAVLINK_MSG_ID_ESC_INFO  290

#define FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX  46
#define FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA  251

#define FASTMAVLINK_MSG_ESC_INFO_FLAGS  0
#define FASTMAVLINK_MSG_ESC_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ESC_INFO_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ESC_INFO_FRAME_LEN_MAX  71

#define FASTMAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_LEN  8 // length of array = number of bytes

#define FASTMAVLINK_MSG_ESC_INFO_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_OFS  8
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_COUNTER_OFS  24
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_OFS  26
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_OFS  34
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_INDEX_OFS  42
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_COUNT_OFS  43
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_CONNECTION_TYPE_OFS  44
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_INFO_OFS  45


//----------------------------------------
//-- Message ESC_INFO pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const int16_t* temperature,
    fmav_status_t* _status)
{
    fmav_esc_info_t* _payload = (fmav_esc_info_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->counter = counter;
    _payload->index = index;
    _payload->count = count;
    _payload->connection_type = connection_type;
    _payload->info = info;
    memcpy(&(_payload->error_count), error_count, sizeof(uint32_t)*4);
    memcpy(&(_payload->failure_flags), failure_flags, sizeof(uint16_t)*4);
    memcpy(&(_payload->temperature), temperature, sizeof(int16_t)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ESC_INFO;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_info_pack(
        _msg, sysid, compid,
        _payload->index, _payload->time_usec, _payload->counter, _payload->count, _payload->connection_type, _payload->info, _payload->failure_flags, _payload->error_count, _payload->temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const int16_t* temperature,
    fmav_status_t* _status)
{
    fmav_esc_info_t* _payload = (fmav_esc_info_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->counter = counter;
    _payload->index = index;
    _payload->count = count;
    _payload->connection_type = connection_type;
    _payload->info = info;
    memcpy(&(_payload->error_count), error_count, sizeof(uint32_t)*4);
    memcpy(&(_payload->failure_flags), failure_flags, sizeof(uint16_t)*4);
    memcpy(&(_payload->temperature), temperature, sizeof(int16_t)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ESC_INFO;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_INFO >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_INFO >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_info_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->index, _payload->time_usec, _payload->counter, _payload->count, _payload->connection_type, _payload->info, _payload->failure_flags, _payload->error_count, _payload->temperature,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const int16_t* temperature,
    fmav_status_t* _status)
{
    fmav_esc_info_t _payload;

    _payload.time_usec = time_usec;
    _payload.counter = counter;
    _payload.index = index;
    _payload.count = count;
    _payload.connection_type = connection_type;
    _payload.info = info;
    memcpy(&(_payload.error_count), error_count, sizeof(uint32_t)*4);
    memcpy(&(_payload.failure_flags), failure_flags, sizeof(uint16_t)*4);
    memcpy(&(_payload.temperature), temperature, sizeof(int16_t)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ESC_INFO,
        FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ESC_INFO,
        FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ESC_INFO decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_esc_info_decode(fmav_esc_info_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_esc_info_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_get_field_counter(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_info_get_field_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_info_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[43]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_info_get_field_connection_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_info_get_field_info(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[45]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_esc_info_get_field_error_count_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_esc_info_get_field_error_count(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_NUM) return 0;
    return ((uint32_t*)&(msg->payload[8]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_esc_info_get_field_failure_flags_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[26]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_get_field_failure_flags(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_NUM) return 0;
    return ((uint16_t*)&(msg->payload[26]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t* fmav_msg_esc_info_get_field_temperature_ptr(const fmav_message_t* msg)
{
    return (int16_t*)&(msg->payload[34]);
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_esc_info_get_field_temperature(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_NUM) return 0;
    return ((int16_t*)&(msg->payload[34]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ESC_INFO  290

#define mavlink_esc_info_t  fmav_esc_info_t

#define MAVLINK_MSG_ID_ESC_INFO_LEN  46
#define MAVLINK_MSG_ID_ESC_INFO_MIN_LEN  46
#define MAVLINK_MSG_ID_290_LEN  46
#define MAVLINK_MSG_ID_290_MIN_LEN  46

#define MAVLINK_MSG_ID_ESC_INFO_CRC  251
#define MAVLINK_MSG_ID_290_CRC  251

#define MAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_LEN 4
#define MAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_LEN 4
#define MAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_info_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const int16_t* temperature)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_esc_info_pack(
        _msg, sysid, compid,
        index, time_usec, counter, count, connection_type, info, failure_flags, error_count, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_info_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_esc_info_t* _payload)
{
    return mavlink_msg_esc_info_pack(
        sysid,
        compid,
        _msg,
        _payload->index, _payload->time_usec, _payload->counter, _payload->count, _payload->connection_type, _payload->info, _payload->failure_flags, _payload->error_count, _payload->temperature);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_info_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const int16_t* temperature)
{
    return fmav_msg_esc_info_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        index, time_usec, counter, count, connection_type, info, failure_flags, error_count, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_esc_info_decode(const mavlink_message_t* msg, mavlink_esc_info_t* payload)
{
    fmav_msg_esc_info_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ESC_INFO_H
