//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOG_ENTRY_H
#define FASTMAVLINK_MSG_LOG_ENTRY_H


//----------------------------------------
//-- Message LOG_ENTRY
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_log_entry_t {
    uint32_t time_utc;
    uint32_t size;
    uint16_t id;
    uint16_t num_logs;
    uint16_t last_log_num;
}) fmav_log_entry_t;


#define FASTMAVLINK_MSG_ID_LOG_ENTRY  118

#define FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX  14
#define FASTMAVLINK_MSG_LOG_ENTRY_CRCEXTRA  56

#define FASTMAVLINK_MSG_LOG_ENTRY_FLAGS  0
#define FASTMAVLINK_MSG_LOG_ENTRY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LOG_ENTRY_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_LOG_ENTRY_FRAME_LEN_MAX  39



#define FASTMAVLINK_MSG_LOG_ENTRY_FIELD_TIME_UTC_OFS  0
#define FASTMAVLINK_MSG_LOG_ENTRY_FIELD_SIZE_OFS  4
#define FASTMAVLINK_MSG_LOG_ENTRY_FIELD_ID_OFS  8
#define FASTMAVLINK_MSG_LOG_ENTRY_FIELD_NUM_LOGS_OFS  10
#define FASTMAVLINK_MSG_LOG_ENTRY_FIELD_LAST_LOG_NUM_OFS  12


//----------------------------------------
//-- Message LOG_ENTRY pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint16_t num_logs, uint16_t last_log_num, uint32_t time_utc, uint32_t size,
    fmav_status_t* _status)
{
    fmav_log_entry_t* _payload = (fmav_log_entry_t*)_msg->payload;

    _payload->time_utc = time_utc;
    _payload->size = size;
    _payload->id = id;
    _payload->num_logs = num_logs;
    _payload->last_log_num = last_log_num;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_LOG_ENTRY;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_LOG_ENTRY_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_entry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_entry_pack(
        _msg, sysid, compid,
        _payload->id, _payload->num_logs, _payload->last_log_num, _payload->time_utc, _payload->size,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint16_t num_logs, uint16_t last_log_num, uint32_t time_utc, uint32_t size,
    fmav_status_t* _status)
{
    fmav_log_entry_t* _payload = (fmav_log_entry_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_utc = time_utc;
    _payload->size = size;
    _payload->id = id;
    _payload->num_logs = num_logs;
    _payload->last_log_num = last_log_num;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOG_ENTRY;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_ENTRY >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_ENTRY >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_ENTRY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_entry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_entry_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->num_logs, _payload->last_log_num, _payload->time_utc, _payload->size,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint16_t num_logs, uint16_t last_log_num, uint32_t time_utc, uint32_t size,
    fmav_status_t* _status)
{
    fmav_log_entry_t _payload;

    _payload.time_utc = time_utc;
    _payload.size = size;
    _payload.id = id;
    _payload.num_logs = num_logs;
    _payload.last_log_num = last_log_num;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LOG_ENTRY,
        FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_ENTRY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_entry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LOG_ENTRY,
        FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_ENTRY_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LOG_ENTRY decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_entry_decode(fmav_log_entry_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_log_entry_get_field_time_utc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_log_entry_get_field_size(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_get_field_id(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_get_field_num_logs(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_get_field_last_log_num(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOG_ENTRY  118

#define mavlink_log_entry_t  fmav_log_entry_t

#define MAVLINK_MSG_ID_LOG_ENTRY_LEN  14
#define MAVLINK_MSG_ID_LOG_ENTRY_MIN_LEN  14
#define MAVLINK_MSG_ID_118_LEN  14
#define MAVLINK_MSG_ID_118_MIN_LEN  14

#define MAVLINK_MSG_ID_LOG_ENTRY_CRC  56
#define MAVLINK_MSG_ID_118_CRC  56




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_entry_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint16_t id, uint16_t num_logs, uint16_t last_log_num, uint32_t time_utc, uint32_t size)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_log_entry_pack(
        _msg, sysid, compid,
        id, num_logs, last_log_num, time_utc, size,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_entry_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_log_entry_t* _payload)
{
    return mavlink_msg_log_entry_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->num_logs, _payload->last_log_num, _payload->time_utc, _payload->size);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_entry_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint16_t num_logs, uint16_t last_log_num, uint32_t time_utc, uint32_t size)
{
    return fmav_msg_log_entry_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, num_logs, last_log_num, time_utc, size,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_log_entry_decode(const mavlink_message_t* msg, mavlink_log_entry_t* payload)
{
    fmav_msg_log_entry_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOG_ENTRY_H
