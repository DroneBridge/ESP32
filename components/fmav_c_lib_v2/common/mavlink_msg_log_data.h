//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOG_DATA_H
#define FASTMAVLINK_MSG_LOG_DATA_H


//----------------------------------------
//-- Message LOG_DATA
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_log_data_t {
    uint32_t ofs;
    uint16_t id;
    uint8_t count;
    uint8_t data[90];
}) fmav_log_data_t;


#define FASTMAVLINK_MSG_ID_LOG_DATA  120

#define FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX  97
#define FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA  134

#define FASTMAVLINK_MSG_LOG_DATA_FLAGS  0
#define FASTMAVLINK_MSG_LOG_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LOG_DATA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_LOG_DATA_FRAME_LEN_MAX  122

#define FASTMAVLINK_MSG_LOG_DATA_FIELD_DATA_NUM  90 // number of elements in array
#define FASTMAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN  90 // length of array = number of bytes

#define FASTMAVLINK_MSG_LOG_DATA_FIELD_OFS_OFS  0
#define FASTMAVLINK_MSG_LOG_DATA_FIELD_ID_OFS  4
#define FASTMAVLINK_MSG_LOG_DATA_FIELD_COUNT_OFS  6
#define FASTMAVLINK_MSG_LOG_DATA_FIELD_DATA_OFS  7


//----------------------------------------
//-- Message LOG_DATA pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_log_data_t* _payload = (fmav_log_data_t*)_msg->payload;

    _payload->ofs = ofs;
    _payload->id = id;
    _payload->count = count;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*90);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_LOG_DATA;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_data_pack(
        _msg, sysid, compid,
        _payload->id, _payload->ofs, _payload->count, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_log_data_t* _payload = (fmav_log_data_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->ofs = ofs;
    _payload->id = id;
    _payload->count = count;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*90);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOG_DATA;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_DATA >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_DATA >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_data_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->ofs, _payload->count, _payload->data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_log_data_t _payload;

    _payload.ofs = ofs;
    _payload.id = id;
    _payload.count = count;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*90);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LOG_DATA,
        FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LOG_DATA,
        FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LOG_DATA decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_data_decode(fmav_log_data_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_log_data_get_field_ofs(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_get_field_id(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_log_data_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_log_data_get_field_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[7]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_log_data_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_LOG_DATA_FIELD_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[7]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOG_DATA  120

#define mavlink_log_data_t  fmav_log_data_t

#define MAVLINK_MSG_ID_LOG_DATA_LEN  97
#define MAVLINK_MSG_ID_LOG_DATA_MIN_LEN  97
#define MAVLINK_MSG_ID_120_LEN  97
#define MAVLINK_MSG_ID_120_MIN_LEN  97

#define MAVLINK_MSG_ID_LOG_DATA_CRC  134
#define MAVLINK_MSG_ID_120_CRC  134

#define MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN 90


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_data_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_log_data_pack(
        _msg, sysid, compid,
        id, ofs, count, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_data_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_log_data_t* _payload)
{
    return mavlink_msg_log_data_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->ofs, _payload->count, _payload->data);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_data_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data)
{
    return fmav_msg_log_data_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, ofs, count, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_log_data_decode(const mavlink_message_t* msg, mavlink_log_data_t* payload)
{
    fmav_msg_log_data_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOG_DATA_H
