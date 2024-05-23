//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COMPONENT_METADATA_H
#define FASTMAVLINK_MSG_COMPONENT_METADATA_H


//----------------------------------------
//-- Message COMPONENT_METADATA
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_component_metadata_t {
    uint32_t time_boot_ms;
    uint32_t file_crc;
    char uri[100];
}) fmav_component_metadata_t;


#define FASTMAVLINK_MSG_ID_COMPONENT_METADATA  397

#define FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX  108
#define FASTMAVLINK_MSG_COMPONENT_METADATA_CRCEXTRA  182

#define FASTMAVLINK_MSG_COMPONENT_METADATA_FLAGS  0
#define FASTMAVLINK_MSG_COMPONENT_METADATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_COMPONENT_METADATA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_COMPONENT_METADATA_FRAME_LEN_MAX  133

#define FASTMAVLINK_MSG_COMPONENT_METADATA_FIELD_URI_NUM  100 // number of elements in array
#define FASTMAVLINK_MSG_COMPONENT_METADATA_FIELD_URI_LEN  100 // length of array = number of bytes

#define FASTMAVLINK_MSG_COMPONENT_METADATA_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_COMPONENT_METADATA_FIELD_FILE_CRC_OFS  4
#define FASTMAVLINK_MSG_COMPONENT_METADATA_FIELD_URI_OFS  8


//----------------------------------------
//-- Message COMPONENT_METADATA pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_metadata_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t file_crc, const char* uri,
    fmav_status_t* _status)
{
    fmav_component_metadata_t* _payload = (fmav_component_metadata_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->file_crc = file_crc;
    memcpy(&(_payload->uri), uri, sizeof(char)*100);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_COMPONENT_METADATA;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_COMPONENT_METADATA_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_metadata_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_metadata_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_component_metadata_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->file_crc, _payload->uri,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_metadata_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t file_crc, const char* uri,
    fmav_status_t* _status)
{
    fmav_component_metadata_t* _payload = (fmav_component_metadata_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->file_crc = file_crc;
    memcpy(&(_payload->uri), uri, sizeof(char)*100);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COMPONENT_METADATA;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPONENT_METADATA >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPONENT_METADATA >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_METADATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_metadata_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_metadata_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_component_metadata_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->file_crc, _payload->uri,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_metadata_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t file_crc, const char* uri,
    fmav_status_t* _status)
{
    fmav_component_metadata_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.file_crc = file_crc;
    memcpy(&(_payload.uri), uri, sizeof(char)*100);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_COMPONENT_METADATA,
        FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_METADATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_metadata_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_metadata_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_COMPONENT_METADATA,
        FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_METADATA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message COMPONENT_METADATA decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_component_metadata_decode(fmav_component_metadata_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMPONENT_METADATA_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_component_metadata_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_component_metadata_get_field_file_crc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_component_metadata_get_field_uri_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_component_metadata_get_field_uri(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_COMPONENT_METADATA_FIELD_URI_NUM) return 0;
    return ((char*)&(msg->payload[8]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COMPONENT_METADATA  397

#define mavlink_component_metadata_t  fmav_component_metadata_t

#define MAVLINK_MSG_ID_COMPONENT_METADATA_LEN  108
#define MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN  108
#define MAVLINK_MSG_ID_397_LEN  108
#define MAVLINK_MSG_ID_397_MIN_LEN  108

#define MAVLINK_MSG_ID_COMPONENT_METADATA_CRC  182
#define MAVLINK_MSG_ID_397_CRC  182

#define MAVLINK_MSG_COMPONENT_METADATA_FIELD_URI_LEN 100


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_component_metadata_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint32_t file_crc, const char* uri)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_component_metadata_pack(
        _msg, sysid, compid,
        time_boot_ms, file_crc, uri,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_component_metadata_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_component_metadata_t* _payload)
{
    return mavlink_msg_component_metadata_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->file_crc, _payload->uri);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_component_metadata_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t file_crc, const char* uri)
{
    return fmav_msg_component_metadata_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, file_crc, uri,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_component_metadata_decode(const mavlink_message_t* msg, mavlink_component_metadata_t* payload)
{
    fmav_msg_component_metadata_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COMPONENT_METADATA_H
