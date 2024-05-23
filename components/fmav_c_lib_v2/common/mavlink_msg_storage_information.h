//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_STORAGE_INFORMATION_H
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_H


//----------------------------------------
//-- Message STORAGE_INFORMATION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_storage_information_t {
    uint32_t time_boot_ms;
    float total_capacity;
    float used_capacity;
    float available_capacity;
    float read_speed;
    float write_speed;
    uint8_t storage_id;
    uint8_t storage_count;
    uint8_t status;
    uint8_t type;
    char name[32];
    uint8_t storage_usage;
}) fmav_storage_information_t;


#define FASTMAVLINK_MSG_ID_STORAGE_INFORMATION  261

#define FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX  61
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_CRCEXTRA  179

#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FRAME_LEN_MAX  86

#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_NAME_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_NAME_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_TOTAL_CAPACITY_OFS  4
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_USED_CAPACITY_OFS  8
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_AVAILABLE_CAPACITY_OFS  12
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_READ_SPEED_OFS  16
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_WRITE_SPEED_OFS  20
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_STORAGE_ID_OFS  24
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_STORAGE_COUNT_OFS  25
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_STATUS_OFS  26
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_TYPE_OFS  27
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_NAME_OFS  28
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_STORAGE_USAGE_OFS  60


//----------------------------------------
//-- Message STORAGE_INFORMATION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char* name, uint8_t storage_usage,
    fmav_status_t* _status)
{
    fmav_storage_information_t* _payload = (fmav_storage_information_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->total_capacity = total_capacity;
    _payload->used_capacity = used_capacity;
    _payload->available_capacity = available_capacity;
    _payload->read_speed = read_speed;
    _payload->write_speed = write_speed;
    _payload->storage_id = storage_id;
    _payload->storage_count = storage_count;
    _payload->status = status;
    _payload->type = type;
    _payload->storage_usage = storage_usage;
    memcpy(&(_payload->name), name, sizeof(char)*32);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_STORAGE_INFORMATION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_STORAGE_INFORMATION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storage_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storage_information_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->storage_id, _payload->storage_count, _payload->status, _payload->total_capacity, _payload->used_capacity, _payload->available_capacity, _payload->read_speed, _payload->write_speed, _payload->type, _payload->name, _payload->storage_usage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char* name, uint8_t storage_usage,
    fmav_status_t* _status)
{
    fmav_storage_information_t* _payload = (fmav_storage_information_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->total_capacity = total_capacity;
    _payload->used_capacity = used_capacity;
    _payload->available_capacity = available_capacity;
    _payload->read_speed = read_speed;
    _payload->write_speed = write_speed;
    _payload->storage_id = storage_id;
    _payload->storage_count = storage_count;
    _payload->status = status;
    _payload->type = type;
    _payload->storage_usage = storage_usage;
    memcpy(&(_payload->name), name, sizeof(char)*32);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_STORAGE_INFORMATION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_STORAGE_INFORMATION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_STORAGE_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storage_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storage_information_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->storage_id, _payload->storage_count, _payload->status, _payload->total_capacity, _payload->used_capacity, _payload->available_capacity, _payload->read_speed, _payload->write_speed, _payload->type, _payload->name, _payload->storage_usage,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char* name, uint8_t storage_usage,
    fmav_status_t* _status)
{
    fmav_storage_information_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.total_capacity = total_capacity;
    _payload.used_capacity = used_capacity;
    _payload.available_capacity = available_capacity;
    _payload.read_speed = read_speed;
    _payload.write_speed = write_speed;
    _payload.storage_id = storage_id;
    _payload.storage_count = storage_count;
    _payload.status = status;
    _payload.type = type;
    _payload.storage_usage = storage_usage;
    memcpy(&(_payload.name), name, sizeof(char)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_STORAGE_INFORMATION,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_storage_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_STORAGE_INFORMATION,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message STORAGE_INFORMATION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_storage_information_decode(fmav_storage_information_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_storage_information_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storage_information_get_field_total_capacity(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storage_information_get_field_used_capacity(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storage_information_get_field_available_capacity(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storage_information_get_field_read_speed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storage_information_get_field_write_speed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storage_information_get_field_storage_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storage_information_get_field_storage_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[25]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storage_information_get_field_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storage_information_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storage_information_get_field_storage_usage(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[60]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_storage_information_get_field_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[28]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_storage_information_get_field_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_NAME_NUM) return 0;
    return ((char*)&(msg->payload[28]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_STORAGE_INFORMATION  261

#define mavlink_storage_information_t  fmav_storage_information_t

#define MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN  61
#define MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN  27
#define MAVLINK_MSG_ID_261_LEN  61
#define MAVLINK_MSG_ID_261_MIN_LEN  27

#define MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC  179
#define MAVLINK_MSG_ID_261_CRC  179

#define MAVLINK_MSG_STORAGE_INFORMATION_FIELD_NAME_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storage_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char* name, uint8_t storage_usage)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_storage_information_pack(
        _msg, sysid, compid,
        time_boot_ms, storage_id, storage_count, status, total_capacity, used_capacity, available_capacity, read_speed, write_speed, type, name, storage_usage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storage_information_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_storage_information_t* _payload)
{
    return mavlink_msg_storage_information_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->storage_id, _payload->storage_count, _payload->status, _payload->total_capacity, _payload->used_capacity, _payload->available_capacity, _payload->read_speed, _payload->write_speed, _payload->type, _payload->name, _payload->storage_usage);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storage_information_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char* name, uint8_t storage_usage)
{
    return fmav_msg_storage_information_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, storage_id, storage_count, status, total_capacity, used_capacity, available_capacity, read_speed, write_speed, type, name, storage_usage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_storage_information_decode(const mavlink_message_t* msg, mavlink_storage_information_t* payload)
{
    fmav_msg_storage_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_STORAGE_INFORMATION_H
