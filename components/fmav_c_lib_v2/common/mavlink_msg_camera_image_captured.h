//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_H
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_H


//----------------------------------------
//-- Message CAMERA_IMAGE_CAPTURED
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_image_captured_t {
    uint64_t time_utc;
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    float q[4];
    int32_t image_index;
    uint8_t camera_id;
    int8_t capture_result;
    char file_url[205];
}) fmav_camera_image_captured_t;


#define FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED  263

#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX  255
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_CRCEXTRA  133

#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FRAME_LEN_MAX  280

#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_Q_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_FILE_URL_NUM  205 // number of elements in array
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_FILE_URL_LEN  205 // length of array = number of bytes

#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_TIME_UTC_OFS  0
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_TIME_BOOT_MS_OFS  8
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_LAT_OFS  12
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_LON_OFS  16
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_ALT_OFS  20
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_RELATIVE_ALT_OFS  24
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_Q_OFS  28
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_IMAGE_INDEX_OFS  44
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_CAMERA_ID_OFS  48
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_CAPTURE_RESULT_OFS  49
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_FILE_URL_OFS  50


//----------------------------------------
//-- Message CAMERA_IMAGE_CAPTURED pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float* q, int32_t image_index, int8_t capture_result, const char* file_url,
    fmav_status_t* _status)
{
    fmav_camera_image_captured_t* _payload = (fmav_camera_image_captured_t*)_msg->payload;

    _payload->time_utc = time_utc;
    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->image_index = image_index;
    _payload->camera_id = camera_id;
    _payload->capture_result = capture_result;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->file_url), file_url, sizeof(char)*205);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_image_captured_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_image_captured_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->time_utc, _payload->camera_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->q, _payload->image_index, _payload->capture_result, _payload->file_url,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float* q, int32_t image_index, int8_t capture_result, const char* file_url,
    fmav_status_t* _status)
{
    fmav_camera_image_captured_t* _payload = (fmav_camera_image_captured_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_utc = time_utc;
    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->image_index = image_index;
    _payload->camera_id = camera_id;
    _payload->capture_result = capture_result;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->file_url), file_url, sizeof(char)*205);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_image_captured_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_image_captured_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->time_utc, _payload->camera_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->q, _payload->image_index, _payload->capture_result, _payload->file_url,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float* q, int32_t image_index, int8_t capture_result, const char* file_url,
    fmav_status_t* _status)
{
    fmav_camera_image_captured_t _payload;

    _payload.time_utc = time_utc;
    _payload.time_boot_ms = time_boot_ms;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.relative_alt = relative_alt;
    _payload.image_index = image_index;
    _payload.camera_id = camera_id;
    _payload.capture_result = capture_result;
    memcpy(&(_payload.q), q, sizeof(float)*4);
    memcpy(&(_payload.file_url), file_url, sizeof(char)*205);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_image_captured_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_IMAGE_CAPTURED decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_image_captured_decode(fmav_camera_image_captured_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_camera_image_captured_get_field_time_utc(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_camera_image_captured_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_image_captured_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_image_captured_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_image_captured_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_image_captured_get_field_relative_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_image_captured_get_field_image_index(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_image_captured_get_field_camera_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_camera_image_captured_get_field_capture_result(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[49]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_camera_image_captured_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[28]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_image_captured_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[28]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_camera_image_captured_get_field_file_url_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[50]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_camera_image_captured_get_field_file_url(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_FILE_URL_NUM) return 0;
    return ((char*)&(msg->payload[50]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED  263

#define mavlink_camera_image_captured_t  fmav_camera_image_captured_t

#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN  255
#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN  255
#define MAVLINK_MSG_ID_263_LEN  255
#define MAVLINK_MSG_ID_263_MIN_LEN  255

#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC  133
#define MAVLINK_MSG_ID_263_CRC  133

#define MAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_Q_LEN 4
#define MAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_FILE_URL_LEN 205


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_image_captured_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float* q, int32_t image_index, int8_t capture_result, const char* file_url)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_image_captured_pack(
        _msg, sysid, compid,
        time_boot_ms, time_utc, camera_id, lat, lon, alt, relative_alt, q, image_index, capture_result, file_url,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_image_captured_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_camera_image_captured_t* _payload)
{
    return mavlink_msg_camera_image_captured_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->time_utc, _payload->camera_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->q, _payload->image_index, _payload->capture_result, _payload->file_url);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_image_captured_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float* q, int32_t image_index, int8_t capture_result, const char* file_url)
{
    return fmav_msg_camera_image_captured_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, time_utc, camera_id, lat, lon, alt, relative_alt, q, image_index, capture_result, file_url,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_image_captured_decode(const mavlink_message_t* msg, mavlink_camera_image_captured_t* payload)
{
    fmav_msg_camera_image_captured_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_H
