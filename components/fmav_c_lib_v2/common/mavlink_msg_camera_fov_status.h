//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_FOV_STATUS_H
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_H


//----------------------------------------
//-- Message CAMERA_FOV_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_fov_status_t {
    uint32_t time_boot_ms;
    int32_t lat_camera;
    int32_t lon_camera;
    int32_t alt_camera;
    int32_t lat_image;
    int32_t lon_image;
    int32_t alt_image;
    float q[4];
    float hfov;
    float vfov;
}) fmav_camera_fov_status_t;


#define FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS  271

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX  52
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA  22

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FRAME_LEN_MAX  77

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_LAT_CAMERA_OFS  4
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_LON_CAMERA_OFS  8
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_ALT_CAMERA_OFS  12
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_LAT_IMAGE_OFS  16
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_LON_IMAGE_OFS  20
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_ALT_IMAGE_OFS  24
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_OFS  28
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_HFOV_OFS  44
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_VFOV_OFS  48


//----------------------------------------
//-- Message CAMERA_FOV_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov,
    fmav_status_t* _status)
{
    fmav_camera_fov_status_t* _payload = (fmav_camera_fov_status_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat_camera = lat_camera;
    _payload->lon_camera = lon_camera;
    _payload->alt_camera = alt_camera;
    _payload->lat_image = lat_image;
    _payload->lon_image = lon_image;
    _payload->alt_image = alt_image;
    _payload->hfov = hfov;
    _payload->vfov = vfov;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_fov_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_fov_status_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->lat_camera, _payload->lon_camera, _payload->alt_camera, _payload->lat_image, _payload->lon_image, _payload->alt_image, _payload->q, _payload->hfov, _payload->vfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov,
    fmav_status_t* _status)
{
    fmav_camera_fov_status_t* _payload = (fmav_camera_fov_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat_camera = lat_camera;
    _payload->lon_camera = lon_camera;
    _payload->alt_camera = alt_camera;
    _payload->lat_image = lat_image;
    _payload->lon_image = lon_image;
    _payload->alt_image = alt_image;
    _payload->hfov = hfov;
    _payload->vfov = vfov;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_fov_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_fov_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->lat_camera, _payload->lon_camera, _payload->alt_camera, _payload->lat_image, _payload->lon_image, _payload->alt_image, _payload->q, _payload->hfov, _payload->vfov,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov,
    fmav_status_t* _status)
{
    fmav_camera_fov_status_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.lat_camera = lat_camera;
    _payload.lon_camera = lon_camera;
    _payload.alt_camera = alt_camera;
    _payload.lat_image = lat_image;
    _payload.lon_image = lon_image;
    _payload.alt_image = alt_image;
    _payload.hfov = hfov;
    _payload.vfov = vfov;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_fov_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_FOV_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_fov_status_decode(fmav_camera_fov_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_camera_fov_status_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_lat_camera(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_lon_camera(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_alt_camera(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_lat_image(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_lon_image(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_alt_image(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_fov_status_get_field_hfov(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_fov_status_get_field_vfov(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_camera_fov_status_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[28]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_fov_status_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[28]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS  271

#define mavlink_camera_fov_status_t  fmav_camera_fov_status_t

#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN  52
#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN  52
#define MAVLINK_MSG_ID_271_LEN  52
#define MAVLINK_MSG_ID_271_MIN_LEN  52

#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC  22
#define MAVLINK_MSG_ID_271_CRC  22

#define MAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_fov_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_fov_status_pack(
        _msg, sysid, compid,
        time_boot_ms, lat_camera, lon_camera, alt_camera, lat_image, lon_image, alt_image, q, hfov, vfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_fov_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_camera_fov_status_t* _payload)
{
    return mavlink_msg_camera_fov_status_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->lat_camera, _payload->lon_camera, _payload->alt_camera, _payload->lat_image, _payload->lon_image, _payload->alt_image, _payload->q, _payload->hfov, _payload->vfov);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_fov_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov)
{
    return fmav_msg_camera_fov_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, lat_camera, lon_camera, alt_camera, lat_image, lon_image, alt_image, q, hfov, vfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_fov_status_decode(const mavlink_message_t* msg, mavlink_camera_fov_status_t* payload)
{
    fmav_msg_camera_fov_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_FOV_STATUS_H
