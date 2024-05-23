//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_H
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_H


//----------------------------------------
//-- Message CAMERA_CAPTURE_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_capture_status_t {
    uint32_t time_boot_ms;
    float image_interval;
    uint32_t recording_time_ms;
    float available_capacity;
    uint8_t image_status;
    uint8_t video_status;
    int32_t image_count;
}) fmav_camera_capture_status_t;


#define FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS  262

#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_CRCEXTRA  12

#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FRAME_LEN_MAX  47



#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FIELD_IMAGE_INTERVAL_OFS  4
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FIELD_RECORDING_TIME_MS_OFS  8
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FIELD_AVAILABLE_CAPACITY_OFS  12
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FIELD_IMAGE_STATUS_OFS  16
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FIELD_VIDEO_STATUS_OFS  17
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FIELD_IMAGE_COUNT_OFS  18


//----------------------------------------
//-- Message CAMERA_CAPTURE_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count,
    fmav_status_t* _status)
{
    fmav_camera_capture_status_t* _payload = (fmav_camera_capture_status_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->image_interval = image_interval;
    _payload->recording_time_ms = recording_time_ms;
    _payload->available_capacity = available_capacity;
    _payload->image_status = image_status;
    _payload->video_status = video_status;
    _payload->image_count = image_count;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_capture_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_capture_status_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->image_status, _payload->video_status, _payload->image_interval, _payload->recording_time_ms, _payload->available_capacity, _payload->image_count,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count,
    fmav_status_t* _status)
{
    fmav_camera_capture_status_t* _payload = (fmav_camera_capture_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->image_interval = image_interval;
    _payload->recording_time_ms = recording_time_ms;
    _payload->available_capacity = available_capacity;
    _payload->image_status = image_status;
    _payload->video_status = video_status;
    _payload->image_count = image_count;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_capture_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_capture_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->image_status, _payload->video_status, _payload->image_interval, _payload->recording_time_ms, _payload->available_capacity, _payload->image_count,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count,
    fmav_status_t* _status)
{
    fmav_camera_capture_status_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.image_interval = image_interval;
    _payload.recording_time_ms = recording_time_ms;
    _payload.available_capacity = available_capacity;
    _payload.image_status = image_status;
    _payload.video_status = video_status;
    _payload.image_count = image_count;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_capture_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_CAPTURE_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_capture_status_decode(fmav_camera_capture_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_camera_capture_status_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_capture_status_get_field_image_interval(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_camera_capture_status_get_field_recording_time_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_capture_status_get_field_available_capacity(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_capture_status_get_field_image_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_capture_status_get_field_video_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[17]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_capture_status_get_field_image_count(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(int32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS  262

#define mavlink_camera_capture_status_t  fmav_camera_capture_status_t

#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN  22
#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN  18
#define MAVLINK_MSG_ID_262_LEN  22
#define MAVLINK_MSG_ID_262_MIN_LEN  18

#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC  12
#define MAVLINK_MSG_ID_262_CRC  12




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_capture_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_capture_status_pack(
        _msg, sysid, compid,
        time_boot_ms, image_status, video_status, image_interval, recording_time_ms, available_capacity, image_count,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_capture_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_camera_capture_status_t* _payload)
{
    return mavlink_msg_camera_capture_status_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->image_status, _payload->video_status, _payload->image_interval, _payload->recording_time_ms, _payload->available_capacity, _payload->image_count);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_capture_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count)
{
    return fmav_msg_camera_capture_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, image_status, video_status, image_interval, recording_time_ms, available_capacity, image_count,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_capture_status_decode(const mavlink_message_t* msg, mavlink_camera_capture_status_t* payload)
{
    fmav_msg_camera_capture_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_H
