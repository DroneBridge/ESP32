//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_H
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_H


//----------------------------------------
//-- Message CAMERA_TRACKING_IMAGE_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_tracking_image_status_t {
    float point_x;
    float point_y;
    float radius;
    float rec_top_x;
    float rec_top_y;
    float rec_bottom_x;
    float rec_bottom_y;
    uint8_t tracking_status;
    uint8_t tracking_mode;
    uint8_t target_data;
}) fmav_camera_tracking_image_status_t;


#define FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS  275

#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX  31
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_CRCEXTRA  126

#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FRAME_LEN_MAX  56



#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_POINT_X_OFS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_POINT_Y_OFS  4
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_RADIUS_OFS  8
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_REC_TOP_X_OFS  12
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_REC_TOP_Y_OFS  16
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_REC_BOTTOM_X_OFS  20
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_REC_BOTTOM_Y_OFS  24
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_TRACKING_STATUS_OFS  28
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_TRACKING_MODE_OFS  29
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FIELD_TARGET_DATA_OFS  30


//----------------------------------------
//-- Message CAMERA_TRACKING_IMAGE_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y,
    fmav_status_t* _status)
{
    fmav_camera_tracking_image_status_t* _payload = (fmav_camera_tracking_image_status_t*)_msg->payload;

    _payload->point_x = point_x;
    _payload->point_y = point_y;
    _payload->radius = radius;
    _payload->rec_top_x = rec_top_x;
    _payload->rec_top_y = rec_top_y;
    _payload->rec_bottom_x = rec_bottom_x;
    _payload->rec_bottom_y = rec_bottom_y;
    _payload->tracking_status = tracking_status;
    _payload->tracking_mode = tracking_mode;
    _payload->target_data = target_data;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_image_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_tracking_image_status_pack(
        _msg, sysid, compid,
        _payload->tracking_status, _payload->tracking_mode, _payload->target_data, _payload->point_x, _payload->point_y, _payload->radius, _payload->rec_top_x, _payload->rec_top_y, _payload->rec_bottom_x, _payload->rec_bottom_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y,
    fmav_status_t* _status)
{
    fmav_camera_tracking_image_status_t* _payload = (fmav_camera_tracking_image_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->point_x = point_x;
    _payload->point_y = point_y;
    _payload->radius = radius;
    _payload->rec_top_x = rec_top_x;
    _payload->rec_top_y = rec_top_y;
    _payload->rec_bottom_x = rec_bottom_x;
    _payload->rec_bottom_y = rec_bottom_y;
    _payload->tracking_status = tracking_status;
    _payload->tracking_mode = tracking_mode;
    _payload->target_data = target_data;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_image_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_tracking_image_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->tracking_status, _payload->tracking_mode, _payload->target_data, _payload->point_x, _payload->point_y, _payload->radius, _payload->rec_top_x, _payload->rec_top_y, _payload->rec_bottom_x, _payload->rec_bottom_y,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y,
    fmav_status_t* _status)
{
    fmav_camera_tracking_image_status_t _payload;

    _payload.point_x = point_x;
    _payload.point_y = point_y;
    _payload.radius = radius;
    _payload.rec_top_x = rec_top_x;
    _payload.rec_top_y = rec_top_y;
    _payload.rec_bottom_x = rec_bottom_x;
    _payload.rec_bottom_y = rec_bottom_y;
    _payload.tracking_status = tracking_status;
    _payload.tracking_mode = tracking_mode;
    _payload.target_data = target_data;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_image_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_TRACKING_IMAGE_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_tracking_image_status_decode(fmav_camera_tracking_image_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_image_status_get_field_point_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_image_status_get_field_point_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_image_status_get_field_radius(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_image_status_get_field_rec_top_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_image_status_get_field_rec_top_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_image_status_get_field_rec_bottom_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_image_status_get_field_rec_bottom_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_tracking_image_status_get_field_tracking_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_tracking_image_status_get_field_tracking_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_tracking_image_status_get_field_target_data(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS  275

#define mavlink_camera_tracking_image_status_t  fmav_camera_tracking_image_status_t

#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN  31
#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN  31
#define MAVLINK_MSG_ID_275_LEN  31
#define MAVLINK_MSG_ID_275_MIN_LEN  31

#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC  126
#define MAVLINK_MSG_ID_275_CRC  126




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_image_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_tracking_image_status_pack(
        _msg, sysid, compid,
        tracking_status, tracking_mode, target_data, point_x, point_y, radius, rec_top_x, rec_top_y, rec_bottom_x, rec_bottom_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_image_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_camera_tracking_image_status_t* _payload)
{
    return mavlink_msg_camera_tracking_image_status_pack(
        sysid,
        compid,
        _msg,
        _payload->tracking_status, _payload->tracking_mode, _payload->target_data, _payload->point_x, _payload->point_y, _payload->radius, _payload->rec_top_x, _payload->rec_top_y, _payload->rec_bottom_x, _payload->rec_bottom_y);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_image_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y)
{
    return fmav_msg_camera_tracking_image_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        tracking_status, tracking_mode, target_data, point_x, point_y, radius, rec_top_x, rec_top_y, rec_bottom_x, rec_bottom_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_tracking_image_status_decode(const mavlink_message_t* msg, mavlink_camera_tracking_image_status_t* payload)
{
    fmav_msg_camera_tracking_image_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_H
