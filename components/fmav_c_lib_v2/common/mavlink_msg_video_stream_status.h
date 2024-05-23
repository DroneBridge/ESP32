//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_H
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_H


//----------------------------------------
//-- Message VIDEO_STREAM_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_video_stream_status_t {
    float framerate;
    uint32_t bitrate;
    uint16_t flags;
    uint16_t resolution_h;
    uint16_t resolution_v;
    uint16_t rotation;
    uint16_t hfov;
    uint8_t stream_id;
}) fmav_video_stream_status_t;


#define FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS  270

#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX  19
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_CRCEXTRA  59

#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FRAME_LEN_MAX  44



#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FIELD_FRAMERATE_OFS  0
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FIELD_BITRATE_OFS  4
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FIELD_FLAGS_OFS  8
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FIELD_RESOLUTION_H_OFS  10
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FIELD_RESOLUTION_V_OFS  12
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FIELD_ROTATION_OFS  14
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FIELD_HFOV_OFS  16
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FIELD_STREAM_ID_OFS  18


//----------------------------------------
//-- Message VIDEO_STREAM_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov,
    fmav_status_t* _status)
{
    fmav_video_stream_status_t* _payload = (fmav_video_stream_status_t*)_msg->payload;

    _payload->framerate = framerate;
    _payload->bitrate = bitrate;
    _payload->flags = flags;
    _payload->resolution_h = resolution_h;
    _payload->resolution_v = resolution_v;
    _payload->rotation = rotation;
    _payload->hfov = hfov;
    _payload->stream_id = stream_id;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_video_stream_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_video_stream_status_pack(
        _msg, sysid, compid,
        _payload->stream_id, _payload->flags, _payload->framerate, _payload->resolution_h, _payload->resolution_v, _payload->bitrate, _payload->rotation, _payload->hfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov,
    fmav_status_t* _status)
{
    fmav_video_stream_status_t* _payload = (fmav_video_stream_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->framerate = framerate;
    _payload->bitrate = bitrate;
    _payload->flags = flags;
    _payload->resolution_h = resolution_h;
    _payload->resolution_v = resolution_v;
    _payload->rotation = rotation;
    _payload->hfov = hfov;
    _payload->stream_id = stream_id;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_video_stream_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_video_stream_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->stream_id, _payload->flags, _payload->framerate, _payload->resolution_h, _payload->resolution_v, _payload->bitrate, _payload->rotation, _payload->hfov,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov,
    fmav_status_t* _status)
{
    fmav_video_stream_status_t _payload;

    _payload.framerate = framerate;
    _payload.bitrate = bitrate;
    _payload.flags = flags;
    _payload.resolution_h = resolution_h;
    _payload.resolution_v = resolution_v;
    _payload.rotation = rotation;
    _payload.hfov = hfov;
    _payload.stream_id = stream_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_video_stream_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message VIDEO_STREAM_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_video_stream_status_decode(fmav_video_stream_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_video_stream_status_get_field_framerate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_video_stream_status_get_field_bitrate(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_get_field_resolution_h(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_get_field_resolution_v(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_get_field_rotation(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_get_field_hfov(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_video_stream_status_get_field_stream_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VIDEO_STREAM_STATUS  270

#define mavlink_video_stream_status_t  fmav_video_stream_status_t

#define MAVLINK_MSG_ID_VIDEO_STREAM_STATUS_LEN  19
#define MAVLINK_MSG_ID_VIDEO_STREAM_STATUS_MIN_LEN  19
#define MAVLINK_MSG_ID_270_LEN  19
#define MAVLINK_MSG_ID_270_MIN_LEN  19

#define MAVLINK_MSG_ID_VIDEO_STREAM_STATUS_CRC  59
#define MAVLINK_MSG_ID_270_CRC  59




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_video_stream_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t stream_id, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_video_stream_status_pack(
        _msg, sysid, compid,
        stream_id, flags, framerate, resolution_h, resolution_v, bitrate, rotation, hfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_video_stream_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_video_stream_status_t* _payload)
{
    return mavlink_msg_video_stream_status_pack(
        sysid,
        compid,
        _msg,
        _payload->stream_id, _payload->flags, _payload->framerate, _payload->resolution_h, _payload->resolution_v, _payload->bitrate, _payload->rotation, _payload->hfov);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_video_stream_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov)
{
    return fmav_msg_video_stream_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        stream_id, flags, framerate, resolution_h, resolution_v, bitrate, rotation, hfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_video_stream_status_decode(const mavlink_message_t* msg, mavlink_video_stream_status_t* payload)
{
    fmav_msg_video_stream_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_H
