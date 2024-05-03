//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_H
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_H


//----------------------------------------
//-- Message CAMERA_TRACKING_GEO_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_tracking_geo_status_t {
    int32_t lat;
    int32_t lon;
    float alt;
    float h_acc;
    float v_acc;
    float vel_n;
    float vel_e;
    float vel_d;
    float vel_acc;
    float dist;
    float hdg;
    float hdg_acc;
    uint8_t tracking_status;
}) fmav_camera_tracking_geo_status_t;


#define FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS  276

#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX  49
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_CRCEXTRA  18

#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FRAME_LEN_MAX  74



#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_LAT_OFS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_LON_OFS  4
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_ALT_OFS  8
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_H_ACC_OFS  12
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_V_ACC_OFS  16
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_VEL_N_OFS  20
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_VEL_E_OFS  24
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_VEL_D_OFS  28
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_VEL_ACC_OFS  32
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_DIST_OFS  36
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_HDG_OFS  40
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_HDG_ACC_OFS  44
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FIELD_TRACKING_STATUS_OFS  48


//----------------------------------------
//-- Message CAMERA_TRACKING_GEO_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc,
    fmav_status_t* _status)
{
    fmav_camera_tracking_geo_status_t* _payload = (fmav_camera_tracking_geo_status_t*)_msg->payload;

    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_n = vel_n;
    _payload->vel_e = vel_e;
    _payload->vel_d = vel_d;
    _payload->vel_acc = vel_acc;
    _payload->dist = dist;
    _payload->hdg = hdg;
    _payload->hdg_acc = hdg_acc;
    _payload->tracking_status = tracking_status;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_geo_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_tracking_geo_status_pack(
        _msg, sysid, compid,
        _payload->tracking_status, _payload->lat, _payload->lon, _payload->alt, _payload->h_acc, _payload->v_acc, _payload->vel_n, _payload->vel_e, _payload->vel_d, _payload->vel_acc, _payload->dist, _payload->hdg, _payload->hdg_acc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc,
    fmav_status_t* _status)
{
    fmav_camera_tracking_geo_status_t* _payload = (fmav_camera_tracking_geo_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_n = vel_n;
    _payload->vel_e = vel_e;
    _payload->vel_d = vel_d;
    _payload->vel_acc = vel_acc;
    _payload->dist = dist;
    _payload->hdg = hdg;
    _payload->hdg_acc = hdg_acc;
    _payload->tracking_status = tracking_status;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_geo_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_tracking_geo_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->tracking_status, _payload->lat, _payload->lon, _payload->alt, _payload->h_acc, _payload->v_acc, _payload->vel_n, _payload->vel_e, _payload->vel_d, _payload->vel_acc, _payload->dist, _payload->hdg, _payload->hdg_acc,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc,
    fmav_status_t* _status)
{
    fmav_camera_tracking_geo_status_t _payload;

    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.h_acc = h_acc;
    _payload.v_acc = v_acc;
    _payload.vel_n = vel_n;
    _payload.vel_e = vel_e;
    _payload.vel_d = vel_d;
    _payload.vel_acc = vel_acc;
    _payload.dist = dist;
    _payload.hdg = hdg;
    _payload.hdg_acc = hdg_acc;
    _payload.tracking_status = tracking_status;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_geo_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_TRACKING_GEO_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_tracking_geo_status_decode(fmav_camera_tracking_geo_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_tracking_geo_status_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_tracking_geo_status_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_h_acc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_v_acc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_vel_n(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_vel_e(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_vel_d(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_vel_acc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_dist(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_hdg(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_tracking_geo_status_get_field_hdg_acc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_tracking_geo_status_get_field_tracking_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS  276

#define mavlink_camera_tracking_geo_status_t  fmav_camera_tracking_geo_status_t

#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN  49
#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN  49
#define MAVLINK_MSG_ID_276_LEN  49
#define MAVLINK_MSG_ID_276_MIN_LEN  49

#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC  18
#define MAVLINK_MSG_ID_276_CRC  18




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_geo_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_tracking_geo_status_pack(
        _msg, sysid, compid,
        tracking_status, lat, lon, alt, h_acc, v_acc, vel_n, vel_e, vel_d, vel_acc, dist, hdg, hdg_acc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_geo_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_camera_tracking_geo_status_t* _payload)
{
    return mavlink_msg_camera_tracking_geo_status_pack(
        sysid,
        compid,
        _msg,
        _payload->tracking_status, _payload->lat, _payload->lon, _payload->alt, _payload->h_acc, _payload->v_acc, _payload->vel_n, _payload->vel_e, _payload->vel_d, _payload->vel_acc, _payload->dist, _payload->hdg, _payload->hdg_acc);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_geo_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc)
{
    return fmav_msg_camera_tracking_geo_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        tracking_status, lat, lon, alt, h_acc, v_acc, vel_n, vel_e, vel_d, vel_acc, dist, hdg, hdg_acc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_tracking_geo_status_decode(const mavlink_message_t* msg, mavlink_camera_tracking_geo_status_t* payload)
{
    fmav_msg_camera_tracking_geo_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_H
