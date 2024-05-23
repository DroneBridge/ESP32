//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS_RAW_INT_H
#define FASTMAVLINK_MSG_GPS_RAW_INT_H


//----------------------------------------
//-- Message GPS_RAW_INT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps_raw_int_t {
    uint64_t time_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint16_t eph;
    uint16_t epv;
    uint16_t vel;
    uint16_t cog;
    uint8_t fix_type;
    uint8_t satellites_visible;
    int32_t alt_ellipsoid;
    uint32_t h_acc;
    uint32_t v_acc;
    uint32_t vel_acc;
    uint32_t hdg_acc;
    uint16_t yaw;
}) fmav_gps_raw_int_t;


#define FASTMAVLINK_MSG_ID_GPS_RAW_INT  24

#define FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX  52
#define FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA  24

#define FASTMAVLINK_MSG_GPS_RAW_INT_FLAGS  0
#define FASTMAVLINK_MSG_GPS_RAW_INT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS_RAW_INT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GPS_RAW_INT_FRAME_LEN_MAX  77



#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_LON_OFS  12
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_ALT_OFS  16
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_EPH_OFS  20
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_EPV_OFS  22
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_VEL_OFS  24
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_COG_OFS  26
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_FIX_TYPE_OFS  28
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_SATELLITES_VISIBLE_OFS  29
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_ALT_ELLIPSOID_OFS  30
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_H_ACC_OFS  34
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_V_ACC_OFS  38
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_VEL_ACC_OFS  42
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_HDG_ACC_OFS  46
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_YAW_OFS  50


//----------------------------------------
//-- Message GPS_RAW_INT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps_raw_int_t* _payload = (fmav_gps_raw_int_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->alt_ellipsoid = alt_ellipsoid;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->hdg_acc = hdg_acc;
    _payload->yaw = yaw;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GPS_RAW_INT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_raw_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_raw_int_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->alt_ellipsoid, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->hdg_acc, _payload->yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps_raw_int_t* _payload = (fmav_gps_raw_int_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->alt_ellipsoid = alt_ellipsoid;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->hdg_acc = hdg_acc;
    _payload->yaw = yaw;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS_RAW_INT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_RAW_INT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_RAW_INT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_raw_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_raw_int_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->alt_ellipsoid, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->hdg_acc, _payload->yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps_raw_int_t _payload;

    _payload.time_usec = time_usec;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.eph = eph;
    _payload.epv = epv;
    _payload.vel = vel;
    _payload.cog = cog;
    _payload.fix_type = fix_type;
    _payload.satellites_visible = satellites_visible;
    _payload.alt_ellipsoid = alt_ellipsoid;
    _payload.h_acc = h_acc;
    _payload.v_acc = v_acc;
    _payload.vel_acc = vel_acc;
    _payload.hdg_acc = hdg_acc;
    _payload.yaw = yaw;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GPS_RAW_INT,
        FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_raw_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GPS_RAW_INT,
        FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GPS_RAW_INT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps_raw_int_decode(fmav_gps_raw_int_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_gps_raw_int_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_raw_int_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_raw_int_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_raw_int_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_eph(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_epv(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_vel(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_cog(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_raw_int_get_field_fix_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_raw_int_get_field_satellites_visible(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_raw_int_get_field_alt_ellipsoid(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps_raw_int_get_field_h_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps_raw_int_get_field_v_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps_raw_int_get_field_vel_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps_raw_int_get_field_hdg_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[46]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_yaw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS_RAW_INT  24

#define mavlink_gps_raw_int_t  fmav_gps_raw_int_t

#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN  52
#define MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN  30
#define MAVLINK_MSG_ID_24_LEN  52
#define MAVLINK_MSG_ID_24_MIN_LEN  30

#define MAVLINK_MSG_ID_GPS_RAW_INT_CRC  24
#define MAVLINK_MSG_ID_24_CRC  24




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_raw_int_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps_raw_int_pack(
        _msg, sysid, compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_raw_int_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gps_raw_int_t* _payload)
{
    return mavlink_msg_gps_raw_int_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->alt_ellipsoid, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->hdg_acc, _payload->yaw);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_raw_int_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw)
{
    return fmav_msg_gps_raw_int_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps_raw_int_decode(const mavlink_message_t* msg, mavlink_gps_raw_int_t* payload)
{
    fmav_msg_gps_raw_int_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS_RAW_INT_H
