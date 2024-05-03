//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS2_RAW_H
#define FASTMAVLINK_MSG_GPS2_RAW_H


//----------------------------------------
//-- Message GPS2_RAW
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps2_raw_t {
    uint64_t time_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint32_t dgps_age;
    uint16_t eph;
    uint16_t epv;
    uint16_t vel;
    uint16_t cog;
    uint8_t fix_type;
    uint8_t satellites_visible;
    uint8_t dgps_numch;
    uint16_t yaw;
    int32_t alt_ellipsoid;
    uint32_t h_acc;
    uint32_t v_acc;
    uint32_t vel_acc;
    uint32_t hdg_acc;
}) fmav_gps2_raw_t;


#define FASTMAVLINK_MSG_ID_GPS2_RAW  124

#define FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX  57
#define FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA  87

#define FASTMAVLINK_MSG_GPS2_RAW_FLAGS  0
#define FASTMAVLINK_MSG_GPS2_RAW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS2_RAW_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GPS2_RAW_FRAME_LEN_MAX  82



#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_LON_OFS  12
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_ALT_OFS  16
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_DGPS_AGE_OFS  20
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_EPH_OFS  24
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_EPV_OFS  26
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_VEL_OFS  28
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_COG_OFS  30
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_FIX_TYPE_OFS  32
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_SATELLITES_VISIBLE_OFS  33
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_DGPS_NUMCH_OFS  34
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_YAW_OFS  35
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_ALT_ELLIPSOID_OFS  37
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_H_ACC_OFS  41
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_V_ACC_OFS  45
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_VEL_ACC_OFS  49
#define FASTMAVLINK_MSG_GPS2_RAW_FIELD_HDG_ACC_OFS  53


//----------------------------------------
//-- Message GPS2_RAW pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc,
    fmav_status_t* _status)
{
    fmav_gps2_raw_t* _payload = (fmav_gps2_raw_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->dgps_age = dgps_age;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->dgps_numch = dgps_numch;
    _payload->yaw = yaw;
    _payload->alt_ellipsoid = alt_ellipsoid;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->hdg_acc = hdg_acc;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GPS2_RAW;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps2_raw_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->dgps_numch, _payload->dgps_age, _payload->yaw, _payload->alt_ellipsoid, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->hdg_acc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc,
    fmav_status_t* _status)
{
    fmav_gps2_raw_t* _payload = (fmav_gps2_raw_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->dgps_age = dgps_age;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->dgps_numch = dgps_numch;
    _payload->yaw = yaw;
    _payload->alt_ellipsoid = alt_ellipsoid;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->hdg_acc = hdg_acc;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS2_RAW;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS2_RAW >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS2_RAW >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps2_raw_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->dgps_numch, _payload->dgps_age, _payload->yaw, _payload->alt_ellipsoid, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->hdg_acc,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc,
    fmav_status_t* _status)
{
    fmav_gps2_raw_t _payload;

    _payload.time_usec = time_usec;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.dgps_age = dgps_age;
    _payload.eph = eph;
    _payload.epv = epv;
    _payload.vel = vel;
    _payload.cog = cog;
    _payload.fix_type = fix_type;
    _payload.satellites_visible = satellites_visible;
    _payload.dgps_numch = dgps_numch;
    _payload.yaw = yaw;
    _payload.alt_ellipsoid = alt_ellipsoid;
    _payload.h_acc = h_acc;
    _payload.v_acc = v_acc;
    _payload.vel_acc = vel_acc;
    _payload.hdg_acc = hdg_acc;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GPS2_RAW,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GPS2_RAW,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GPS2_RAW decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps2_raw_decode(fmav_gps2_raw_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_gps2_raw_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps2_raw_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps2_raw_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps2_raw_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps2_raw_get_field_dgps_age(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_get_field_eph(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_get_field_epv(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_get_field_vel(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_get_field_cog(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps2_raw_get_field_fix_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps2_raw_get_field_satellites_visible(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[33]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps2_raw_get_field_dgps_numch(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_get_field_yaw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps2_raw_get_field_alt_ellipsoid(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps2_raw_get_field_h_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[41]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps2_raw_get_field_v_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[45]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps2_raw_get_field_vel_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[49]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps2_raw_get_field_hdg_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[53]), sizeof(uint32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS2_RAW  124

#define mavlink_gps2_raw_t  fmav_gps2_raw_t

#define MAVLINK_MSG_ID_GPS2_RAW_LEN  57
#define MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN  35
#define MAVLINK_MSG_ID_124_LEN  57
#define MAVLINK_MSG_ID_124_MIN_LEN  35

#define MAVLINK_MSG_ID_GPS2_RAW_CRC  87
#define MAVLINK_MSG_ID_124_CRC  87




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_raw_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps2_raw_pack(
        _msg, sysid, compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, dgps_numch, dgps_age, yaw, alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_raw_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gps2_raw_t* _payload)
{
    return mavlink_msg_gps2_raw_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->dgps_numch, _payload->dgps_age, _payload->yaw, _payload->alt_ellipsoid, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->hdg_acc);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_raw_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
    return fmav_msg_gps2_raw_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, dgps_numch, dgps_age, yaw, alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps2_raw_decode(const mavlink_message_t* msg, mavlink_gps2_raw_t* payload)
{
    fmav_msg_gps2_raw_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS2_RAW_H
