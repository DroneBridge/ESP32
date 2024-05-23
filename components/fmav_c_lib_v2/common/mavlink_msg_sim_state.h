//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SIM_STATE_H
#define FASTMAVLINK_MSG_SIM_STATE_H


//----------------------------------------
//-- Message SIM_STATE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sim_state_t {
    float q1;
    float q2;
    float q3;
    float q4;
    float roll;
    float pitch;
    float yaw;
    float xacc;
    float yacc;
    float zacc;
    float xgyro;
    float ygyro;
    float zgyro;
    float lat;
    float lon;
    float alt;
    float std_dev_horz;
    float std_dev_vert;
    float vn;
    float ve;
    float vd;
    int32_t lat_int;
    int32_t lon_int;
}) fmav_sim_state_t;


#define FASTMAVLINK_MSG_ID_SIM_STATE  108

#define FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX  92
#define FASTMAVLINK_MSG_SIM_STATE_CRCEXTRA  32

#define FASTMAVLINK_MSG_SIM_STATE_FLAGS  0
#define FASTMAVLINK_MSG_SIM_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SIM_STATE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SIM_STATE_FRAME_LEN_MAX  117



#define FASTMAVLINK_MSG_SIM_STATE_FIELD_Q1_OFS  0
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_Q2_OFS  4
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_Q3_OFS  8
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_Q4_OFS  12
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_ROLL_OFS  16
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_PITCH_OFS  20
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_YAW_OFS  24
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_XACC_OFS  28
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_YACC_OFS  32
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_ZACC_OFS  36
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_XGYRO_OFS  40
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_YGYRO_OFS  44
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_ZGYRO_OFS  48
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_LAT_OFS  52
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_LON_OFS  56
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_ALT_OFS  60
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_STD_DEV_HORZ_OFS  64
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_STD_DEV_VERT_OFS  68
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_VN_OFS  72
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_VE_OFS  76
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_VD_OFS  80
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_LAT_INT_OFS  84
#define FASTMAVLINK_MSG_SIM_STATE_FIELD_LON_INT_OFS  88


//----------------------------------------
//-- Message SIM_STATE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd, int32_t lat_int, int32_t lon_int,
    fmav_status_t* _status)
{
    fmav_sim_state_t* _payload = (fmav_sim_state_t*)_msg->payload;

    _payload->q1 = q1;
    _payload->q2 = q2;
    _payload->q3 = q3;
    _payload->q4 = q4;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->std_dev_horz = std_dev_horz;
    _payload->std_dev_vert = std_dev_vert;
    _payload->vn = vn;
    _payload->ve = ve;
    _payload->vd = vd;
    _payload->lat_int = lat_int;
    _payload->lon_int = lon_int;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SIM_STATE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SIM_STATE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sim_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sim_state_pack(
        _msg, sysid, compid,
        _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lon, _payload->alt, _payload->std_dev_horz, _payload->std_dev_vert, _payload->vn, _payload->ve, _payload->vd, _payload->lat_int, _payload->lon_int,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd, int32_t lat_int, int32_t lon_int,
    fmav_status_t* _status)
{
    fmav_sim_state_t* _payload = (fmav_sim_state_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->q1 = q1;
    _payload->q2 = q2;
    _payload->q3 = q3;
    _payload->q4 = q4;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->std_dev_horz = std_dev_horz;
    _payload->std_dev_vert = std_dev_vert;
    _payload->vn = vn;
    _payload->ve = ve;
    _payload->vd = vd;
    _payload->lat_int = lat_int;
    _payload->lon_int = lon_int;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SIM_STATE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SIM_STATE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SIM_STATE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SIM_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sim_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sim_state_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lon, _payload->alt, _payload->std_dev_horz, _payload->std_dev_vert, _payload->vn, _payload->ve, _payload->vd, _payload->lat_int, _payload->lon_int,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd, int32_t lat_int, int32_t lon_int,
    fmav_status_t* _status)
{
    fmav_sim_state_t _payload;

    _payload.q1 = q1;
    _payload.q2 = q2;
    _payload.q3 = q3;
    _payload.q4 = q4;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.xacc = xacc;
    _payload.yacc = yacc;
    _payload.zacc = zacc;
    _payload.xgyro = xgyro;
    _payload.ygyro = ygyro;
    _payload.zgyro = zgyro;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.std_dev_horz = std_dev_horz;
    _payload.std_dev_vert = std_dev_vert;
    _payload.vn = vn;
    _payload.ve = ve;
    _payload.vd = vd;
    _payload.lat_int = lat_int;
    _payload.lon_int = lon_int;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SIM_STATE,
        FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SIM_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_sim_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SIM_STATE,
        FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SIM_STATE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SIM_STATE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sim_state_decode(fmav_sim_state_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_q1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_q2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_q3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_q4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_xacc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_yacc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_zacc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_xgyro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_ygyro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_zgyro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_lat(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[52]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_lon(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[56]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[60]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_std_dev_horz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[64]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_std_dev_vert(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[68]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_vn(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[72]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_ve(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[76]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sim_state_get_field_vd(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[80]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_sim_state_get_field_lat_int(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[84]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_sim_state_get_field_lon_int(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[88]), sizeof(int32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SIM_STATE  108

#define mavlink_sim_state_t  fmav_sim_state_t

#define MAVLINK_MSG_ID_SIM_STATE_LEN  92
#define MAVLINK_MSG_ID_SIM_STATE_MIN_LEN  84
#define MAVLINK_MSG_ID_108_LEN  92
#define MAVLINK_MSG_ID_108_MIN_LEN  84

#define MAVLINK_MSG_ID_SIM_STATE_CRC  32
#define MAVLINK_MSG_ID_108_CRC  32




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sim_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd, int32_t lat_int, int32_t lon_int)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sim_state_pack(
        _msg, sysid, compid,
        q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, lat_int, lon_int,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sim_state_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_sim_state_t* _payload)
{
    return mavlink_msg_sim_state_pack(
        sysid,
        compid,
        _msg,
        _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lon, _payload->alt, _payload->std_dev_horz, _payload->std_dev_vert, _payload->vn, _payload->ve, _payload->vd, _payload->lat_int, _payload->lon_int);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sim_state_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd, int32_t lat_int, int32_t lon_int)
{
    return fmav_msg_sim_state_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd, lat_int, lon_int,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sim_state_decode(const mavlink_message_t* msg, mavlink_sim_state_t* payload)
{
    fmav_msg_sim_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SIM_STATE_H
