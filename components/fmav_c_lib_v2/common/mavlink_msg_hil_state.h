//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_STATE_H
#define FASTMAVLINK_MSG_HIL_STATE_H


//----------------------------------------
//-- Message HIL_STATE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_state_t {
    uint64_t time_usec;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
}) fmav_hil_state_t;


#define FASTMAVLINK_MSG_ID_HIL_STATE  90

#define FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX  56
#define FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA  183

#define FASTMAVLINK_MSG_HIL_STATE_FLAGS  0
#define FASTMAVLINK_MSG_HIL_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_STATE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIL_STATE_FRAME_LEN_MAX  81



#define FASTMAVLINK_MSG_HIL_STATE_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_ROLL_OFS  8
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_PITCH_OFS  12
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_YAW_OFS  16
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_ROLLSPEED_OFS  20
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_PITCHSPEED_OFS  24
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_YAWSPEED_OFS  28
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_LAT_OFS  32
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_LON_OFS  36
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_ALT_OFS  40
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_VX_OFS  44
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_VY_OFS  46
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_VZ_OFS  48
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_XACC_OFS  50
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_YACC_OFS  52
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_ZACC_OFS  54


//----------------------------------------
//-- Message HIL_STATE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_t* _payload = (fmav_hil_state_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_HIL_STATE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_state_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->lat, _payload->lon, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->xacc, _payload->yacc, _payload->zacc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_t* _payload = (fmav_hil_state_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_STATE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_STATE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_STATE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_state_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->lat, _payload->lon, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->xacc, _payload->yacc, _payload->zacc,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_t _payload;

    _payload.time_usec = time_usec;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.rollspeed = rollspeed;
    _payload.pitchspeed = pitchspeed;
    _payload.yawspeed = yawspeed;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.xacc = xacc;
    _payload.yacc = yacc;
    _payload.zacc = zacc;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIL_STATE,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIL_STATE,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIL_STATE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_state_decode(fmav_hil_state_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_state_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_rollspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_pitchspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_yawspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_hil_state_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_hil_state_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_hil_state_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_vx(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_vy(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[46]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_vz(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_xacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_yacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[52]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_zacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[54]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_STATE  90

#define mavlink_hil_state_t  fmav_hil_state_t

#define MAVLINK_MSG_ID_HIL_STATE_LEN  56
#define MAVLINK_MSG_ID_HIL_STATE_MIN_LEN  56
#define MAVLINK_MSG_ID_90_LEN  56
#define MAVLINK_MSG_ID_90_MIN_LEN  56

#define MAVLINK_MSG_ID_HIL_STATE_CRC  183
#define MAVLINK_MSG_ID_90_CRC  183




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_state_pack(
        _msg, sysid, compid,
        time_usec, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, xacc, yacc, zacc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_state_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_hil_state_t* _payload)
{
    return mavlink_msg_hil_state_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->lat, _payload->lon, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->xacc, _payload->yacc, _payload->zacc);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_state_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
    return fmav_msg_hil_state_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, xacc, yacc, zacc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_state_decode(const mavlink_message_t* msg, mavlink_hil_state_t* payload)
{
    fmav_msg_hil_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_STATE_H
