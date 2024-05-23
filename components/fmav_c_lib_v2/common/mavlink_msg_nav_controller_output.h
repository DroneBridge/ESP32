//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_H
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_H


//----------------------------------------
//-- Message NAV_CONTROLLER_OUTPUT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_nav_controller_output_t {
    float nav_roll;
    float nav_pitch;
    float alt_error;
    float aspd_error;
    float xtrack_error;
    int16_t nav_bearing;
    int16_t target_bearing;
    uint16_t wp_dist;
}) fmav_nav_controller_output_t;


#define FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT  62

#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX  26
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA  183

#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FLAGS  0
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FRAME_LEN_MAX  51



#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_NAV_ROLL_OFS  0
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_NAV_PITCH_OFS  4
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_ALT_ERROR_OFS  8
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_ASPD_ERROR_OFS  12
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_XTRACK_ERROR_OFS  16
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_NAV_BEARING_OFS  20
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_TARGET_BEARING_OFS  22
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_WP_DIST_OFS  24


//----------------------------------------
//-- Message NAV_CONTROLLER_OUTPUT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error,
    fmav_status_t* _status)
{
    fmav_nav_controller_output_t* _payload = (fmav_nav_controller_output_t*)_msg->payload;

    _payload->nav_roll = nav_roll;
    _payload->nav_pitch = nav_pitch;
    _payload->alt_error = alt_error;
    _payload->aspd_error = aspd_error;
    _payload->xtrack_error = xtrack_error;
    _payload->nav_bearing = nav_bearing;
    _payload->target_bearing = target_bearing;
    _payload->wp_dist = wp_dist;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_controller_output_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_nav_controller_output_pack(
        _msg, sysid, compid,
        _payload->nav_roll, _payload->nav_pitch, _payload->nav_bearing, _payload->target_bearing, _payload->wp_dist, _payload->alt_error, _payload->aspd_error, _payload->xtrack_error,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error,
    fmav_status_t* _status)
{
    fmav_nav_controller_output_t* _payload = (fmav_nav_controller_output_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->nav_roll = nav_roll;
    _payload->nav_pitch = nav_pitch;
    _payload->alt_error = alt_error;
    _payload->aspd_error = aspd_error;
    _payload->xtrack_error = xtrack_error;
    _payload->nav_bearing = nav_bearing;
    _payload->target_bearing = target_bearing;
    _payload->wp_dist = wp_dist;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_controller_output_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_nav_controller_output_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->nav_roll, _payload->nav_pitch, _payload->nav_bearing, _payload->target_bearing, _payload->wp_dist, _payload->alt_error, _payload->aspd_error, _payload->xtrack_error,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error,
    fmav_status_t* _status)
{
    fmav_nav_controller_output_t _payload;

    _payload.nav_roll = nav_roll;
    _payload.nav_pitch = nav_pitch;
    _payload.alt_error = alt_error;
    _payload.aspd_error = aspd_error;
    _payload.xtrack_error = xtrack_error;
    _payload.nav_bearing = nav_bearing;
    _payload.target_bearing = target_bearing;
    _payload.wp_dist = wp_dist;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_controller_output_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message NAV_CONTROLLER_OUTPUT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_nav_controller_output_decode(fmav_nav_controller_output_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_nav_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_nav_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_alt_error(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_aspd_error(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_xtrack_error(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_nav_controller_output_get_field_nav_bearing(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_nav_controller_output_get_field_target_bearing(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_get_field_wp_dist(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT  62

#define mavlink_nav_controller_output_t  fmav_nav_controller_output_t

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN  26
#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_MIN_LEN  26
#define MAVLINK_MSG_ID_62_LEN  26
#define MAVLINK_MSG_ID_62_MIN_LEN  26

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC  183
#define MAVLINK_MSG_ID_62_CRC  183




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_nav_controller_output_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_nav_controller_output_pack(
        _msg, sysid, compid,
        nav_roll, nav_pitch, nav_bearing, target_bearing, wp_dist, alt_error, aspd_error, xtrack_error,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_nav_controller_output_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_nav_controller_output_t* _payload)
{
    return mavlink_msg_nav_controller_output_pack(
        sysid,
        compid,
        _msg,
        _payload->nav_roll, _payload->nav_pitch, _payload->nav_bearing, _payload->target_bearing, _payload->wp_dist, _payload->alt_error, _payload->aspd_error, _payload->xtrack_error);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_nav_controller_output_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
    return fmav_msg_nav_controller_output_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        nav_roll, nav_pitch, nav_bearing, target_bearing, wp_dist, alt_error, aspd_error, xtrack_error,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_nav_controller_output_decode(const mavlink_message_t* msg, mavlink_nav_controller_output_t* payload)
{
    fmav_msg_nav_controller_output_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_H
