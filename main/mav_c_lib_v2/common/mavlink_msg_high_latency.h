//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIGH_LATENCY_H
#define FASTMAVLINK_MSG_HIGH_LATENCY_H


//----------------------------------------
//-- Message HIGH_LATENCY
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_high_latency_t {
    uint32_t custom_mode;
    int32_t latitude;
    int32_t longitude;
    int16_t roll;
    int16_t pitch;
    uint16_t heading;
    int16_t heading_sp;
    int16_t altitude_amsl;
    int16_t altitude_sp;
    uint16_t wp_distance;
    uint8_t base_mode;
    uint8_t landed_state;
    int8_t throttle;
    uint8_t airspeed;
    uint8_t airspeed_sp;
    uint8_t groundspeed;
    int8_t climb_rate;
    uint8_t gps_nsat;
    uint8_t gps_fix_type;
    uint8_t battery_remaining;
    int8_t temperature;
    int8_t temperature_air;
    uint8_t failsafe;
    uint8_t wp_num;
}) fmav_high_latency_t;


#define FASTMAVLINK_MSG_ID_HIGH_LATENCY  234

#define FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX  40
#define FASTMAVLINK_MSG_HIGH_LATENCY_CRCEXTRA  150

#define FASTMAVLINK_MSG_HIGH_LATENCY_FLAGS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIGH_LATENCY_FRAME_LEN_MAX  65



#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_CUSTOM_MODE_OFS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_LATITUDE_OFS  4
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_LONGITUDE_OFS  8
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_ROLL_OFS  12
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_PITCH_OFS  14
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_HEADING_OFS  16
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_HEADING_SP_OFS  18
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_ALTITUDE_AMSL_OFS  20
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_ALTITUDE_SP_OFS  22
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_WP_DISTANCE_OFS  24
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_BASE_MODE_OFS  26
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_LANDED_STATE_OFS  27
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_THROTTLE_OFS  28
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_AIRSPEED_OFS  29
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_AIRSPEED_SP_OFS  30
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_GROUNDSPEED_OFS  31
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_CLIMB_RATE_OFS  32
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_GPS_NSAT_OFS  33
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_GPS_FIX_TYPE_OFS  34
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_BATTERY_REMAINING_OFS  35
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_TEMPERATURE_OFS  36
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_TEMPERATURE_AIR_OFS  37
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_FAILSAFE_OFS  38
#define FASTMAVLINK_MSG_HIGH_LATENCY_FIELD_WP_NUM_OFS  39


//----------------------------------------
//-- Message HIGH_LATENCY pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance,
    fmav_status_t* _status)
{
    fmav_high_latency_t* _payload = (fmav_high_latency_t*)_msg->payload;

    _payload->custom_mode = custom_mode;
    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->heading = heading;
    _payload->heading_sp = heading_sp;
    _payload->altitude_amsl = altitude_amsl;
    _payload->altitude_sp = altitude_sp;
    _payload->wp_distance = wp_distance;
    _payload->base_mode = base_mode;
    _payload->landed_state = landed_state;
    _payload->throttle = throttle;
    _payload->airspeed = airspeed;
    _payload->airspeed_sp = airspeed_sp;
    _payload->groundspeed = groundspeed;
    _payload->climb_rate = climb_rate;
    _payload->gps_nsat = gps_nsat;
    _payload->gps_fix_type = gps_fix_type;
    _payload->battery_remaining = battery_remaining;
    _payload->temperature = temperature;
    _payload->temperature_air = temperature_air;
    _payload->failsafe = failsafe;
    _payload->wp_num = wp_num;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_HIGH_LATENCY;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_HIGH_LATENCY_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_high_latency_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_high_latency_pack(
        _msg, sysid, compid,
        _payload->base_mode, _payload->custom_mode, _payload->landed_state, _payload->roll, _payload->pitch, _payload->heading, _payload->throttle, _payload->heading_sp, _payload->latitude, _payload->longitude, _payload->altitude_amsl, _payload->altitude_sp, _payload->airspeed, _payload->airspeed_sp, _payload->groundspeed, _payload->climb_rate, _payload->gps_nsat, _payload->gps_fix_type, _payload->battery_remaining, _payload->temperature, _payload->temperature_air, _payload->failsafe, _payload->wp_num, _payload->wp_distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance,
    fmav_status_t* _status)
{
    fmav_high_latency_t* _payload = (fmav_high_latency_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->custom_mode = custom_mode;
    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->heading = heading;
    _payload->heading_sp = heading_sp;
    _payload->altitude_amsl = altitude_amsl;
    _payload->altitude_sp = altitude_sp;
    _payload->wp_distance = wp_distance;
    _payload->base_mode = base_mode;
    _payload->landed_state = landed_state;
    _payload->throttle = throttle;
    _payload->airspeed = airspeed;
    _payload->airspeed_sp = airspeed_sp;
    _payload->groundspeed = groundspeed;
    _payload->climb_rate = climb_rate;
    _payload->gps_nsat = gps_nsat;
    _payload->gps_fix_type = gps_fix_type;
    _payload->battery_remaining = battery_remaining;
    _payload->temperature = temperature;
    _payload->temperature_air = temperature_air;
    _payload->failsafe = failsafe;
    _payload->wp_num = wp_num;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIGH_LATENCY;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIGH_LATENCY >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIGH_LATENCY >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIGH_LATENCY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_high_latency_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_high_latency_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->base_mode, _payload->custom_mode, _payload->landed_state, _payload->roll, _payload->pitch, _payload->heading, _payload->throttle, _payload->heading_sp, _payload->latitude, _payload->longitude, _payload->altitude_amsl, _payload->altitude_sp, _payload->airspeed, _payload->airspeed_sp, _payload->groundspeed, _payload->climb_rate, _payload->gps_nsat, _payload->gps_fix_type, _payload->battery_remaining, _payload->temperature, _payload->temperature_air, _payload->failsafe, _payload->wp_num, _payload->wp_distance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance,
    fmav_status_t* _status)
{
    fmav_high_latency_t _payload;

    _payload.custom_mode = custom_mode;
    _payload.latitude = latitude;
    _payload.longitude = longitude;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.heading = heading;
    _payload.heading_sp = heading_sp;
    _payload.altitude_amsl = altitude_amsl;
    _payload.altitude_sp = altitude_sp;
    _payload.wp_distance = wp_distance;
    _payload.base_mode = base_mode;
    _payload.landed_state = landed_state;
    _payload.throttle = throttle;
    _payload.airspeed = airspeed;
    _payload.airspeed_sp = airspeed_sp;
    _payload.groundspeed = groundspeed;
    _payload.climb_rate = climb_rate;
    _payload.gps_nsat = gps_nsat;
    _payload.gps_fix_type = gps_fix_type;
    _payload.battery_remaining = battery_remaining;
    _payload.temperature = temperature;
    _payload.temperature_air = temperature_air;
    _payload.failsafe = failsafe;
    _payload.wp_num = wp_num;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIGH_LATENCY,
        FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIGH_LATENCY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_high_latency_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIGH_LATENCY,
        FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIGH_LATENCY_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIGH_LATENCY decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_high_latency_decode(fmav_high_latency_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_high_latency_get_field_custom_mode(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_high_latency_get_field_latitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_high_latency_get_field_longitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_high_latency_get_field_roll(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_high_latency_get_field_pitch(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_get_field_heading(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_high_latency_get_field_heading_sp(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_high_latency_get_field_altitude_amsl(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_high_latency_get_field_altitude_sp(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_get_field_wp_distance(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_base_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_landed_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency_get_field_throttle(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_airspeed(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_airspeed_sp(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_groundspeed(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[31]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency_get_field_climb_rate(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_gps_nsat(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[33]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_gps_fix_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_battery_remaining(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency_get_field_temperature(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency_get_field_temperature_air(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_failsafe(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency_get_field_wp_num(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[39]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIGH_LATENCY  234

#define mavlink_high_latency_t  fmav_high_latency_t

#define MAVLINK_MSG_ID_HIGH_LATENCY_LEN  40
#define MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN  40
#define MAVLINK_MSG_ID_234_LEN  40
#define MAVLINK_MSG_ID_234_MIN_LEN  40

#define MAVLINK_MSG_ID_HIGH_LATENCY_CRC  150
#define MAVLINK_MSG_ID_234_CRC  150




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_high_latency_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_high_latency_pack(
        _msg, sysid, compid,
        base_mode, custom_mode, landed_state, roll, pitch, heading, throttle, heading_sp, latitude, longitude, altitude_amsl, altitude_sp, airspeed, airspeed_sp, groundspeed, climb_rate, gps_nsat, gps_fix_type, battery_remaining, temperature, temperature_air, failsafe, wp_num, wp_distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_high_latency_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_high_latency_t* _payload)
{
    return mavlink_msg_high_latency_pack(
        sysid,
        compid,
        _msg,
        _payload->base_mode, _payload->custom_mode, _payload->landed_state, _payload->roll, _payload->pitch, _payload->heading, _payload->throttle, _payload->heading_sp, _payload->latitude, _payload->longitude, _payload->altitude_amsl, _payload->altitude_sp, _payload->airspeed, _payload->airspeed_sp, _payload->groundspeed, _payload->climb_rate, _payload->gps_nsat, _payload->gps_fix_type, _payload->battery_remaining, _payload->temperature, _payload->temperature_air, _payload->failsafe, _payload->wp_num, _payload->wp_distance);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_high_latency_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance)
{
    return fmav_msg_high_latency_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        base_mode, custom_mode, landed_state, roll, pitch, heading, throttle, heading_sp, latitude, longitude, altitude_amsl, altitude_sp, airspeed, airspeed_sp, groundspeed, climb_rate, gps_nsat, gps_fix_type, battery_remaining, temperature, temperature_air, failsafe, wp_num, wp_distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_high_latency_decode(const mavlink_message_t* msg, mavlink_high_latency_t* payload)
{
    fmav_msg_high_latency_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIGH_LATENCY_H
