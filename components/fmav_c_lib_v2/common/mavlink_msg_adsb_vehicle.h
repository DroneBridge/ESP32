//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ADSB_VEHICLE_H
#define FASTMAVLINK_MSG_ADSB_VEHICLE_H


//----------------------------------------
//-- Message ADSB_VEHICLE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_adsb_vehicle_t {
    uint32_t ICAO_address;
    int32_t lat;
    int32_t lon;
    int32_t altitude;
    uint16_t heading;
    uint16_t hor_velocity;
    int16_t ver_velocity;
    uint16_t flags;
    uint16_t squawk;
    uint8_t altitude_type;
    char callsign[9];
    uint8_t emitter_type;
    uint8_t tslc;
}) fmav_adsb_vehicle_t;


#define FASTMAVLINK_MSG_ID_ADSB_VEHICLE  246

#define FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX  38
#define FASTMAVLINK_MSG_ADSB_VEHICLE_CRCEXTRA  184

#define FASTMAVLINK_MSG_ADSB_VEHICLE_FLAGS  0
#define FASTMAVLINK_MSG_ADSB_VEHICLE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ADSB_VEHICLE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ADSB_VEHICLE_FRAME_LEN_MAX  63

#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_CALLSIGN_NUM  9 // number of elements in array
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_CALLSIGN_LEN  9 // length of array = number of bytes

#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_ICAO_ADDRESS_OFS  0
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_LAT_OFS  4
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_LON_OFS  8
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_ALTITUDE_OFS  12
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_HEADING_OFS  16
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_HOR_VELOCITY_OFS  18
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_VER_VELOCITY_OFS  20
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_FLAGS_OFS  22
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_SQUAWK_OFS  24
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_ALTITUDE_TYPE_OFS  26
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_CALLSIGN_OFS  27
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_EMITTER_TYPE_OFS  36
#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_TSLC_OFS  37


//----------------------------------------
//-- Message ADSB_VEHICLE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity, const char* callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_adsb_vehicle_t* _payload = (fmav_adsb_vehicle_t*)_msg->payload;

    _payload->ICAO_address = ICAO_address;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->altitude = altitude;
    _payload->heading = heading;
    _payload->hor_velocity = hor_velocity;
    _payload->ver_velocity = ver_velocity;
    _payload->flags = flags;
    _payload->squawk = squawk;
    _payload->altitude_type = altitude_type;
    _payload->emitter_type = emitter_type;
    _payload->tslc = tslc;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*9);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ADSB_VEHICLE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ADSB_VEHICLE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_adsb_vehicle_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_adsb_vehicle_pack(
        _msg, sysid, compid,
        _payload->ICAO_address, _payload->lat, _payload->lon, _payload->altitude_type, _payload->altitude, _payload->heading, _payload->hor_velocity, _payload->ver_velocity, _payload->callsign, _payload->emitter_type, _payload->tslc, _payload->flags, _payload->squawk,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity, const char* callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_adsb_vehicle_t* _payload = (fmav_adsb_vehicle_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->ICAO_address = ICAO_address;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->altitude = altitude;
    _payload->heading = heading;
    _payload->hor_velocity = hor_velocity;
    _payload->ver_velocity = ver_velocity;
    _payload->flags = flags;
    _payload->squawk = squawk;
    _payload->altitude_type = altitude_type;
    _payload->emitter_type = emitter_type;
    _payload->tslc = tslc;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*9);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ADSB_VEHICLE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ADSB_VEHICLE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ADSB_VEHICLE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ADSB_VEHICLE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_adsb_vehicle_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_adsb_vehicle_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->ICAO_address, _payload->lat, _payload->lon, _payload->altitude_type, _payload->altitude, _payload->heading, _payload->hor_velocity, _payload->ver_velocity, _payload->callsign, _payload->emitter_type, _payload->tslc, _payload->flags, _payload->squawk,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity, const char* callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_adsb_vehicle_t _payload;

    _payload.ICAO_address = ICAO_address;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.altitude = altitude;
    _payload.heading = heading;
    _payload.hor_velocity = hor_velocity;
    _payload.ver_velocity = ver_velocity;
    _payload.flags = flags;
    _payload.squawk = squawk;
    _payload.altitude_type = altitude_type;
    _payload.emitter_type = emitter_type;
    _payload.tslc = tslc;
    memcpy(&(_payload.callsign), callsign, sizeof(char)*9);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ADSB_VEHICLE,
        FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ADSB_VEHICLE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_adsb_vehicle_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ADSB_VEHICLE,
        FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ADSB_VEHICLE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ADSB_VEHICLE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_adsb_vehicle_decode(fmav_adsb_vehicle_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_adsb_vehicle_get_field_ICAO_address(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_adsb_vehicle_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_adsb_vehicle_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_adsb_vehicle_get_field_altitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_get_field_heading(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_get_field_hor_velocity(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_adsb_vehicle_get_field_ver_velocity(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_get_field_squawk(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_adsb_vehicle_get_field_altitude_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_adsb_vehicle_get_field_emitter_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_adsb_vehicle_get_field_tslc(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_adsb_vehicle_get_field_callsign_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[27]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_adsb_vehicle_get_field_callsign(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_CALLSIGN_NUM) return 0;
    return ((char*)&(msg->payload[27]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ADSB_VEHICLE  246

#define mavlink_adsb_vehicle_t  fmav_adsb_vehicle_t

#define MAVLINK_MSG_ID_ADSB_VEHICLE_LEN  38
#define MAVLINK_MSG_ID_ADSB_VEHICLE_MIN_LEN  38
#define MAVLINK_MSG_ID_246_LEN  38
#define MAVLINK_MSG_ID_246_MIN_LEN  38

#define MAVLINK_MSG_ID_ADSB_VEHICLE_CRC  184
#define MAVLINK_MSG_ID_246_CRC  184

#define MAVLINK_MSG_ADSB_VEHICLE_FIELD_CALLSIGN_LEN 9


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_adsb_vehicle_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity, const char* callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_adsb_vehicle_pack(
        _msg, sysid, compid,
        ICAO_address, lat, lon, altitude_type, altitude, heading, hor_velocity, ver_velocity, callsign, emitter_type, tslc, flags, squawk,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_adsb_vehicle_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_adsb_vehicle_t* _payload)
{
    return mavlink_msg_adsb_vehicle_pack(
        sysid,
        compid,
        _msg,
        _payload->ICAO_address, _payload->lat, _payload->lon, _payload->altitude_type, _payload->altitude, _payload->heading, _payload->hor_velocity, _payload->ver_velocity, _payload->callsign, _payload->emitter_type, _payload->tslc, _payload->flags, _payload->squawk);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_adsb_vehicle_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity, const char* callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk)
{
    return fmav_msg_adsb_vehicle_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        ICAO_address, lat, lon, altitude_type, altitude, heading, hor_velocity, ver_velocity, callsign, emitter_type, tslc, flags, squawk,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_adsb_vehicle_decode(const mavlink_message_t* msg, mavlink_adsb_vehicle_t* payload)
{
    fmav_msg_adsb_vehicle_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ADSB_VEHICLE_H
