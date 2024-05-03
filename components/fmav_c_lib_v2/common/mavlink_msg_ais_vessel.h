//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AIS_VESSEL_H
#define FASTMAVLINK_MSG_AIS_VESSEL_H


//----------------------------------------
//-- Message AIS_VESSEL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ais_vessel_t {
    uint32_t MMSI;
    int32_t lat;
    int32_t lon;
    uint16_t COG;
    uint16_t heading;
    uint16_t velocity;
    uint16_t dimension_bow;
    uint16_t dimension_stern;
    uint16_t tslc;
    uint16_t flags;
    int8_t turn_rate;
    uint8_t navigational_status;
    uint8_t type;
    uint8_t dimension_port;
    uint8_t dimension_starboard;
    char callsign[7];
    char name[20];
}) fmav_ais_vessel_t;


#define FASTMAVLINK_MSG_ID_AIS_VESSEL  301

#define FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX  58
#define FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA  243

#define FASTMAVLINK_MSG_AIS_VESSEL_FLAGS  0
#define FASTMAVLINK_MSG_AIS_VESSEL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AIS_VESSEL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AIS_VESSEL_FRAME_LEN_MAX  83

#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_CALLSIGN_NUM  7 // number of elements in array
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_CALLSIGN_LEN  7 // length of array = number of bytes
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_NAME_NUM  20 // number of elements in array
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_NAME_LEN  20 // length of array = number of bytes

#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_MMSI_OFS  0
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_LAT_OFS  4
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_LON_OFS  8
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_COG_OFS  12
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_HEADING_OFS  14
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_VELOCITY_OFS  16
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_DIMENSION_BOW_OFS  18
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_DIMENSION_STERN_OFS  20
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_TSLC_OFS  22
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_FLAGS_OFS  24
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_TURN_RATE_OFS  26
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_NAVIGATIONAL_STATUS_OFS  27
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_TYPE_OFS  28
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_DIMENSION_PORT_OFS  29
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_DIMENSION_STARBOARD_OFS  30
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_CALLSIGN_OFS  31
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_NAME_OFS  38


//----------------------------------------
//-- Message AIS_VESSEL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags,
    fmav_status_t* _status)
{
    fmav_ais_vessel_t* _payload = (fmav_ais_vessel_t*)_msg->payload;

    _payload->MMSI = MMSI;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->COG = COG;
    _payload->heading = heading;
    _payload->velocity = velocity;
    _payload->dimension_bow = dimension_bow;
    _payload->dimension_stern = dimension_stern;
    _payload->tslc = tslc;
    _payload->flags = flags;
    _payload->turn_rate = turn_rate;
    _payload->navigational_status = navigational_status;
    _payload->type = type;
    _payload->dimension_port = dimension_port;
    _payload->dimension_starboard = dimension_starboard;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*7);
    memcpy(&(_payload->name), name, sizeof(char)*20);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AIS_VESSEL;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ais_vessel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ais_vessel_pack(
        _msg, sysid, compid,
        _payload->MMSI, _payload->lat, _payload->lon, _payload->COG, _payload->heading, _payload->velocity, _payload->turn_rate, _payload->navigational_status, _payload->type, _payload->dimension_bow, _payload->dimension_stern, _payload->dimension_port, _payload->dimension_starboard, _payload->callsign, _payload->name, _payload->tslc, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags,
    fmav_status_t* _status)
{
    fmav_ais_vessel_t* _payload = (fmav_ais_vessel_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->MMSI = MMSI;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->COG = COG;
    _payload->heading = heading;
    _payload->velocity = velocity;
    _payload->dimension_bow = dimension_bow;
    _payload->dimension_stern = dimension_stern;
    _payload->tslc = tslc;
    _payload->flags = flags;
    _payload->turn_rate = turn_rate;
    _payload->navigational_status = navigational_status;
    _payload->type = type;
    _payload->dimension_port = dimension_port;
    _payload->dimension_starboard = dimension_starboard;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*7);
    memcpy(&(_payload->name), name, sizeof(char)*20);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AIS_VESSEL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AIS_VESSEL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AIS_VESSEL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ais_vessel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ais_vessel_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->MMSI, _payload->lat, _payload->lon, _payload->COG, _payload->heading, _payload->velocity, _payload->turn_rate, _payload->navigational_status, _payload->type, _payload->dimension_bow, _payload->dimension_stern, _payload->dimension_port, _payload->dimension_starboard, _payload->callsign, _payload->name, _payload->tslc, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags,
    fmav_status_t* _status)
{
    fmav_ais_vessel_t _payload;

    _payload.MMSI = MMSI;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.COG = COG;
    _payload.heading = heading;
    _payload.velocity = velocity;
    _payload.dimension_bow = dimension_bow;
    _payload.dimension_stern = dimension_stern;
    _payload.tslc = tslc;
    _payload.flags = flags;
    _payload.turn_rate = turn_rate;
    _payload.navigational_status = navigational_status;
    _payload.type = type;
    _payload.dimension_port = dimension_port;
    _payload.dimension_starboard = dimension_starboard;
    memcpy(&(_payload.callsign), callsign, sizeof(char)*7);
    memcpy(&(_payload.name), name, sizeof(char)*20);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AIS_VESSEL,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_ais_vessel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AIS_VESSEL,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AIS_VESSEL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ais_vessel_decode(fmav_ais_vessel_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_ais_vessel_get_field_MMSI(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_ais_vessel_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_ais_vessel_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_get_field_COG(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_get_field_heading(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_get_field_velocity(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_get_field_dimension_bow(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_get_field_dimension_stern(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_get_field_tslc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_ais_vessel_get_field_turn_rate(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_ais_vessel_get_field_navigational_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_ais_vessel_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_ais_vessel_get_field_dimension_port(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_ais_vessel_get_field_dimension_starboard(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_ais_vessel_get_field_callsign_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[31]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_ais_vessel_get_field_callsign(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AIS_VESSEL_FIELD_CALLSIGN_NUM) return 0;
    return ((char*)&(msg->payload[31]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_ais_vessel_get_field_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[38]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_ais_vessel_get_field_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AIS_VESSEL_FIELD_NAME_NUM) return 0;
    return ((char*)&(msg->payload[38]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AIS_VESSEL  301

#define mavlink_ais_vessel_t  fmav_ais_vessel_t

#define MAVLINK_MSG_ID_AIS_VESSEL_LEN  58
#define MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN  58
#define MAVLINK_MSG_ID_301_LEN  58
#define MAVLINK_MSG_ID_301_MIN_LEN  58

#define MAVLINK_MSG_ID_AIS_VESSEL_CRC  243
#define MAVLINK_MSG_ID_301_CRC  243

#define MAVLINK_MSG_AIS_VESSEL_FIELD_CALLSIGN_LEN 7
#define MAVLINK_MSG_AIS_VESSEL_FIELD_NAME_LEN 20


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ais_vessel_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ais_vessel_pack(
        _msg, sysid, compid,
        MMSI, lat, lon, COG, heading, velocity, turn_rate, navigational_status, type, dimension_bow, dimension_stern, dimension_port, dimension_starboard, callsign, name, tslc, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ais_vessel_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_ais_vessel_t* _payload)
{
    return mavlink_msg_ais_vessel_pack(
        sysid,
        compid,
        _msg,
        _payload->MMSI, _payload->lat, _payload->lon, _payload->COG, _payload->heading, _payload->velocity, _payload->turn_rate, _payload->navigational_status, _payload->type, _payload->dimension_bow, _payload->dimension_stern, _payload->dimension_port, _payload->dimension_starboard, _payload->callsign, _payload->name, _payload->tslc, _payload->flags);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ais_vessel_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags)
{
    return fmav_msg_ais_vessel_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        MMSI, lat, lon, COG, heading, velocity, turn_rate, navigational_status, type, dimension_bow, dimension_stern, dimension_port, dimension_starboard, callsign, name, tslc, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ais_vessel_decode(const mavlink_message_t* msg, mavlink_ais_vessel_t* payload)
{
    fmav_msg_ais_vessel_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AIS_VESSEL_H
