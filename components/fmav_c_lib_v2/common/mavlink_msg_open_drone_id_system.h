//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_H
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_H


//----------------------------------------
//-- Message OPEN_DRONE_ID_SYSTEM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_open_drone_id_system_t {
    int32_t operator_latitude;
    int32_t operator_longitude;
    float area_ceiling;
    float area_floor;
    float operator_altitude_geo;
    uint32_t timestamp;
    uint16_t area_count;
    uint16_t area_radius;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t id_or_mac[20];
    uint8_t operator_location_type;
    uint8_t classification_type;
    uint8_t category_eu;
    uint8_t class_eu;
}) fmav_open_drone_id_system_t;


#define FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM  12904

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX  54
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_CRCEXTRA  77

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FLAGS  3
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_TARGET_SYSTEM_OFS  28
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_TARGET_COMPONENT_OFS  29

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FRAME_LEN_MAX  79

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_ID_OR_MAC_NUM  20 // number of elements in array
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_ID_OR_MAC_LEN  20 // length of array = number of bytes

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_OPERATOR_LATITUDE_OFS  0
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_OPERATOR_LONGITUDE_OFS  4
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_AREA_CEILING_OFS  8
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_AREA_FLOOR_OFS  12
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_OPERATOR_ALTITUDE_GEO_OFS  16
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_TIMESTAMP_OFS  20
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_AREA_COUNT_OFS  24
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_AREA_RADIUS_OFS  26
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_TARGET_SYSTEM_OFS  28
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_TARGET_COMPONENT_OFS  29
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_ID_OR_MAC_OFS  30
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_OPERATOR_LOCATION_TYPE_OFS  50
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_CLASSIFICATION_TYPE_OFS  51
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_CATEGORY_EU_OFS  52
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_CLASS_EU_OFS  53


//----------------------------------------
//-- Message OPEN_DRONE_ID_SYSTEM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_system_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t operator_location_type, uint8_t classification_type, int32_t operator_latitude, int32_t operator_longitude, uint16_t area_count, uint16_t area_radius, float area_ceiling, float area_floor, uint8_t category_eu, uint8_t class_eu, float operator_altitude_geo, uint32_t timestamp,
    fmav_status_t* _status)
{
    fmav_open_drone_id_system_t* _payload = (fmav_open_drone_id_system_t*)_msg->payload;

    _payload->operator_latitude = operator_latitude;
    _payload->operator_longitude = operator_longitude;
    _payload->area_ceiling = area_ceiling;
    _payload->area_floor = area_floor;
    _payload->operator_altitude_geo = operator_altitude_geo;
    _payload->timestamp = timestamp;
    _payload->area_count = area_count;
    _payload->area_radius = area_radius;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->operator_location_type = operator_location_type;
    _payload->classification_type = classification_type;
    _payload->category_eu = category_eu;
    _payload->class_eu = class_eu;
    memcpy(&(_payload->id_or_mac), id_or_mac, sizeof(uint8_t)*20);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_system_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_system_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_open_drone_id_system_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->operator_location_type, _payload->classification_type, _payload->operator_latitude, _payload->operator_longitude, _payload->area_count, _payload->area_radius, _payload->area_ceiling, _payload->area_floor, _payload->category_eu, _payload->class_eu, _payload->operator_altitude_geo, _payload->timestamp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_system_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t operator_location_type, uint8_t classification_type, int32_t operator_latitude, int32_t operator_longitude, uint16_t area_count, uint16_t area_radius, float area_ceiling, float area_floor, uint8_t category_eu, uint8_t class_eu, float operator_altitude_geo, uint32_t timestamp,
    fmav_status_t* _status)
{
    fmav_open_drone_id_system_t* _payload = (fmav_open_drone_id_system_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->operator_latitude = operator_latitude;
    _payload->operator_longitude = operator_longitude;
    _payload->area_ceiling = area_ceiling;
    _payload->area_floor = area_floor;
    _payload->operator_altitude_geo = operator_altitude_geo;
    _payload->timestamp = timestamp;
    _payload->area_count = area_count;
    _payload->area_radius = area_radius;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->operator_location_type = operator_location_type;
    _payload->classification_type = classification_type;
    _payload->category_eu = category_eu;
    _payload->class_eu = class_eu;
    memcpy(&(_payload->id_or_mac), id_or_mac, sizeof(uint8_t)*20);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_system_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_system_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_open_drone_id_system_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->operator_location_type, _payload->classification_type, _payload->operator_latitude, _payload->operator_longitude, _payload->area_count, _payload->area_radius, _payload->area_ceiling, _payload->area_floor, _payload->category_eu, _payload->class_eu, _payload->operator_altitude_geo, _payload->timestamp,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_system_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t operator_location_type, uint8_t classification_type, int32_t operator_latitude, int32_t operator_longitude, uint16_t area_count, uint16_t area_radius, float area_ceiling, float area_floor, uint8_t category_eu, uint8_t class_eu, float operator_altitude_geo, uint32_t timestamp,
    fmav_status_t* _status)
{
    fmav_open_drone_id_system_t _payload;

    _payload.operator_latitude = operator_latitude;
    _payload.operator_longitude = operator_longitude;
    _payload.area_ceiling = area_ceiling;
    _payload.area_floor = area_floor;
    _payload.operator_altitude_geo = operator_altitude_geo;
    _payload.timestamp = timestamp;
    _payload.area_count = area_count;
    _payload.area_radius = area_radius;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.operator_location_type = operator_location_type;
    _payload.classification_type = classification_type;
    _payload.category_eu = category_eu;
    _payload.class_eu = class_eu;
    memcpy(&(_payload.id_or_mac), id_or_mac, sizeof(uint8_t)*20);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_system_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_system_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OPEN_DRONE_ID_SYSTEM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_open_drone_id_system_decode(fmav_open_drone_id_system_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_open_drone_id_system_get_field_operator_latitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_open_drone_id_system_get_field_operator_longitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_open_drone_id_system_get_field_area_ceiling(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_open_drone_id_system_get_field_area_floor(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_open_drone_id_system_get_field_operator_altitude_geo(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_open_drone_id_system_get_field_timestamp(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_system_get_field_area_count(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_system_get_field_area_radius(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_system_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_system_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_system_get_field_operator_location_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_system_get_field_classification_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[51]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_system_get_field_category_eu(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[52]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_system_get_field_class_eu(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[53]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_open_drone_id_system_get_field_id_or_mac_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[30]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_system_get_field_id_or_mac(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_ID_OR_MAC_NUM) return 0;
    return ((uint8_t*)&(msg->payload[30]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM  12904

#define mavlink_open_drone_id_system_t  fmav_open_drone_id_system_t

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN  54
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN  54
#define MAVLINK_MSG_ID_12904_LEN  54
#define MAVLINK_MSG_ID_12904_MIN_LEN  54

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_CRC  77
#define MAVLINK_MSG_ID_12904_CRC  77

#define MAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_FIELD_ID_OR_MAC_LEN 20


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_system_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t operator_location_type, uint8_t classification_type, int32_t operator_latitude, int32_t operator_longitude, uint16_t area_count, uint16_t area_radius, float area_ceiling, float area_floor, uint8_t category_eu, uint8_t class_eu, float operator_altitude_geo, uint32_t timestamp)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_open_drone_id_system_pack(
        _msg, sysid, compid,
        target_system, target_component, id_or_mac, operator_location_type, classification_type, operator_latitude, operator_longitude, area_count, area_radius, area_ceiling, area_floor, category_eu, class_eu, operator_altitude_geo, timestamp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_system_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_open_drone_id_system_t* _payload)
{
    return mavlink_msg_open_drone_id_system_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->operator_location_type, _payload->classification_type, _payload->operator_latitude, _payload->operator_longitude, _payload->area_count, _payload->area_radius, _payload->area_ceiling, _payload->area_floor, _payload->category_eu, _payload->class_eu, _payload->operator_altitude_geo, _payload->timestamp);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_system_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t operator_location_type, uint8_t classification_type, int32_t operator_latitude, int32_t operator_longitude, uint16_t area_count, uint16_t area_radius, float area_ceiling, float area_floor, uint8_t category_eu, uint8_t class_eu, float operator_altitude_geo, uint32_t timestamp)
{
    return fmav_msg_open_drone_id_system_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, id_or_mac, operator_location_type, classification_type, operator_latitude, operator_longitude, area_count, area_radius, area_ceiling, area_floor, category_eu, class_eu, operator_altitude_geo, timestamp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_open_drone_id_system_decode(const mavlink_message_t* msg, mavlink_open_drone_id_system_t* payload)
{
    fmav_msg_open_drone_id_system_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OPEN_DRONE_ID_SYSTEM_H
