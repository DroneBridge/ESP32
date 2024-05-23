//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_H
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_H


//----------------------------------------
//-- Message OPEN_DRONE_ID_LOCATION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_open_drone_id_location_t {
    int32_t latitude;
    int32_t longitude;
    float altitude_barometric;
    float altitude_geodetic;
    float height;
    float timestamp;
    uint16_t direction;
    uint16_t speed_horizontal;
    int16_t speed_vertical;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t id_or_mac[20];
    uint8_t status;
    uint8_t height_reference;
    uint8_t horizontal_accuracy;
    uint8_t vertical_accuracy;
    uint8_t barometer_accuracy;
    uint8_t speed_accuracy;
    uint8_t timestamp_accuracy;
}) fmav_open_drone_id_location_t;


#define FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION  12901

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX  59
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA  254

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FLAGS  3
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_TARGET_COMPONENT_OFS  31

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FRAME_LEN_MAX  84

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_ID_OR_MAC_NUM  20 // number of elements in array
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_ID_OR_MAC_LEN  20 // length of array = number of bytes

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_LATITUDE_OFS  0
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_LONGITUDE_OFS  4
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_ALTITUDE_BAROMETRIC_OFS  8
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_ALTITUDE_GEODETIC_OFS  12
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_HEIGHT_OFS  16
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_TIMESTAMP_OFS  20
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_DIRECTION_OFS  24
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_SPEED_HORIZONTAL_OFS  26
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_SPEED_VERTICAL_OFS  28
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_TARGET_COMPONENT_OFS  31
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_ID_OR_MAC_OFS  32
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_STATUS_OFS  52
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_HEIGHT_REFERENCE_OFS  53
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_HORIZONTAL_ACCURACY_OFS  54
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_VERTICAL_ACCURACY_OFS  55
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_BAROMETER_ACCURACY_OFS  56
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_SPEED_ACCURACY_OFS  57
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_TIMESTAMP_ACCURACY_OFS  58


//----------------------------------------
//-- Message OPEN_DRONE_ID_LOCATION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy,
    fmav_status_t* _status)
{
    fmav_open_drone_id_location_t* _payload = (fmav_open_drone_id_location_t*)_msg->payload;

    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->altitude_barometric = altitude_barometric;
    _payload->altitude_geodetic = altitude_geodetic;
    _payload->height = height;
    _payload->timestamp = timestamp;
    _payload->direction = direction;
    _payload->speed_horizontal = speed_horizontal;
    _payload->speed_vertical = speed_vertical;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->status = status;
    _payload->height_reference = height_reference;
    _payload->horizontal_accuracy = horizontal_accuracy;
    _payload->vertical_accuracy = vertical_accuracy;
    _payload->barometer_accuracy = barometer_accuracy;
    _payload->speed_accuracy = speed_accuracy;
    _payload->timestamp_accuracy = timestamp_accuracy;
    memcpy(&(_payload->id_or_mac), id_or_mac, sizeof(uint8_t)*20);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_location_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_open_drone_id_location_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->status, _payload->direction, _payload->speed_horizontal, _payload->speed_vertical, _payload->latitude, _payload->longitude, _payload->altitude_barometric, _payload->altitude_geodetic, _payload->height_reference, _payload->height, _payload->horizontal_accuracy, _payload->vertical_accuracy, _payload->barometer_accuracy, _payload->speed_accuracy, _payload->timestamp, _payload->timestamp_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy,
    fmav_status_t* _status)
{
    fmav_open_drone_id_location_t* _payload = (fmav_open_drone_id_location_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->altitude_barometric = altitude_barometric;
    _payload->altitude_geodetic = altitude_geodetic;
    _payload->height = height;
    _payload->timestamp = timestamp;
    _payload->direction = direction;
    _payload->speed_horizontal = speed_horizontal;
    _payload->speed_vertical = speed_vertical;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->status = status;
    _payload->height_reference = height_reference;
    _payload->horizontal_accuracy = horizontal_accuracy;
    _payload->vertical_accuracy = vertical_accuracy;
    _payload->barometer_accuracy = barometer_accuracy;
    _payload->speed_accuracy = speed_accuracy;
    _payload->timestamp_accuracy = timestamp_accuracy;
    memcpy(&(_payload->id_or_mac), id_or_mac, sizeof(uint8_t)*20);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_location_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_open_drone_id_location_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->status, _payload->direction, _payload->speed_horizontal, _payload->speed_vertical, _payload->latitude, _payload->longitude, _payload->altitude_barometric, _payload->altitude_geodetic, _payload->height_reference, _payload->height, _payload->horizontal_accuracy, _payload->vertical_accuracy, _payload->barometer_accuracy, _payload->speed_accuracy, _payload->timestamp, _payload->timestamp_accuracy,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy,
    fmav_status_t* _status)
{
    fmav_open_drone_id_location_t _payload;

    _payload.latitude = latitude;
    _payload.longitude = longitude;
    _payload.altitude_barometric = altitude_barometric;
    _payload.altitude_geodetic = altitude_geodetic;
    _payload.height = height;
    _payload.timestamp = timestamp;
    _payload.direction = direction;
    _payload.speed_horizontal = speed_horizontal;
    _payload.speed_vertical = speed_vertical;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.status = status;
    _payload.height_reference = height_reference;
    _payload.horizontal_accuracy = horizontal_accuracy;
    _payload.vertical_accuracy = vertical_accuracy;
    _payload.barometer_accuracy = barometer_accuracy;
    _payload.speed_accuracy = speed_accuracy;
    _payload.timestamp_accuracy = timestamp_accuracy;
    memcpy(&(_payload.id_or_mac), id_or_mac, sizeof(uint8_t)*20);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_location_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OPEN_DRONE_ID_LOCATION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_open_drone_id_location_decode(fmav_open_drone_id_location_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_open_drone_id_location_get_field_latitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_open_drone_id_location_get_field_longitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_open_drone_id_location_get_field_altitude_barometric(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_open_drone_id_location_get_field_altitude_geodetic(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_open_drone_id_location_get_field_height(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_open_drone_id_location_get_field_timestamp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_get_field_direction(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_get_field_speed_horizontal(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_open_drone_id_location_get_field_speed_vertical(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[31]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[52]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_height_reference(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[53]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_horizontal_accuracy(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[54]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_vertical_accuracy(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[55]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_barometer_accuracy(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[56]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_speed_accuracy(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[57]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_timestamp_accuracy(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[58]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_open_drone_id_location_get_field_id_or_mac_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[32]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_location_get_field_id_or_mac(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_ID_OR_MAC_NUM) return 0;
    return ((uint8_t*)&(msg->payload[32]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION  12901

#define mavlink_open_drone_id_location_t  fmav_open_drone_id_location_t

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_LEN  59
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_MIN_LEN  59
#define MAVLINK_MSG_ID_12901_LEN  59
#define MAVLINK_MSG_ID_12901_MIN_LEN  59

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_CRC  254
#define MAVLINK_MSG_ID_12901_CRC  254

#define MAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_ID_OR_MAC_LEN 20


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_location_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_open_drone_id_location_pack(
        _msg, sysid, compid,
        target_system, target_component, id_or_mac, status, direction, speed_horizontal, speed_vertical, latitude, longitude, altitude_barometric, altitude_geodetic, height_reference, height, horizontal_accuracy, vertical_accuracy, barometer_accuracy, speed_accuracy, timestamp, timestamp_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_location_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_open_drone_id_location_t* _payload)
{
    return mavlink_msg_open_drone_id_location_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->status, _payload->direction, _payload->speed_horizontal, _payload->speed_vertical, _payload->latitude, _payload->longitude, _payload->altitude_barometric, _payload->altitude_geodetic, _payload->height_reference, _payload->height, _payload->horizontal_accuracy, _payload->vertical_accuracy, _payload->barometer_accuracy, _payload->speed_accuracy, _payload->timestamp, _payload->timestamp_accuracy);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_location_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy)
{
    return fmav_msg_open_drone_id_location_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, id_or_mac, status, direction, speed_horizontal, speed_vertical, latitude, longitude, altitude_barometric, altitude_geodetic, height_reference, height, horizontal_accuracy, vertical_accuracy, barometer_accuracy, speed_accuracy, timestamp, timestamp_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_open_drone_id_location_decode(const mavlink_message_t* msg, mavlink_open_drone_id_location_t* payload)
{
    fmav_msg_open_drone_id_location_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_H
