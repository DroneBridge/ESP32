//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_H
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_H


//----------------------------------------
//-- Message UTM_GLOBAL_POSITION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_utm_global_position_t {
    uint64_t time;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int32_t next_lat;
    int32_t next_lon;
    int32_t next_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t h_acc;
    uint16_t v_acc;
    uint16_t vel_acc;
    uint16_t update_rate;
    uint8_t uas_id[18];
    uint8_t flight_state;
    uint8_t flags;
}) fmav_utm_global_position_t;


#define FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION  340

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX  70
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA  99

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FLAGS  0
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FRAME_LEN_MAX  95

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_NUM  18 // number of elements in array
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_LEN  18 // length of array = number of bytes

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_TIME_OFS  0
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_LON_OFS  12
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_ALT_OFS  16
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_RELATIVE_ALT_OFS  20
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_NEXT_LAT_OFS  24
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_NEXT_LON_OFS  28
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_NEXT_ALT_OFS  32
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_VX_OFS  36
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_VY_OFS  38
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_VZ_OFS  40
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_H_ACC_OFS  42
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_V_ACC_OFS  44
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_VEL_ACC_OFS  46
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UPDATE_RATE_OFS  48
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_OFS  50
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_FLIGHT_STATE_OFS  68
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_FLAGS_OFS  69


//----------------------------------------
//-- Message UTM_GLOBAL_POSITION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_utm_global_position_t* _payload = (fmav_utm_global_position_t*)_msg->payload;

    _payload->time = time;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->next_lat = next_lat;
    _payload->next_lon = next_lon;
    _payload->next_alt = next_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->update_rate = update_rate;
    _payload->flight_state = flight_state;
    _payload->flags = flags;
    memcpy(&(_payload->uas_id), uas_id, sizeof(uint8_t)*18);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_utm_global_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_utm_global_position_pack(
        _msg, sysid, compid,
        _payload->time, _payload->uas_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->next_lat, _payload->next_lon, _payload->next_alt, _payload->update_rate, _payload->flight_state, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_utm_global_position_t* _payload = (fmav_utm_global_position_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time = time;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->next_lat = next_lat;
    _payload->next_lon = next_lon;
    _payload->next_alt = next_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->update_rate = update_rate;
    _payload->flight_state = flight_state;
    _payload->flags = flags;
    memcpy(&(_payload->uas_id), uas_id, sizeof(uint8_t)*18);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_utm_global_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_utm_global_position_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time, _payload->uas_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->next_lat, _payload->next_lon, _payload->next_alt, _payload->update_rate, _payload->flight_state, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_utm_global_position_t _payload;

    _payload.time = time;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.relative_alt = relative_alt;
    _payload.next_lat = next_lat;
    _payload.next_lon = next_lon;
    _payload.next_alt = next_alt;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.h_acc = h_acc;
    _payload.v_acc = v_acc;
    _payload.vel_acc = vel_acc;
    _payload.update_rate = update_rate;
    _payload.flight_state = flight_state;
    _payload.flags = flags;
    memcpy(&(_payload.uas_id), uas_id, sizeof(uint8_t)*18);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_utm_global_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UTM_GLOBAL_POSITION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_utm_global_position_decode(fmav_utm_global_position_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_utm_global_position_get_field_time(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_relative_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_next_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_next_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_next_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_utm_global_position_get_field_vx(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_utm_global_position_get_field_vy(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_utm_global_position_get_field_vz(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_get_field_h_acc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_get_field_v_acc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_get_field_vel_acc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[46]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_get_field_update_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_utm_global_position_get_field_flight_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[68]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_utm_global_position_get_field_flags(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[69]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_utm_global_position_get_field_uas_id_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[50]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_utm_global_position_get_field_uas_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_NUM) return 0;
    return ((uint8_t*)&(msg->payload[50]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION  340

#define mavlink_utm_global_position_t  fmav_utm_global_position_t

#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_LEN  70
#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_MIN_LEN  70
#define MAVLINK_MSG_ID_340_LEN  70
#define MAVLINK_MSG_ID_340_MIN_LEN  70

#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_CRC  99
#define MAVLINK_MSG_ID_340_CRC  99

#define MAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_LEN 18


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_utm_global_position_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_utm_global_position_pack(
        _msg, sysid, compid,
        time, uas_id, lat, lon, alt, relative_alt, vx, vy, vz, h_acc, v_acc, vel_acc, next_lat, next_lon, next_alt, update_rate, flight_state, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_utm_global_position_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_utm_global_position_t* _payload)
{
    return mavlink_msg_utm_global_position_pack(
        sysid,
        compid,
        _msg,
        _payload->time, _payload->uas_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->next_lat, _payload->next_lon, _payload->next_alt, _payload->update_rate, _payload->flight_state, _payload->flags);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_utm_global_position_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags)
{
    return fmav_msg_utm_global_position_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time, uas_id, lat, lon, alt, relative_alt, vx, vy, vz, h_acc, v_acc, vel_acc, next_lat, next_lon, next_alt, update_rate, flight_state, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_utm_global_position_decode(const mavlink_message_t* msg, mavlink_utm_global_position_t* payload)
{
    fmav_msg_utm_global_position_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_H
