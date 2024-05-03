//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS_STATUS_H
#define FASTMAVLINK_MSG_GPS_STATUS_H


//----------------------------------------
//-- Message GPS_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps_status_t {
    uint8_t satellites_visible;
    uint8_t satellite_prn[20];
    uint8_t satellite_used[20];
    uint8_t satellite_elevation[20];
    uint8_t satellite_azimuth[20];
    uint8_t satellite_snr[20];
}) fmav_gps_status_t;


#define FASTMAVLINK_MSG_ID_GPS_STATUS  25

#define FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX  101
#define FASTMAVLINK_MSG_GPS_STATUS_CRCEXTRA  23

#define FASTMAVLINK_MSG_GPS_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_GPS_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GPS_STATUS_FRAME_LEN_MAX  126

#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_NUM  20 // number of elements in array
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_USED_NUM  20 // number of elements in array
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_USED_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_NUM  20 // number of elements in array
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_NUM  20 // number of elements in array
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_NUM  20 // number of elements in array
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_LEN  20 // length of array = number of bytes

#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITES_VISIBLE_OFS  0
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_OFS  1
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_USED_OFS  21
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_OFS  41
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_OFS  61
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_OFS  81


//----------------------------------------
//-- Message GPS_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t satellites_visible, const uint8_t* satellite_prn, const uint8_t* satellite_used, const uint8_t* satellite_elevation, const uint8_t* satellite_azimuth, const uint8_t* satellite_snr,
    fmav_status_t* _status)
{
    fmav_gps_status_t* _payload = (fmav_gps_status_t*)_msg->payload;

    _payload->satellites_visible = satellites_visible;
    memcpy(&(_payload->satellite_prn), satellite_prn, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_used), satellite_used, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_elevation), satellite_elevation, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_azimuth), satellite_azimuth, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_snr), satellite_snr, sizeof(uint8_t)*20);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GPS_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_GPS_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_status_pack(
        _msg, sysid, compid,
        _payload->satellites_visible, _payload->satellite_prn, _payload->satellite_used, _payload->satellite_elevation, _payload->satellite_azimuth, _payload->satellite_snr,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t satellites_visible, const uint8_t* satellite_prn, const uint8_t* satellite_used, const uint8_t* satellite_elevation, const uint8_t* satellite_azimuth, const uint8_t* satellite_snr,
    fmav_status_t* _status)
{
    fmav_gps_status_t* _payload = (fmav_gps_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->satellites_visible = satellites_visible;
    memcpy(&(_payload->satellite_prn), satellite_prn, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_used), satellite_used, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_elevation), satellite_elevation, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_azimuth), satellite_azimuth, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_snr), satellite_snr, sizeof(uint8_t)*20);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->satellites_visible, _payload->satellite_prn, _payload->satellite_used, _payload->satellite_elevation, _payload->satellite_azimuth, _payload->satellite_snr,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t satellites_visible, const uint8_t* satellite_prn, const uint8_t* satellite_used, const uint8_t* satellite_elevation, const uint8_t* satellite_azimuth, const uint8_t* satellite_snr,
    fmav_status_t* _status)
{
    fmav_gps_status_t _payload;

    _payload.satellites_visible = satellites_visible;
    memcpy(&(_payload.satellite_prn), satellite_prn, sizeof(uint8_t)*20);
    memcpy(&(_payload.satellite_used), satellite_used, sizeof(uint8_t)*20);
    memcpy(&(_payload.satellite_elevation), satellite_elevation, sizeof(uint8_t)*20);
    memcpy(&(_payload.satellite_azimuth), satellite_azimuth, sizeof(uint8_t)*20);
    memcpy(&(_payload.satellite_snr), satellite_snr, sizeof(uint8_t)*20);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GPS_STATUS,
        FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GPS_STATUS,
        FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GPS_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps_status_decode(fmav_gps_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_status_get_field_satellites_visible(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_gps_status_get_field_satellite_prn_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[1]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_status_get_field_satellite_prn(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_NUM) return 0;
    return ((uint8_t*)&(msg->payload[1]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_gps_status_get_field_satellite_used_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[21]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_status_get_field_satellite_used(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_USED_NUM) return 0;
    return ((uint8_t*)&(msg->payload[21]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_gps_status_get_field_satellite_elevation_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[41]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_status_get_field_satellite_elevation(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_NUM) return 0;
    return ((uint8_t*)&(msg->payload[41]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_gps_status_get_field_satellite_azimuth_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[61]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_status_get_field_satellite_azimuth(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_NUM) return 0;
    return ((uint8_t*)&(msg->payload[61]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_gps_status_get_field_satellite_snr_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[81]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_status_get_field_satellite_snr(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_NUM) return 0;
    return ((uint8_t*)&(msg->payload[81]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS_STATUS  25

#define mavlink_gps_status_t  fmav_gps_status_t

#define MAVLINK_MSG_ID_GPS_STATUS_LEN  101
#define MAVLINK_MSG_ID_GPS_STATUS_MIN_LEN  101
#define MAVLINK_MSG_ID_25_LEN  101
#define MAVLINK_MSG_ID_25_MIN_LEN  101

#define MAVLINK_MSG_ID_GPS_STATUS_CRC  23
#define MAVLINK_MSG_ID_25_CRC  23

#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_USED_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_LEN 20


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t satellites_visible, const uint8_t* satellite_prn, const uint8_t* satellite_used, const uint8_t* satellite_elevation, const uint8_t* satellite_azimuth, const uint8_t* satellite_snr)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps_status_pack(
        _msg, sysid, compid,
        satellites_visible, satellite_prn, satellite_used, satellite_elevation, satellite_azimuth, satellite_snr,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gps_status_t* _payload)
{
    return mavlink_msg_gps_status_pack(
        sysid,
        compid,
        _msg,
        _payload->satellites_visible, _payload->satellite_prn, _payload->satellite_used, _payload->satellite_elevation, _payload->satellite_azimuth, _payload->satellite_snr);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t satellites_visible, const uint8_t* satellite_prn, const uint8_t* satellite_used, const uint8_t* satellite_elevation, const uint8_t* satellite_azimuth, const uint8_t* satellite_snr)
{
    return fmav_msg_gps_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        satellites_visible, satellite_prn, satellite_used, satellite_elevation, satellite_azimuth, satellite_snr,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps_status_decode(const mavlink_message_t* msg, mavlink_gps_status_t* payload)
{
    fmav_msg_gps_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS_STATUS_H
