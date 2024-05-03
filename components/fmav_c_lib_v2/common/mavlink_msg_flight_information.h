//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FLIGHT_INFORMATION_H
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_H


//----------------------------------------
//-- Message FLIGHT_INFORMATION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_flight_information_t {
    uint64_t arming_time_utc;
    uint64_t takeoff_time_utc;
    uint64_t flight_uuid;
    uint32_t time_boot_ms;
    uint32_t landing_time;
}) fmav_flight_information_t;


#define FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION  264

#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA  49

#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FRAME_LEN_MAX  57



#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FIELD_ARMING_TIME_UTC_OFS  0
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FIELD_TAKEOFF_TIME_UTC_OFS  8
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FIELD_FLIGHT_UUID_OFS  16
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FIELD_TIME_BOOT_MS_OFS  24
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FIELD_LANDING_TIME_OFS  28


//----------------------------------------
//-- Message FLIGHT_INFORMATION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid, uint32_t landing_time,
    fmav_status_t* _status)
{
    fmav_flight_information_t* _payload = (fmav_flight_information_t*)_msg->payload;

    _payload->arming_time_utc = arming_time_utc;
    _payload->takeoff_time_utc = takeoff_time_utc;
    _payload->flight_uuid = flight_uuid;
    _payload->time_boot_ms = time_boot_ms;
    _payload->landing_time = landing_time;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_flight_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_flight_information_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->arming_time_utc, _payload->takeoff_time_utc, _payload->flight_uuid, _payload->landing_time,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid, uint32_t landing_time,
    fmav_status_t* _status)
{
    fmav_flight_information_t* _payload = (fmav_flight_information_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->arming_time_utc = arming_time_utc;
    _payload->takeoff_time_utc = takeoff_time_utc;
    _payload->flight_uuid = flight_uuid;
    _payload->time_boot_ms = time_boot_ms;
    _payload->landing_time = landing_time;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_flight_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_flight_information_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->arming_time_utc, _payload->takeoff_time_utc, _payload->flight_uuid, _payload->landing_time,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid, uint32_t landing_time,
    fmav_status_t* _status)
{
    fmav_flight_information_t _payload;

    _payload.arming_time_utc = arming_time_utc;
    _payload.takeoff_time_utc = takeoff_time_utc;
    _payload.flight_uuid = flight_uuid;
    _payload.time_boot_ms = time_boot_ms;
    _payload.landing_time = landing_time;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_flight_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message FLIGHT_INFORMATION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_flight_information_decode(fmav_flight_information_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_flight_information_get_field_arming_time_utc(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_flight_information_get_field_takeoff_time_utc(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_flight_information_get_field_flight_uuid(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_flight_information_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_flight_information_get_field_landing_time(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FLIGHT_INFORMATION  264

#define mavlink_flight_information_t  fmav_flight_information_t

#define MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN  32
#define MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN  28
#define MAVLINK_MSG_ID_264_LEN  32
#define MAVLINK_MSG_ID_264_MIN_LEN  28

#define MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC  49
#define MAVLINK_MSG_ID_264_CRC  49




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_flight_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid, uint32_t landing_time)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_flight_information_pack(
        _msg, sysid, compid,
        time_boot_ms, arming_time_utc, takeoff_time_utc, flight_uuid, landing_time,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_flight_information_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_flight_information_t* _payload)
{
    return mavlink_msg_flight_information_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->arming_time_utc, _payload->takeoff_time_utc, _payload->flight_uuid, _payload->landing_time);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_flight_information_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid, uint32_t landing_time)
{
    return fmav_msg_flight_information_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, arming_time_utc, takeoff_time_utc, flight_uuid, landing_time,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_flight_information_decode(const mavlink_message_t* msg, mavlink_flight_information_t* payload)
{
    fmav_msg_flight_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FLIGHT_INFORMATION_H
