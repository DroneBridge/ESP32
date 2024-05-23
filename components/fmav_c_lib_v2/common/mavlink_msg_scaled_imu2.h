//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SCALED_IMU2_H
#define FASTMAVLINK_MSG_SCALED_IMU2_H


//----------------------------------------
//-- Message SCALED_IMU2
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_scaled_imu2_t {
    uint32_t time_boot_ms;
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
    int16_t xgyro;
    int16_t ygyro;
    int16_t zgyro;
    int16_t xmag;
    int16_t ymag;
    int16_t zmag;
    int16_t temperature;
}) fmav_scaled_imu2_t;


#define FASTMAVLINK_MSG_ID_SCALED_IMU2  116

#define FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_SCALED_IMU2_CRCEXTRA  76

#define FASTMAVLINK_MSG_SCALED_IMU2_FLAGS  0
#define FASTMAVLINK_MSG_SCALED_IMU2_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SCALED_IMU2_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SCALED_IMU2_FRAME_LEN_MAX  49



#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_XACC_OFS  4
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_YACC_OFS  6
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_ZACC_OFS  8
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_XGYRO_OFS  10
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_YGYRO_OFS  12
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_ZGYRO_OFS  14
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_XMAG_OFS  16
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_YMAG_OFS  18
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_ZMAG_OFS  20
#define FASTMAVLINK_MSG_SCALED_IMU2_FIELD_TEMPERATURE_OFS  22


//----------------------------------------
//-- Message SCALED_IMU2 pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu2_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_scaled_imu2_t* _payload = (fmav_scaled_imu2_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->xmag = xmag;
    _payload->ymag = ymag;
    _payload->zmag = zmag;
    _payload->temperature = temperature;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SCALED_IMU2;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SCALED_IMU2_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu2_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_imu2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_scaled_imu2_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->xmag, _payload->ymag, _payload->zmag, _payload->temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu2_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_scaled_imu2_t* _payload = (fmav_scaled_imu2_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->xmag = xmag;
    _payload->ymag = ymag;
    _payload->zmag = zmag;
    _payload->temperature = temperature;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SCALED_IMU2;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SCALED_IMU2 >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SCALED_IMU2 >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_IMU2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu2_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_imu2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_scaled_imu2_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->xmag, _payload->ymag, _payload->zmag, _payload->temperature,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu2_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_scaled_imu2_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.xacc = xacc;
    _payload.yacc = yacc;
    _payload.zacc = zacc;
    _payload.xgyro = xgyro;
    _payload.ygyro = ygyro;
    _payload.zgyro = zgyro;
    _payload.xmag = xmag;
    _payload.ymag = ymag;
    _payload.zmag = zmag;
    _payload.temperature = temperature;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SCALED_IMU2,
        FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_IMU2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu2_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_imu2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SCALED_IMU2,
        FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_IMU2_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SCALED_IMU2 decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_scaled_imu2_decode(fmav_scaled_imu2_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SCALED_IMU2_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_scaled_imu2_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_xacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_yacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_zacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_xgyro(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_ygyro(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_zgyro(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_xmag(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_ymag(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_zmag(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu2_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SCALED_IMU2  116

#define mavlink_scaled_imu2_t  fmav_scaled_imu2_t

#define MAVLINK_MSG_ID_SCALED_IMU2_LEN  24
#define MAVLINK_MSG_ID_SCALED_IMU2_MIN_LEN  22
#define MAVLINK_MSG_ID_116_LEN  24
#define MAVLINK_MSG_ID_116_MIN_LEN  22

#define MAVLINK_MSG_ID_SCALED_IMU2_CRC  76
#define MAVLINK_MSG_ID_116_CRC  76




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_scaled_imu2_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_scaled_imu2_pack(
        _msg, sysid, compid,
        time_boot_ms, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_scaled_imu2_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_scaled_imu2_t* _payload)
{
    return mavlink_msg_scaled_imu2_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->xmag, _payload->ymag, _payload->zmag, _payload->temperature);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_scaled_imu2_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature)
{
    return fmav_msg_scaled_imu2_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_scaled_imu2_decode(const mavlink_message_t* msg, mavlink_scaled_imu2_t* payload)
{
    fmav_msg_scaled_imu2_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SCALED_IMU2_H
