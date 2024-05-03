//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ILLUMINATOR_STATUS_H
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_H


//----------------------------------------
//-- Message ILLUMINATOR_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_illuminator_status_t {
    uint32_t uptime_ms;
    uint32_t error_status;
    float brightness;
    float strobe_period;
    float strobe_duty_cycle;
    float temp_c;
    float min_strobe_period;
    float max_strobe_period;
    uint8_t enable;
    uint8_t mode_bitmask;
    uint8_t mode;
}) fmav_illuminator_status_t;


#define FASTMAVLINK_MSG_ID_ILLUMINATOR_STATUS  440

#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_PAYLOAD_LEN_MAX  35
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_CRCEXTRA  66

#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FRAME_LEN_MAX  60



#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_UPTIME_MS_OFS  0
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_ERROR_STATUS_OFS  4
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_BRIGHTNESS_OFS  8
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_STROBE_PERIOD_OFS  12
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_STROBE_DUTY_CYCLE_OFS  16
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_TEMP_C_OFS  20
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_MIN_STROBE_PERIOD_OFS  24
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_MAX_STROBE_PERIOD_OFS  28
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_ENABLE_OFS  32
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_MODE_BITMASK_OFS  33
#define FASTMAVLINK_MSG_ILLUMINATOR_STATUS_FIELD_MODE_OFS  34


//----------------------------------------
//-- Message ILLUMINATOR_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_illuminator_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t uptime_ms, uint8_t enable, uint8_t mode_bitmask, uint32_t error_status, uint8_t mode, float brightness, float strobe_period, float strobe_duty_cycle, float temp_c, float min_strobe_period, float max_strobe_period,
    fmav_status_t* _status)
{
    fmav_illuminator_status_t* _payload = (fmav_illuminator_status_t*)_msg->payload;

    _payload->uptime_ms = uptime_ms;
    _payload->error_status = error_status;
    _payload->brightness = brightness;
    _payload->strobe_period = strobe_period;
    _payload->strobe_duty_cycle = strobe_duty_cycle;
    _payload->temp_c = temp_c;
    _payload->min_strobe_period = min_strobe_period;
    _payload->max_strobe_period = max_strobe_period;
    _payload->enable = enable;
    _payload->mode_bitmask = mode_bitmask;
    _payload->mode = mode;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ILLUMINATOR_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ILLUMINATOR_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ILLUMINATOR_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_illuminator_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_illuminator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_illuminator_status_pack(
        _msg, sysid, compid,
        _payload->uptime_ms, _payload->enable, _payload->mode_bitmask, _payload->error_status, _payload->mode, _payload->brightness, _payload->strobe_period, _payload->strobe_duty_cycle, _payload->temp_c, _payload->min_strobe_period, _payload->max_strobe_period,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_illuminator_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t uptime_ms, uint8_t enable, uint8_t mode_bitmask, uint32_t error_status, uint8_t mode, float brightness, float strobe_period, float strobe_duty_cycle, float temp_c, float min_strobe_period, float max_strobe_period,
    fmav_status_t* _status)
{
    fmav_illuminator_status_t* _payload = (fmav_illuminator_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->uptime_ms = uptime_ms;
    _payload->error_status = error_status;
    _payload->brightness = brightness;
    _payload->strobe_period = strobe_period;
    _payload->strobe_duty_cycle = strobe_duty_cycle;
    _payload->temp_c = temp_c;
    _payload->min_strobe_period = min_strobe_period;
    _payload->max_strobe_period = max_strobe_period;
    _payload->enable = enable;
    _payload->mode_bitmask = mode_bitmask;
    _payload->mode = mode;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ILLUMINATOR_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ILLUMINATOR_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ILLUMINATOR_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ILLUMINATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ILLUMINATOR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_illuminator_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_illuminator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_illuminator_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->uptime_ms, _payload->enable, _payload->mode_bitmask, _payload->error_status, _payload->mode, _payload->brightness, _payload->strobe_period, _payload->strobe_duty_cycle, _payload->temp_c, _payload->min_strobe_period, _payload->max_strobe_period,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_illuminator_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t uptime_ms, uint8_t enable, uint8_t mode_bitmask, uint32_t error_status, uint8_t mode, float brightness, float strobe_period, float strobe_duty_cycle, float temp_c, float min_strobe_period, float max_strobe_period,
    fmav_status_t* _status)
{
    fmav_illuminator_status_t _payload;

    _payload.uptime_ms = uptime_ms;
    _payload.error_status = error_status;
    _payload.brightness = brightness;
    _payload.strobe_period = strobe_period;
    _payload.strobe_duty_cycle = strobe_duty_cycle;
    _payload.temp_c = temp_c;
    _payload.min_strobe_period = min_strobe_period;
    _payload.max_strobe_period = max_strobe_period;
    _payload.enable = enable;
    _payload.mode_bitmask = mode_bitmask;
    _payload.mode = mode;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ILLUMINATOR_STATUS,
        FASTMAVLINK_MSG_ILLUMINATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ILLUMINATOR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_illuminator_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_illuminator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ILLUMINATOR_STATUS,
        FASTMAVLINK_MSG_ILLUMINATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ILLUMINATOR_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ILLUMINATOR_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_illuminator_status_decode(fmav_illuminator_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ILLUMINATOR_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ILLUMINATOR_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ILLUMINATOR_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ILLUMINATOR_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_illuminator_status_get_field_uptime_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_illuminator_status_get_field_error_status(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_illuminator_status_get_field_brightness(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_illuminator_status_get_field_strobe_period(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_illuminator_status_get_field_strobe_duty_cycle(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_illuminator_status_get_field_temp_c(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_illuminator_status_get_field_min_strobe_period(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_illuminator_status_get_field_max_strobe_period(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_illuminator_status_get_field_enable(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_illuminator_status_get_field_mode_bitmask(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[33]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_illuminator_status_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ILLUMINATOR_STATUS  440

#define mavlink_illuminator_status_t  fmav_illuminator_status_t

#define MAVLINK_MSG_ID_ILLUMINATOR_STATUS_LEN  35
#define MAVLINK_MSG_ID_ILLUMINATOR_STATUS_MIN_LEN  35
#define MAVLINK_MSG_ID_440_LEN  35
#define MAVLINK_MSG_ID_440_MIN_LEN  35

#define MAVLINK_MSG_ID_ILLUMINATOR_STATUS_CRC  66
#define MAVLINK_MSG_ID_440_CRC  66




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_illuminator_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t uptime_ms, uint8_t enable, uint8_t mode_bitmask, uint32_t error_status, uint8_t mode, float brightness, float strobe_period, float strobe_duty_cycle, float temp_c, float min_strobe_period, float max_strobe_period)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_illuminator_status_pack(
        _msg, sysid, compid,
        uptime_ms, enable, mode_bitmask, error_status, mode, brightness, strobe_period, strobe_duty_cycle, temp_c, min_strobe_period, max_strobe_period,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_illuminator_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_illuminator_status_t* _payload)
{
    return mavlink_msg_illuminator_status_pack(
        sysid,
        compid,
        _msg,
        _payload->uptime_ms, _payload->enable, _payload->mode_bitmask, _payload->error_status, _payload->mode, _payload->brightness, _payload->strobe_period, _payload->strobe_duty_cycle, _payload->temp_c, _payload->min_strobe_period, _payload->max_strobe_period);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_illuminator_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t uptime_ms, uint8_t enable, uint8_t mode_bitmask, uint32_t error_status, uint8_t mode, float brightness, float strobe_period, float strobe_duty_cycle, float temp_c, float min_strobe_period, float max_strobe_period)
{
    return fmav_msg_illuminator_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        uptime_ms, enable, mode_bitmask, error_status, mode, brightness, strobe_period, strobe_duty_cycle, temp_c, min_strobe_period, max_strobe_period,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_illuminator_status_decode(const mavlink_message_t* msg, mavlink_illuminator_status_t* payload)
{
    fmav_msg_illuminator_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ILLUMINATOR_STATUS_H
