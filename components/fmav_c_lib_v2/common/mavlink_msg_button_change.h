//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_BUTTON_CHANGE_H
#define FASTMAVLINK_MSG_BUTTON_CHANGE_H


//----------------------------------------
//-- Message BUTTON_CHANGE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_button_change_t {
    uint32_t time_boot_ms;
    uint32_t last_change_ms;
    uint8_t state;
}) fmav_button_change_t;


#define FASTMAVLINK_MSG_ID_BUTTON_CHANGE  257

#define FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX  9
#define FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA  131

#define FASTMAVLINK_MSG_BUTTON_CHANGE_FLAGS  0
#define FASTMAVLINK_MSG_BUTTON_CHANGE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_BUTTON_CHANGE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_BUTTON_CHANGE_FRAME_LEN_MAX  34



#define FASTMAVLINK_MSG_BUTTON_CHANGE_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_BUTTON_CHANGE_FIELD_LAST_CHANGE_MS_OFS  4
#define FASTMAVLINK_MSG_BUTTON_CHANGE_FIELD_STATE_OFS  8


//----------------------------------------
//-- Message BUTTON_CHANGE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state,
    fmav_status_t* _status)
{
    fmav_button_change_t* _payload = (fmav_button_change_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->last_change_ms = last_change_ms;
    _payload->state = state;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_BUTTON_CHANGE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_button_change_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_button_change_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->last_change_ms, _payload->state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state,
    fmav_status_t* _status)
{
    fmav_button_change_t* _payload = (fmav_button_change_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->last_change_ms = last_change_ms;
    _payload->state = state;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_BUTTON_CHANGE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_BUTTON_CHANGE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_BUTTON_CHANGE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_button_change_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_button_change_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->last_change_ms, _payload->state,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state,
    fmav_status_t* _status)
{
    fmav_button_change_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.last_change_ms = last_change_ms;
    _payload.state = state;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_BUTTON_CHANGE,
        FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_button_change_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_BUTTON_CHANGE,
        FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message BUTTON_CHANGE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_button_change_decode(fmav_button_change_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_button_change_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_button_change_get_field_last_change_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_button_change_get_field_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_BUTTON_CHANGE  257

#define mavlink_button_change_t  fmav_button_change_t

#define MAVLINK_MSG_ID_BUTTON_CHANGE_LEN  9
#define MAVLINK_MSG_ID_BUTTON_CHANGE_MIN_LEN  9
#define MAVLINK_MSG_ID_257_LEN  9
#define MAVLINK_MSG_ID_257_MIN_LEN  9

#define MAVLINK_MSG_ID_BUTTON_CHANGE_CRC  131
#define MAVLINK_MSG_ID_257_CRC  131




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_button_change_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_button_change_pack(
        _msg, sysid, compid,
        time_boot_ms, last_change_ms, state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_button_change_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_button_change_t* _payload)
{
    return mavlink_msg_button_change_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->last_change_ms, _payload->state);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_button_change_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state)
{
    return fmav_msg_button_change_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, last_change_ms, state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_button_change_decode(const mavlink_message_t* msg, mavlink_button_change_t* payload)
{
    fmav_msg_button_change_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_BUTTON_CHANGE_H
