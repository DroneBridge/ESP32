//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_H
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_H


//----------------------------------------
//-- Message HIL_RC_INPUTS_RAW
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_rc_inputs_raw_t {
    uint64_t time_usec;
    uint16_t chan1_raw;
    uint16_t chan2_raw;
    uint16_t chan3_raw;
    uint16_t chan4_raw;
    uint16_t chan5_raw;
    uint16_t chan6_raw;
    uint16_t chan7_raw;
    uint16_t chan8_raw;
    uint16_t chan9_raw;
    uint16_t chan10_raw;
    uint16_t chan11_raw;
    uint16_t chan12_raw;
    uint8_t rssi;
}) fmav_hil_rc_inputs_raw_t;


#define FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW  92

#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_CRCEXTRA  54

#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FLAGS  0
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FRAME_LEN_MAX  58



#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN1_RAW_OFS  8
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN2_RAW_OFS  10
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN3_RAW_OFS  12
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN4_RAW_OFS  14
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN5_RAW_OFS  16
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN6_RAW_OFS  18
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN7_RAW_OFS  20
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN8_RAW_OFS  22
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN9_RAW_OFS  24
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN10_RAW_OFS  26
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN11_RAW_OFS  28
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_CHAN12_RAW_OFS  30
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FIELD_RSSI_OFS  32


//----------------------------------------
//-- Message HIL_RC_INPUTS_RAW pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_hil_rc_inputs_raw_t* _payload = (fmav_hil_rc_inputs_raw_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->chan1_raw = chan1_raw;
    _payload->chan2_raw = chan2_raw;
    _payload->chan3_raw = chan3_raw;
    _payload->chan4_raw = chan4_raw;
    _payload->chan5_raw = chan5_raw;
    _payload->chan6_raw = chan6_raw;
    _payload->chan7_raw = chan7_raw;
    _payload->chan8_raw = chan8_raw;
    _payload->chan9_raw = chan9_raw;
    _payload->chan10_raw = chan10_raw;
    _payload->chan11_raw = chan11_raw;
    _payload->chan12_raw = chan12_raw;
    _payload->rssi = rssi;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_rc_inputs_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_rc_inputs_raw_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_hil_rc_inputs_raw_t* _payload = (fmav_hil_rc_inputs_raw_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->chan1_raw = chan1_raw;
    _payload->chan2_raw = chan2_raw;
    _payload->chan3_raw = chan3_raw;
    _payload->chan4_raw = chan4_raw;
    _payload->chan5_raw = chan5_raw;
    _payload->chan6_raw = chan6_raw;
    _payload->chan7_raw = chan7_raw;
    _payload->chan8_raw = chan8_raw;
    _payload->chan9_raw = chan9_raw;
    _payload->chan10_raw = chan10_raw;
    _payload->chan11_raw = chan11_raw;
    _payload->chan12_raw = chan12_raw;
    _payload->rssi = rssi;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_rc_inputs_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_rc_inputs_raw_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->rssi,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_hil_rc_inputs_raw_t _payload;

    _payload.time_usec = time_usec;
    _payload.chan1_raw = chan1_raw;
    _payload.chan2_raw = chan2_raw;
    _payload.chan3_raw = chan3_raw;
    _payload.chan4_raw = chan4_raw;
    _payload.chan5_raw = chan5_raw;
    _payload.chan6_raw = chan6_raw;
    _payload.chan7_raw = chan7_raw;
    _payload.chan8_raw = chan8_raw;
    _payload.chan9_raw = chan9_raw;
    _payload.chan10_raw = chan10_raw;
    _payload.chan11_raw = chan11_raw;
    _payload.chan12_raw = chan12_raw;
    _payload.rssi = rssi;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_rc_inputs_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIL_RC_INPUTS_RAW decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_rc_inputs_raw_decode(fmav_hil_rc_inputs_raw_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_rc_inputs_raw_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan1_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan2_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan3_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan4_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan5_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan6_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan7_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan8_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan9_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan10_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan11_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_get_field_chan12_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_rc_inputs_raw_get_field_rssi(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW  92

#define mavlink_hil_rc_inputs_raw_t  fmav_hil_rc_inputs_raw_t

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN  33
#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_MIN_LEN  33
#define MAVLINK_MSG_ID_92_LEN  33
#define MAVLINK_MSG_ID_92_MIN_LEN  33

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_CRC  54
#define MAVLINK_MSG_ID_92_CRC  54




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_rc_inputs_raw_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_rc_inputs_raw_pack(
        _msg, sysid, compid,
        time_usec, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_rc_inputs_raw_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_hil_rc_inputs_raw_t* _payload)
{
    return mavlink_msg_hil_rc_inputs_raw_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->rssi);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_rc_inputs_raw_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi)
{
    return fmav_msg_hil_rc_inputs_raw_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_rc_inputs_raw_decode(const mavlink_message_t* msg, mavlink_hil_rc_inputs_raw_t* payload)
{
    fmav_msg_hil_rc_inputs_raw_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_H
