//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RC_CHANNELS_H
#define FASTMAVLINK_MSG_RC_CHANNELS_H


//----------------------------------------
//-- Message RC_CHANNELS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_rc_channels_t {
    uint32_t time_boot_ms;
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
    uint16_t chan13_raw;
    uint16_t chan14_raw;
    uint16_t chan15_raw;
    uint16_t chan16_raw;
    uint16_t chan17_raw;
    uint16_t chan18_raw;
    uint8_t chancount;
    uint8_t rssi;
}) fmav_rc_channels_t;


#define FASTMAVLINK_MSG_ID_RC_CHANNELS  65

#define FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_RC_CHANNELS_CRCEXTRA  118

#define FASTMAVLINK_MSG_RC_CHANNELS_FLAGS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RC_CHANNELS_FRAME_LEN_MAX  67



#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN1_RAW_OFS  4
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN2_RAW_OFS  6
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN3_RAW_OFS  8
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN4_RAW_OFS  10
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN5_RAW_OFS  12
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN6_RAW_OFS  14
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN7_RAW_OFS  16
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN8_RAW_OFS  18
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN9_RAW_OFS  20
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN10_RAW_OFS  22
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN11_RAW_OFS  24
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN12_RAW_OFS  26
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN13_RAW_OFS  28
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN14_RAW_OFS  30
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN15_RAW_OFS  32
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN16_RAW_OFS  34
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN17_RAW_OFS  36
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHAN18_RAW_OFS  38
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_CHANCOUNT_OFS  40
#define FASTMAVLINK_MSG_RC_CHANNELS_FIELD_RSSI_OFS  41


//----------------------------------------
//-- Message RC_CHANNELS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t chancount, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_rc_channels_t* _payload = (fmav_rc_channels_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
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
    _payload->chan13_raw = chan13_raw;
    _payload->chan14_raw = chan14_raw;
    _payload->chan15_raw = chan15_raw;
    _payload->chan16_raw = chan16_raw;
    _payload->chan17_raw = chan17_raw;
    _payload->chan18_raw = chan18_raw;
    _payload->chancount = chancount;
    _payload->rssi = rssi;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RC_CHANNELS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_RC_CHANNELS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rc_channels_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->chancount, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->chan13_raw, _payload->chan14_raw, _payload->chan15_raw, _payload->chan16_raw, _payload->chan17_raw, _payload->chan18_raw, _payload->rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t chancount, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_rc_channels_t* _payload = (fmav_rc_channels_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
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
    _payload->chan13_raw = chan13_raw;
    _payload->chan14_raw = chan14_raw;
    _payload->chan15_raw = chan15_raw;
    _payload->chan16_raw = chan16_raw;
    _payload->chan17_raw = chan17_raw;
    _payload->chan18_raw = chan18_raw;
    _payload->chancount = chancount;
    _payload->rssi = rssi;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RC_CHANNELS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RC_CHANNELS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RC_CHANNELS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rc_channels_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->chancount, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->chan13_raw, _payload->chan14_raw, _payload->chan15_raw, _payload->chan16_raw, _payload->chan17_raw, _payload->chan18_raw, _payload->rssi,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t chancount, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_rc_channels_t _payload;

    _payload.time_boot_ms = time_boot_ms;
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
    _payload.chan13_raw = chan13_raw;
    _payload.chan14_raw = chan14_raw;
    _payload.chan15_raw = chan15_raw;
    _payload.chan16_raw = chan16_raw;
    _payload.chan17_raw = chan17_raw;
    _payload.chan18_raw = chan18_raw;
    _payload.chancount = chancount;
    _payload.rssi = rssi;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RC_CHANNELS,
        FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RC_CHANNELS,
        FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RC_CHANNELS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rc_channels_decode(fmav_rc_channels_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RC_CHANNELS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_rc_channels_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan1_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan2_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan3_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan4_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan5_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan6_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan7_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan8_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan9_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan10_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan11_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan12_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan13_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan14_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan15_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan16_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan17_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_get_field_chan18_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rc_channels_get_field_chancount(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rc_channels_get_field_rssi(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[41]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RC_CHANNELS  65

#define mavlink_rc_channels_t  fmav_rc_channels_t

#define MAVLINK_MSG_ID_RC_CHANNELS_LEN  42
#define MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN  42
#define MAVLINK_MSG_ID_65_LEN  42
#define MAVLINK_MSG_ID_65_MIN_LEN  42

#define MAVLINK_MSG_ID_RC_CHANNELS_CRC  118
#define MAVLINK_MSG_ID_65_CRC  118




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t chancount, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw, uint8_t rssi)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_rc_channels_pack(
        _msg, sysid, compid,
        time_boot_ms, chancount, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, chan13_raw, chan14_raw, chan15_raw, chan16_raw, chan17_raw, chan18_raw, rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_rc_channels_t* _payload)
{
    return mavlink_msg_rc_channels_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->chancount, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->chan13_raw, _payload->chan14_raw, _payload->chan15_raw, _payload->chan16_raw, _payload->chan17_raw, _payload->chan18_raw, _payload->rssi);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t chancount, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw, uint8_t rssi)
{
    return fmav_msg_rc_channels_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, chancount, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, chan13_raw, chan14_raw, chan15_raw, chan16_raw, chan17_raw, chan18_raw, rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_rc_channels_decode(const mavlink_message_t* msg, mavlink_rc_channels_t* payload)
{
    fmav_msg_rc_channels_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RC_CHANNELS_H
