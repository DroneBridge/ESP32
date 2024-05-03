//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RC_CHANNELS_SCALED_H
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_H


//----------------------------------------
//-- Message RC_CHANNELS_SCALED
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_rc_channels_scaled_t {
    uint32_t time_boot_ms;
    int16_t chan1_scaled;
    int16_t chan2_scaled;
    int16_t chan3_scaled;
    int16_t chan4_scaled;
    int16_t chan5_scaled;
    int16_t chan6_scaled;
    int16_t chan7_scaled;
    int16_t chan8_scaled;
    uint8_t port;
    uint8_t rssi;
}) fmav_rc_channels_scaled_t;


#define FASTMAVLINK_MSG_ID_RC_CHANNELS_SCALED  34

#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_CRCEXTRA  237

#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FLAGS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FRAME_LEN_MAX  47



#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_CHAN1_SCALED_OFS  4
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_CHAN2_SCALED_OFS  6
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_CHAN3_SCALED_OFS  8
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_CHAN4_SCALED_OFS  10
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_CHAN5_SCALED_OFS  12
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_CHAN6_SCALED_OFS  14
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_CHAN7_SCALED_OFS  16
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_CHAN8_SCALED_OFS  18
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_PORT_OFS  20
#define FASTMAVLINK_MSG_RC_CHANNELS_SCALED_FIELD_RSSI_OFS  21


//----------------------------------------
//-- Message RC_CHANNELS_SCALED pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_scaled_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t port, int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled, int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled, int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_rc_channels_scaled_t* _payload = (fmav_rc_channels_scaled_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->chan1_scaled = chan1_scaled;
    _payload->chan2_scaled = chan2_scaled;
    _payload->chan3_scaled = chan3_scaled;
    _payload->chan4_scaled = chan4_scaled;
    _payload->chan5_scaled = chan5_scaled;
    _payload->chan6_scaled = chan6_scaled;
    _payload->chan7_scaled = chan7_scaled;
    _payload->chan8_scaled = chan8_scaled;
    _payload->port = port;
    _payload->rssi = rssi;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RC_CHANNELS_SCALED;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_RC_CHANNELS_SCALED_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_scaled_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_scaled_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rc_channels_scaled_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->port, _payload->chan1_scaled, _payload->chan2_scaled, _payload->chan3_scaled, _payload->chan4_scaled, _payload->chan5_scaled, _payload->chan6_scaled, _payload->chan7_scaled, _payload->chan8_scaled, _payload->rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_scaled_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t port, int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled, int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled, int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_rc_channels_scaled_t* _payload = (fmav_rc_channels_scaled_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->chan1_scaled = chan1_scaled;
    _payload->chan2_scaled = chan2_scaled;
    _payload->chan3_scaled = chan3_scaled;
    _payload->chan4_scaled = chan4_scaled;
    _payload->chan5_scaled = chan5_scaled;
    _payload->chan6_scaled = chan6_scaled;
    _payload->chan7_scaled = chan7_scaled;
    _payload->chan8_scaled = chan8_scaled;
    _payload->port = port;
    _payload->rssi = rssi;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RC_CHANNELS_SCALED;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RC_CHANNELS_SCALED >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RC_CHANNELS_SCALED >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_SCALED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_scaled_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_scaled_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rc_channels_scaled_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->port, _payload->chan1_scaled, _payload->chan2_scaled, _payload->chan3_scaled, _payload->chan4_scaled, _payload->chan5_scaled, _payload->chan6_scaled, _payload->chan7_scaled, _payload->chan8_scaled, _payload->rssi,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_scaled_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t port, int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled, int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled, int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_rc_channels_scaled_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.chan1_scaled = chan1_scaled;
    _payload.chan2_scaled = chan2_scaled;
    _payload.chan3_scaled = chan3_scaled;
    _payload.chan4_scaled = chan4_scaled;
    _payload.chan5_scaled = chan5_scaled;
    _payload.chan6_scaled = chan6_scaled;
    _payload.chan7_scaled = chan7_scaled;
    _payload.chan8_scaled = chan8_scaled;
    _payload.port = port;
    _payload.rssi = rssi;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RC_CHANNELS_SCALED,
        FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_SCALED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_scaled_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_scaled_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RC_CHANNELS_SCALED,
        FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_SCALED_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RC_CHANNELS_SCALED decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rc_channels_scaled_decode(fmav_rc_channels_scaled_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RC_CHANNELS_SCALED_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_rc_channels_scaled_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rc_channels_scaled_get_field_chan1_scaled(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rc_channels_scaled_get_field_chan2_scaled(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rc_channels_scaled_get_field_chan3_scaled(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rc_channels_scaled_get_field_chan4_scaled(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rc_channels_scaled_get_field_chan5_scaled(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rc_channels_scaled_get_field_chan6_scaled(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rc_channels_scaled_get_field_chan7_scaled(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rc_channels_scaled_get_field_chan8_scaled(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rc_channels_scaled_get_field_port(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rc_channels_scaled_get_field_rssi(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[21]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED  34

#define mavlink_rc_channels_scaled_t  fmav_rc_channels_scaled_t

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN  22
#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED_MIN_LEN  22
#define MAVLINK_MSG_ID_34_LEN  22
#define MAVLINK_MSG_ID_34_MIN_LEN  22

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED_CRC  237
#define MAVLINK_MSG_ID_34_CRC  237




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_scaled_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t port, int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled, int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled, int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_rc_channels_scaled_pack(
        _msg, sysid, compid,
        time_boot_ms, port, chan1_scaled, chan2_scaled, chan3_scaled, chan4_scaled, chan5_scaled, chan6_scaled, chan7_scaled, chan8_scaled, rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_scaled_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_rc_channels_scaled_t* _payload)
{
    return mavlink_msg_rc_channels_scaled_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->port, _payload->chan1_scaled, _payload->chan2_scaled, _payload->chan3_scaled, _payload->chan4_scaled, _payload->chan5_scaled, _payload->chan6_scaled, _payload->chan7_scaled, _payload->chan8_scaled, _payload->rssi);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_scaled_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t port, int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled, int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled, int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi)
{
    return fmav_msg_rc_channels_scaled_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, port, chan1_scaled, chan2_scaled, chan3_scaled, chan4_scaled, chan5_scaled, chan6_scaled, chan7_scaled, chan8_scaled, rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_rc_channels_scaled_decode(const mavlink_message_t* msg, mavlink_rc_channels_scaled_t* payload)
{
    fmav_msg_rc_channels_scaled_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RC_CHANNELS_SCALED_H
