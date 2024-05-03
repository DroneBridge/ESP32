//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ISBD_LINK_STATUS_H
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_H


//----------------------------------------
//-- Message ISBD_LINK_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_isbd_link_status_t {
    uint64_t timestamp;
    uint64_t last_heartbeat;
    uint16_t failed_sessions;
    uint16_t successful_sessions;
    uint8_t signal_quality;
    uint8_t ring_pending;
    uint8_t tx_session_pending;
    uint8_t rx_session_pending;
}) fmav_isbd_link_status_t;


#define FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS  335

#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA  225

#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FRAME_LEN_MAX  49



#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FIELD_LAST_HEARTBEAT_OFS  8
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FIELD_FAILED_SESSIONS_OFS  16
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FIELD_SUCCESSFUL_SESSIONS_OFS  18
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FIELD_SIGNAL_QUALITY_OFS  20
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FIELD_RING_PENDING_OFS  21
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FIELD_TX_SESSION_PENDING_OFS  22
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FIELD_RX_SESSION_PENDING_OFS  23


//----------------------------------------
//-- Message ISBD_LINK_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending,
    fmav_status_t* _status)
{
    fmav_isbd_link_status_t* _payload = (fmav_isbd_link_status_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->last_heartbeat = last_heartbeat;
    _payload->failed_sessions = failed_sessions;
    _payload->successful_sessions = successful_sessions;
    _payload->signal_quality = signal_quality;
    _payload->ring_pending = ring_pending;
    _payload->tx_session_pending = tx_session_pending;
    _payload->rx_session_pending = rx_session_pending;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_isbd_link_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_isbd_link_status_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->last_heartbeat, _payload->failed_sessions, _payload->successful_sessions, _payload->signal_quality, _payload->ring_pending, _payload->tx_session_pending, _payload->rx_session_pending,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending,
    fmav_status_t* _status)
{
    fmav_isbd_link_status_t* _payload = (fmav_isbd_link_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->last_heartbeat = last_heartbeat;
    _payload->failed_sessions = failed_sessions;
    _payload->successful_sessions = successful_sessions;
    _payload->signal_quality = signal_quality;
    _payload->ring_pending = ring_pending;
    _payload->tx_session_pending = tx_session_pending;
    _payload->rx_session_pending = rx_session_pending;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_isbd_link_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_isbd_link_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->last_heartbeat, _payload->failed_sessions, _payload->successful_sessions, _payload->signal_quality, _payload->ring_pending, _payload->tx_session_pending, _payload->rx_session_pending,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending,
    fmav_status_t* _status)
{
    fmav_isbd_link_status_t _payload;

    _payload.timestamp = timestamp;
    _payload.last_heartbeat = last_heartbeat;
    _payload.failed_sessions = failed_sessions;
    _payload.successful_sessions = successful_sessions;
    _payload.signal_quality = signal_quality;
    _payload.ring_pending = ring_pending;
    _payload.tx_session_pending = tx_session_pending;
    _payload.rx_session_pending = rx_session_pending;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_isbd_link_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ISBD_LINK_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_isbd_link_status_decode(fmav_isbd_link_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_isbd_link_status_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_isbd_link_status_get_field_last_heartbeat(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_get_field_failed_sessions(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_get_field_successful_sessions(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_isbd_link_status_get_field_signal_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_isbd_link_status_get_field_ring_pending(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[21]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_isbd_link_status_get_field_tx_session_pending(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_isbd_link_status_get_field_rx_session_pending(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[23]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ISBD_LINK_STATUS  335

#define mavlink_isbd_link_status_t  fmav_isbd_link_status_t

#define MAVLINK_MSG_ID_ISBD_LINK_STATUS_LEN  24
#define MAVLINK_MSG_ID_ISBD_LINK_STATUS_MIN_LEN  24
#define MAVLINK_MSG_ID_335_LEN  24
#define MAVLINK_MSG_ID_335_MIN_LEN  24

#define MAVLINK_MSG_ID_ISBD_LINK_STATUS_CRC  225
#define MAVLINK_MSG_ID_335_CRC  225




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_isbd_link_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_isbd_link_status_pack(
        _msg, sysid, compid,
        timestamp, last_heartbeat, failed_sessions, successful_sessions, signal_quality, ring_pending, tx_session_pending, rx_session_pending,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_isbd_link_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_isbd_link_status_t* _payload)
{
    return mavlink_msg_isbd_link_status_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->last_heartbeat, _payload->failed_sessions, _payload->successful_sessions, _payload->signal_quality, _payload->ring_pending, _payload->tx_session_pending, _payload->rx_session_pending);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_isbd_link_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending)
{
    return fmav_msg_isbd_link_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, last_heartbeat, failed_sessions, successful_sessions, signal_quality, ring_pending, tx_session_pending, rx_session_pending,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_isbd_link_status_decode(const mavlink_message_t* msg, mavlink_isbd_link_status_t* payload)
{
    fmav_msg_isbd_link_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ISBD_LINK_STATUS_H
