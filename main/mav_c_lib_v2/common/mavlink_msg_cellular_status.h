//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CELLULAR_STATUS_H
#define FASTMAVLINK_MSG_CELLULAR_STATUS_H


//----------------------------------------
//-- Message CELLULAR_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_cellular_status_t {
    uint16_t mcc;
    uint16_t mnc;
    uint16_t lac;
    uint8_t status;
    uint8_t failure_reason;
    uint8_t type;
    uint8_t quality;
}) fmav_cellular_status_t;


#define FASTMAVLINK_MSG_ID_CELLULAR_STATUS  334

#define FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX  10
#define FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA  72

#define FASTMAVLINK_MSG_CELLULAR_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CELLULAR_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CELLULAR_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CELLULAR_STATUS_FRAME_LEN_MAX  35



#define FASTMAVLINK_MSG_CELLULAR_STATUS_FIELD_MCC_OFS  0
#define FASTMAVLINK_MSG_CELLULAR_STATUS_FIELD_MNC_OFS  2
#define FASTMAVLINK_MSG_CELLULAR_STATUS_FIELD_LAC_OFS  4
#define FASTMAVLINK_MSG_CELLULAR_STATUS_FIELD_STATUS_OFS  6
#define FASTMAVLINK_MSG_CELLULAR_STATUS_FIELD_FAILURE_REASON_OFS  7
#define FASTMAVLINK_MSG_CELLULAR_STATUS_FIELD_TYPE_OFS  8
#define FASTMAVLINK_MSG_CELLULAR_STATUS_FIELD_QUALITY_OFS  9


//----------------------------------------
//-- Message CELLULAR_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac,
    fmav_status_t* _status)
{
    fmav_cellular_status_t* _payload = (fmav_cellular_status_t*)_msg->payload;

    _payload->mcc = mcc;
    _payload->mnc = mnc;
    _payload->lac = lac;
    _payload->status = status;
    _payload->failure_reason = failure_reason;
    _payload->type = type;
    _payload->quality = quality;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CELLULAR_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cellular_status_pack(
        _msg, sysid, compid,
        _payload->status, _payload->failure_reason, _payload->type, _payload->quality, _payload->mcc, _payload->mnc, _payload->lac,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac,
    fmav_status_t* _status)
{
    fmav_cellular_status_t* _payload = (fmav_cellular_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mcc = mcc;
    _payload->mnc = mnc;
    _payload->lac = lac;
    _payload->status = status;
    _payload->failure_reason = failure_reason;
    _payload->type = type;
    _payload->quality = quality;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CELLULAR_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CELLULAR_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CELLULAR_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cellular_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->status, _payload->failure_reason, _payload->type, _payload->quality, _payload->mcc, _payload->mnc, _payload->lac,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac,
    fmav_status_t* _status)
{
    fmav_cellular_status_t _payload;

    _payload.mcc = mcc;
    _payload.mnc = mnc;
    _payload.lac = lac;
    _payload.status = status;
    _payload.failure_reason = failure_reason;
    _payload.type = type;
    _payload.quality = quality;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CELLULAR_STATUS,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CELLULAR_STATUS,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CELLULAR_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_cellular_status_decode(fmav_cellular_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_get_field_mcc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_get_field_mnc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_get_field_lac(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_status_get_field_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_status_get_field_failure_reason(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_status_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_status_get_field_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CELLULAR_STATUS  334

#define mavlink_cellular_status_t  fmav_cellular_status_t

#define MAVLINK_MSG_ID_CELLULAR_STATUS_LEN  10
#define MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN  10
#define MAVLINK_MSG_ID_334_LEN  10
#define MAVLINK_MSG_ID_334_MIN_LEN  10

#define MAVLINK_MSG_ID_CELLULAR_STATUS_CRC  72
#define MAVLINK_MSG_ID_334_CRC  72




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_cellular_status_pack(
        _msg, sysid, compid,
        status, failure_reason, type, quality, mcc, mnc, lac,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_cellular_status_t* _payload)
{
    return mavlink_msg_cellular_status_pack(
        sysid,
        compid,
        _msg,
        _payload->status, _payload->failure_reason, _payload->type, _payload->quality, _payload->mcc, _payload->mnc, _payload->lac);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac)
{
    return fmav_msg_cellular_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        status, failure_reason, type, quality, mcc, mnc, lac,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_cellular_status_decode(const mavlink_message_t* msg, mavlink_cellular_status_t* payload)
{
    fmav_msg_cellular_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CELLULAR_STATUS_H
