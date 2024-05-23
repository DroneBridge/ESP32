//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RAW_RPM_H
#define FASTMAVLINK_MSG_RAW_RPM_H


//----------------------------------------
//-- Message RAW_RPM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_raw_rpm_t {
    float frequency;
    uint8_t index;
}) fmav_raw_rpm_t;


#define FASTMAVLINK_MSG_ID_RAW_RPM  339

#define FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX  5
#define FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA  199

#define FASTMAVLINK_MSG_RAW_RPM_FLAGS  0
#define FASTMAVLINK_MSG_RAW_RPM_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RAW_RPM_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RAW_RPM_FRAME_LEN_MAX  30



#define FASTMAVLINK_MSG_RAW_RPM_FIELD_FREQUENCY_OFS  0
#define FASTMAVLINK_MSG_RAW_RPM_FIELD_INDEX_OFS  4


//----------------------------------------
//-- Message RAW_RPM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, float frequency,
    fmav_status_t* _status)
{
    fmav_raw_rpm_t* _payload = (fmav_raw_rpm_t*)_msg->payload;

    _payload->frequency = frequency;
    _payload->index = index;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RAW_RPM;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_raw_rpm_pack(
        _msg, sysid, compid,
        _payload->index, _payload->frequency,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, float frequency,
    fmav_status_t* _status)
{
    fmav_raw_rpm_t* _payload = (fmav_raw_rpm_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->frequency = frequency;
    _payload->index = index;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RAW_RPM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RAW_RPM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RAW_RPM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_raw_rpm_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->index, _payload->frequency,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, float frequency,
    fmav_status_t* _status)
{
    fmav_raw_rpm_t _payload;

    _payload.frequency = frequency;
    _payload.index = index;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RAW_RPM,
        FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RAW_RPM,
        FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RAW_RPM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_raw_rpm_decode(fmav_raw_rpm_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_raw_rpm_get_field_frequency(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_raw_rpm_get_field_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RAW_RPM  339

#define mavlink_raw_rpm_t  fmav_raw_rpm_t

#define MAVLINK_MSG_ID_RAW_RPM_LEN  5
#define MAVLINK_MSG_ID_RAW_RPM_MIN_LEN  5
#define MAVLINK_MSG_ID_339_LEN  5
#define MAVLINK_MSG_ID_339_MIN_LEN  5

#define MAVLINK_MSG_ID_RAW_RPM_CRC  199
#define MAVLINK_MSG_ID_339_CRC  199




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_raw_rpm_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t index, float frequency)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_raw_rpm_pack(
        _msg, sysid, compid,
        index, frequency,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_raw_rpm_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_raw_rpm_t* _payload)
{
    return mavlink_msg_raw_rpm_pack(
        sysid,
        compid,
        _msg,
        _payload->index, _payload->frequency);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_raw_rpm_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, float frequency)
{
    return fmav_msg_raw_rpm_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        index, frequency,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_raw_rpm_decode(const mavlink_message_t* msg, mavlink_raw_rpm_t* payload)
{
    fmav_msg_raw_rpm_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RAW_RPM_H
