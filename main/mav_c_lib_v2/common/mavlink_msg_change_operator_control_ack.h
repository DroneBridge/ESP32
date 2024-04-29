//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_H
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_H


//----------------------------------------
//-- Message CHANGE_OPERATOR_CONTROL_ACK
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_change_operator_control_ack_t {
    uint8_t gcs_system_id;
    uint8_t control_request;
    uint8_t ack;
}) fmav_change_operator_control_ack_t;


#define FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK  6

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX  3
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA  104

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FLAGS  0
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FRAME_LEN_MAX  28



#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FIELD_GCS_SYSTEM_ID_OFS  0
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FIELD_CONTROL_REQUEST_OFS  1
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FIELD_ACK_OFS  2


//----------------------------------------
//-- Message CHANGE_OPERATOR_CONTROL_ACK pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack,
    fmav_status_t* _status)
{
    fmav_change_operator_control_ack_t* _payload = (fmav_change_operator_control_ack_t*)_msg->payload;

    _payload->gcs_system_id = gcs_system_id;
    _payload->control_request = control_request;
    _payload->ack = ack;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_change_operator_control_ack_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_change_operator_control_ack_pack(
        _msg, sysid, compid,
        _payload->gcs_system_id, _payload->control_request, _payload->ack,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack,
    fmav_status_t* _status)
{
    fmav_change_operator_control_ack_t* _payload = (fmav_change_operator_control_ack_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->gcs_system_id = gcs_system_id;
    _payload->control_request = control_request;
    _payload->ack = ack;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_change_operator_control_ack_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_change_operator_control_ack_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->gcs_system_id, _payload->control_request, _payload->ack,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack,
    fmav_status_t* _status)
{
    fmav_change_operator_control_ack_t _payload;

    _payload.gcs_system_id = gcs_system_id;
    _payload.control_request = control_request;
    _payload.ack = ack;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_change_operator_control_ack_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CHANGE_OPERATOR_CONTROL_ACK decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_change_operator_control_ack_decode(fmav_change_operator_control_ack_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_change_operator_control_ack_get_field_gcs_system_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_change_operator_control_ack_get_field_control_request(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_change_operator_control_ack_get_field_ack(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK  6

#define mavlink_change_operator_control_ack_t  fmav_change_operator_control_ack_t

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_LEN  3
#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_MIN_LEN  3
#define MAVLINK_MSG_ID_6_LEN  3
#define MAVLINK_MSG_ID_6_MIN_LEN  3

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_CRC  104
#define MAVLINK_MSG_ID_6_CRC  104




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_change_operator_control_ack_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_change_operator_control_ack_pack(
        _msg, sysid, compid,
        gcs_system_id, control_request, ack,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_change_operator_control_ack_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_change_operator_control_ack_t* _payload)
{
    return mavlink_msg_change_operator_control_ack_pack(
        sysid,
        compid,
        _msg,
        _payload->gcs_system_id, _payload->control_request, _payload->ack);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_change_operator_control_ack_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack)
{
    return fmav_msg_change_operator_control_ack_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        gcs_system_id, control_request, ack,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_change_operator_control_ack_decode(const mavlink_message_t* msg, mavlink_change_operator_control_ack_t* payload)
{
    fmav_msg_change_operator_control_ack_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_H
