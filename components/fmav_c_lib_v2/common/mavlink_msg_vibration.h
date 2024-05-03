//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VIBRATION_H
#define FASTMAVLINK_MSG_VIBRATION_H


//----------------------------------------
//-- Message VIBRATION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_vibration_t {
    uint64_t time_usec;
    float vibration_x;
    float vibration_y;
    float vibration_z;
    uint32_t clipping_0;
    uint32_t clipping_1;
    uint32_t clipping_2;
}) fmav_vibration_t;


#define FASTMAVLINK_MSG_ID_VIBRATION  241

#define FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_VIBRATION_CRCEXTRA  90

#define FASTMAVLINK_MSG_VIBRATION_FLAGS  0
#define FASTMAVLINK_MSG_VIBRATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VIBRATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_VIBRATION_FRAME_LEN_MAX  57



#define FASTMAVLINK_MSG_VIBRATION_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_VIBRATION_FIELD_VIBRATION_X_OFS  8
#define FASTMAVLINK_MSG_VIBRATION_FIELD_VIBRATION_Y_OFS  12
#define FASTMAVLINK_MSG_VIBRATION_FIELD_VIBRATION_Z_OFS  16
#define FASTMAVLINK_MSG_VIBRATION_FIELD_CLIPPING_0_OFS  20
#define FASTMAVLINK_MSG_VIBRATION_FIELD_CLIPPING_1_OFS  24
#define FASTMAVLINK_MSG_VIBRATION_FIELD_CLIPPING_2_OFS  28


//----------------------------------------
//-- Message VIBRATION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float vibration_x, float vibration_y, float vibration_z, uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2,
    fmav_status_t* _status)
{
    fmav_vibration_t* _payload = (fmav_vibration_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->vibration_x = vibration_x;
    _payload->vibration_y = vibration_y;
    _payload->vibration_z = vibration_z;
    _payload->clipping_0 = clipping_0;
    _payload->clipping_1 = clipping_1;
    _payload->clipping_2 = clipping_2;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_VIBRATION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_VIBRATION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vibration_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->vibration_x, _payload->vibration_y, _payload->vibration_z, _payload->clipping_0, _payload->clipping_1, _payload->clipping_2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float vibration_x, float vibration_y, float vibration_z, uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2,
    fmav_status_t* _status)
{
    fmav_vibration_t* _payload = (fmav_vibration_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->vibration_x = vibration_x;
    _payload->vibration_y = vibration_y;
    _payload->vibration_z = vibration_z;
    _payload->clipping_0 = clipping_0;
    _payload->clipping_1 = clipping_1;
    _payload->clipping_2 = clipping_2;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VIBRATION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VIBRATION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VIBRATION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIBRATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vibration_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->vibration_x, _payload->vibration_y, _payload->vibration_z, _payload->clipping_0, _payload->clipping_1, _payload->clipping_2,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float vibration_x, float vibration_y, float vibration_z, uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2,
    fmav_status_t* _status)
{
    fmav_vibration_t _payload;

    _payload.time_usec = time_usec;
    _payload.vibration_x = vibration_x;
    _payload.vibration_y = vibration_y;
    _payload.vibration_z = vibration_z;
    _payload.clipping_0 = clipping_0;
    _payload.clipping_1 = clipping_1;
    _payload.clipping_2 = clipping_2;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_VIBRATION,
        FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIBRATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_vibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_VIBRATION,
        FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIBRATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message VIBRATION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_vibration_decode(fmav_vibration_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_vibration_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vibration_get_field_vibration_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vibration_get_field_vibration_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vibration_get_field_vibration_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_vibration_get_field_clipping_0(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_vibration_get_field_clipping_1(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_vibration_get_field_clipping_2(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VIBRATION  241

#define mavlink_vibration_t  fmav_vibration_t

#define MAVLINK_MSG_ID_VIBRATION_LEN  32
#define MAVLINK_MSG_ID_VIBRATION_MIN_LEN  32
#define MAVLINK_MSG_ID_241_LEN  32
#define MAVLINK_MSG_ID_241_MIN_LEN  32

#define MAVLINK_MSG_ID_VIBRATION_CRC  90
#define MAVLINK_MSG_ID_241_CRC  90




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vibration_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, float vibration_x, float vibration_y, float vibration_z, uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_vibration_pack(
        _msg, sysid, compid,
        time_usec, vibration_x, vibration_y, vibration_z, clipping_0, clipping_1, clipping_2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vibration_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_vibration_t* _payload)
{
    return mavlink_msg_vibration_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->vibration_x, _payload->vibration_y, _payload->vibration_z, _payload->clipping_0, _payload->clipping_1, _payload->clipping_2);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vibration_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float vibration_x, float vibration_y, float vibration_z, uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2)
{
    return fmav_msg_vibration_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, vibration_x, vibration_y, vibration_z, clipping_0, clipping_1, clipping_2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_vibration_decode(const mavlink_message_t* msg, mavlink_vibration_t* payload)
{
    fmav_msg_vibration_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VIBRATION_H
