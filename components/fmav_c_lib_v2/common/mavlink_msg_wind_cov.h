//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WIND_COV_H
#define FASTMAVLINK_MSG_WIND_COV_H


//----------------------------------------
//-- Message WIND_COV
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_wind_cov_t {
    uint64_t time_usec;
    float wind_x;
    float wind_y;
    float wind_z;
    float var_horiz;
    float var_vert;
    float wind_alt;
    float horiz_accuracy;
    float vert_accuracy;
}) fmav_wind_cov_t;


#define FASTMAVLINK_MSG_ID_WIND_COV  231

#define FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX  40
#define FASTMAVLINK_MSG_WIND_COV_CRCEXTRA  105

#define FASTMAVLINK_MSG_WIND_COV_FLAGS  0
#define FASTMAVLINK_MSG_WIND_COV_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WIND_COV_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_WIND_COV_FRAME_LEN_MAX  65



#define FASTMAVLINK_MSG_WIND_COV_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_WIND_COV_FIELD_WIND_X_OFS  8
#define FASTMAVLINK_MSG_WIND_COV_FIELD_WIND_Y_OFS  12
#define FASTMAVLINK_MSG_WIND_COV_FIELD_WIND_Z_OFS  16
#define FASTMAVLINK_MSG_WIND_COV_FIELD_VAR_HORIZ_OFS  20
#define FASTMAVLINK_MSG_WIND_COV_FIELD_VAR_VERT_OFS  24
#define FASTMAVLINK_MSG_WIND_COV_FIELD_WIND_ALT_OFS  28
#define FASTMAVLINK_MSG_WIND_COV_FIELD_HORIZ_ACCURACY_OFS  32
#define FASTMAVLINK_MSG_WIND_COV_FIELD_VERT_ACCURACY_OFS  36


//----------------------------------------
//-- Message WIND_COV pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy,
    fmav_status_t* _status)
{
    fmav_wind_cov_t* _payload = (fmav_wind_cov_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->wind_x = wind_x;
    _payload->wind_y = wind_y;
    _payload->wind_z = wind_z;
    _payload->var_horiz = var_horiz;
    _payload->var_vert = var_vert;
    _payload->wind_alt = wind_alt;
    _payload->horiz_accuracy = horiz_accuracy;
    _payload->vert_accuracy = vert_accuracy;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_WIND_COV;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_WIND_COV_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wind_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wind_cov_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->wind_x, _payload->wind_y, _payload->wind_z, _payload->var_horiz, _payload->var_vert, _payload->wind_alt, _payload->horiz_accuracy, _payload->vert_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy,
    fmav_status_t* _status)
{
    fmav_wind_cov_t* _payload = (fmav_wind_cov_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->wind_x = wind_x;
    _payload->wind_y = wind_y;
    _payload->wind_z = wind_z;
    _payload->var_horiz = var_horiz;
    _payload->var_vert = var_vert;
    _payload->wind_alt = wind_alt;
    _payload->horiz_accuracy = horiz_accuracy;
    _payload->vert_accuracy = vert_accuracy;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WIND_COV;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WIND_COV >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WIND_COV >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIND_COV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wind_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wind_cov_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->wind_x, _payload->wind_y, _payload->wind_z, _payload->var_horiz, _payload->var_vert, _payload->wind_alt, _payload->horiz_accuracy, _payload->vert_accuracy,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy,
    fmav_status_t* _status)
{
    fmav_wind_cov_t _payload;

    _payload.time_usec = time_usec;
    _payload.wind_x = wind_x;
    _payload.wind_y = wind_y;
    _payload.wind_z = wind_z;
    _payload.var_horiz = var_horiz;
    _payload.var_vert = var_vert;
    _payload.wind_alt = wind_alt;
    _payload.horiz_accuracy = horiz_accuracy;
    _payload.vert_accuracy = vert_accuracy;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_WIND_COV,
        FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIND_COV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_wind_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_WIND_COV,
        FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIND_COV_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message WIND_COV decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wind_cov_decode(fmav_wind_cov_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_wind_cov_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_cov_get_field_wind_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_cov_get_field_wind_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_cov_get_field_wind_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_cov_get_field_var_horiz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_cov_get_field_var_vert(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_cov_get_field_wind_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_cov_get_field_horiz_accuracy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_cov_get_field_vert_accuracy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WIND_COV  231

#define mavlink_wind_cov_t  fmav_wind_cov_t

#define MAVLINK_MSG_ID_WIND_COV_LEN  40
#define MAVLINK_MSG_ID_WIND_COV_MIN_LEN  40
#define MAVLINK_MSG_ID_231_LEN  40
#define MAVLINK_MSG_ID_231_MIN_LEN  40

#define MAVLINK_MSG_ID_WIND_COV_CRC  105
#define MAVLINK_MSG_ID_231_CRC  105




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wind_cov_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_wind_cov_pack(
        _msg, sysid, compid,
        time_usec, wind_x, wind_y, wind_z, var_horiz, var_vert, wind_alt, horiz_accuracy, vert_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wind_cov_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_wind_cov_t* _payload)
{
    return mavlink_msg_wind_cov_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->wind_x, _payload->wind_y, _payload->wind_z, _payload->var_horiz, _payload->var_vert, _payload->wind_alt, _payload->horiz_accuracy, _payload->vert_accuracy);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wind_cov_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy)
{
    return fmav_msg_wind_cov_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, wind_x, wind_y, wind_z, var_horiz, var_vert, wind_alt, horiz_accuracy, vert_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_wind_cov_decode(const mavlink_message_t* msg, mavlink_wind_cov_t* payload)
{
    fmav_msg_wind_cov_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WIND_COV_H
