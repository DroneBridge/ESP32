//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS2_RTK_H
#define FASTMAVLINK_MSG_GPS2_RTK_H


//----------------------------------------
//-- Message GPS2_RTK
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps2_rtk_t {
    uint32_t time_last_baseline_ms;
    uint32_t tow;
    int32_t baseline_a_mm;
    int32_t baseline_b_mm;
    int32_t baseline_c_mm;
    uint32_t accuracy;
    int32_t iar_num_hypotheses;
    uint16_t wn;
    uint8_t rtk_receiver_id;
    uint8_t rtk_health;
    uint8_t rtk_rate;
    uint8_t nsats;
    uint8_t baseline_coords_type;
}) fmav_gps2_rtk_t;


#define FASTMAVLINK_MSG_ID_GPS2_RTK  128

#define FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX  35
#define FASTMAVLINK_MSG_GPS2_RTK_CRCEXTRA  226

#define FASTMAVLINK_MSG_GPS2_RTK_FLAGS  0
#define FASTMAVLINK_MSG_GPS2_RTK_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS2_RTK_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GPS2_RTK_FRAME_LEN_MAX  60



#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_TIME_LAST_BASELINE_MS_OFS  0
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_TOW_OFS  4
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_BASELINE_A_MM_OFS  8
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_BASELINE_B_MM_OFS  12
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_BASELINE_C_MM_OFS  16
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_ACCURACY_OFS  20
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_IAR_NUM_HYPOTHESES_OFS  24
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_WN_OFS  28
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_RTK_RECEIVER_ID_OFS  30
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_RTK_HEALTH_OFS  31
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_RTK_RATE_OFS  32
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_NSATS_OFS  33
#define FASTMAVLINK_MSG_GPS2_RTK_FIELD_BASELINE_COORDS_TYPE_OFS  34


//----------------------------------------
//-- Message GPS2_RTK pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses,
    fmav_status_t* _status)
{
    fmav_gps2_rtk_t* _payload = (fmav_gps2_rtk_t*)_msg->payload;

    _payload->time_last_baseline_ms = time_last_baseline_ms;
    _payload->tow = tow;
    _payload->baseline_a_mm = baseline_a_mm;
    _payload->baseline_b_mm = baseline_b_mm;
    _payload->baseline_c_mm = baseline_c_mm;
    _payload->accuracy = accuracy;
    _payload->iar_num_hypotheses = iar_num_hypotheses;
    _payload->wn = wn;
    _payload->rtk_receiver_id = rtk_receiver_id;
    _payload->rtk_health = rtk_health;
    _payload->rtk_rate = rtk_rate;
    _payload->nsats = nsats;
    _payload->baseline_coords_type = baseline_coords_type;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GPS2_RTK;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_GPS2_RTK_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_rtk_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps2_rtk_pack(
        _msg, sysid, compid,
        _payload->time_last_baseline_ms, _payload->rtk_receiver_id, _payload->wn, _payload->tow, _payload->rtk_health, _payload->rtk_rate, _payload->nsats, _payload->baseline_coords_type, _payload->baseline_a_mm, _payload->baseline_b_mm, _payload->baseline_c_mm, _payload->accuracy, _payload->iar_num_hypotheses,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses,
    fmav_status_t* _status)
{
    fmav_gps2_rtk_t* _payload = (fmav_gps2_rtk_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_last_baseline_ms = time_last_baseline_ms;
    _payload->tow = tow;
    _payload->baseline_a_mm = baseline_a_mm;
    _payload->baseline_b_mm = baseline_b_mm;
    _payload->baseline_c_mm = baseline_c_mm;
    _payload->accuracy = accuracy;
    _payload->iar_num_hypotheses = iar_num_hypotheses;
    _payload->wn = wn;
    _payload->rtk_receiver_id = rtk_receiver_id;
    _payload->rtk_health = rtk_health;
    _payload->rtk_rate = rtk_rate;
    _payload->nsats = nsats;
    _payload->baseline_coords_type = baseline_coords_type;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS2_RTK;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS2_RTK >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS2_RTK >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RTK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_rtk_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps2_rtk_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_last_baseline_ms, _payload->rtk_receiver_id, _payload->wn, _payload->tow, _payload->rtk_health, _payload->rtk_rate, _payload->nsats, _payload->baseline_coords_type, _payload->baseline_a_mm, _payload->baseline_b_mm, _payload->baseline_c_mm, _payload->accuracy, _payload->iar_num_hypotheses,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses,
    fmav_status_t* _status)
{
    fmav_gps2_rtk_t _payload;

    _payload.time_last_baseline_ms = time_last_baseline_ms;
    _payload.tow = tow;
    _payload.baseline_a_mm = baseline_a_mm;
    _payload.baseline_b_mm = baseline_b_mm;
    _payload.baseline_c_mm = baseline_c_mm;
    _payload.accuracy = accuracy;
    _payload.iar_num_hypotheses = iar_num_hypotheses;
    _payload.wn = wn;
    _payload.rtk_receiver_id = rtk_receiver_id;
    _payload.rtk_health = rtk_health;
    _payload.rtk_rate = rtk_rate;
    _payload.nsats = nsats;
    _payload.baseline_coords_type = baseline_coords_type;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GPS2_RTK,
        FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RTK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_rtk_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GPS2_RTK,
        FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RTK_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GPS2_RTK decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps2_rtk_decode(fmav_gps2_rtk_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps2_rtk_get_field_time_last_baseline_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps2_rtk_get_field_tow(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps2_rtk_get_field_baseline_a_mm(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps2_rtk_get_field_baseline_b_mm(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps2_rtk_get_field_baseline_c_mm(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps2_rtk_get_field_accuracy(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps2_rtk_get_field_iar_num_hypotheses(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_get_field_wn(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps2_rtk_get_field_rtk_receiver_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps2_rtk_get_field_rtk_health(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[31]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps2_rtk_get_field_rtk_rate(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps2_rtk_get_field_nsats(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[33]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps2_rtk_get_field_baseline_coords_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS2_RTK  128

#define mavlink_gps2_rtk_t  fmav_gps2_rtk_t

#define MAVLINK_MSG_ID_GPS2_RTK_LEN  35
#define MAVLINK_MSG_ID_GPS2_RTK_MIN_LEN  35
#define MAVLINK_MSG_ID_128_LEN  35
#define MAVLINK_MSG_ID_128_MIN_LEN  35

#define MAVLINK_MSG_ID_GPS2_RTK_CRC  226
#define MAVLINK_MSG_ID_128_CRC  226




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_rtk_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps2_rtk_pack(
        _msg, sysid, compid,
        time_last_baseline_ms, rtk_receiver_id, wn, tow, rtk_health, rtk_rate, nsats, baseline_coords_type, baseline_a_mm, baseline_b_mm, baseline_c_mm, accuracy, iar_num_hypotheses,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_rtk_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gps2_rtk_t* _payload)
{
    return mavlink_msg_gps2_rtk_pack(
        sysid,
        compid,
        _msg,
        _payload->time_last_baseline_ms, _payload->rtk_receiver_id, _payload->wn, _payload->tow, _payload->rtk_health, _payload->rtk_rate, _payload->nsats, _payload->baseline_coords_type, _payload->baseline_a_mm, _payload->baseline_b_mm, _payload->baseline_c_mm, _payload->accuracy, _payload->iar_num_hypotheses);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_rtk_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses)
{
    return fmav_msg_gps2_rtk_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_last_baseline_ms, rtk_receiver_id, wn, tow, rtk_health, rtk_rate, nsats, baseline_coords_type, baseline_a_mm, baseline_b_mm, baseline_c_mm, accuracy, iar_num_hypotheses,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps2_rtk_decode(const mavlink_message_t* msg, mavlink_gps2_rtk_t* payload)
{
    fmav_msg_gps2_rtk_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS2_RTK_H
