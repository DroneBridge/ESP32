//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MANUAL_CONTROL_H
#define FASTMAVLINK_MSG_MANUAL_CONTROL_H


//----------------------------------------
//-- Message MANUAL_CONTROL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_manual_control_t {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t r;
    uint16_t buttons;
    uint8_t target;
    uint16_t buttons2;
    uint8_t enabled_extensions;
    int16_t s;
    int16_t t;
    int16_t aux1;
    int16_t aux2;
    int16_t aux3;
    int16_t aux4;
    int16_t aux5;
    int16_t aux6;
}) fmav_manual_control_t;


#define FASTMAVLINK_MSG_ID_MANUAL_CONTROL  69

#define FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX  30
#define FASTMAVLINK_MSG_MANUAL_CONTROL_CRCEXTRA  243

#define FASTMAVLINK_MSG_MANUAL_CONTROL_FLAGS  1
#define FASTMAVLINK_MSG_MANUAL_CONTROL_TARGET_SYSTEM_OFS  10
#define FASTMAVLINK_MSG_MANUAL_CONTROL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MANUAL_CONTROL_FRAME_LEN_MAX  55



#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_X_OFS  0
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_Y_OFS  2
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_Z_OFS  4
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_R_OFS  6
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_BUTTONS_OFS  8
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_TARGET_OFS  10
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_BUTTONS2_OFS  11
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_ENABLED_EXTENSIONS_OFS  13
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_S_OFS  14
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_T_OFS  16
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_AUX1_OFS  18
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_AUX2_OFS  20
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_AUX3_OFS  22
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_AUX4_OFS  24
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_AUX5_OFS  26
#define FASTMAVLINK_MSG_MANUAL_CONTROL_FIELD_AUX6_OFS  28


//----------------------------------------
//-- Message MANUAL_CONTROL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_control_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions, int16_t s, int16_t t, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6,
    fmav_status_t* _status)
{
    fmav_manual_control_t* _payload = (fmav_manual_control_t*)_msg->payload;

    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->r = r;
    _payload->buttons = buttons;
    _payload->target = target;
    _payload->buttons2 = buttons2;
    _payload->enabled_extensions = enabled_extensions;
    _payload->s = s;
    _payload->t = t;
    _payload->aux1 = aux1;
    _payload->aux2 = aux2;
    _payload->aux3 = aux3;
    _payload->aux4 = aux4;
    _payload->aux5 = aux5;
    _payload->aux6 = aux6;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MANUAL_CONTROL;
    _msg->target_sysid = target;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MANUAL_CONTROL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_control_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_manual_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_manual_control_pack(
        _msg, sysid, compid,
        _payload->target, _payload->x, _payload->y, _payload->z, _payload->r, _payload->buttons, _payload->buttons2, _payload->enabled_extensions, _payload->s, _payload->t, _payload->aux1, _payload->aux2, _payload->aux3, _payload->aux4, _payload->aux5, _payload->aux6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_control_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions, int16_t s, int16_t t, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6,
    fmav_status_t* _status)
{
    fmav_manual_control_t* _payload = (fmav_manual_control_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->r = r;
    _payload->buttons = buttons;
    _payload->target = target;
    _payload->buttons2 = buttons2;
    _payload->enabled_extensions = enabled_extensions;
    _payload->s = s;
    _payload->t = t;
    _payload->aux1 = aux1;
    _payload->aux2 = aux2;
    _payload->aux3 = aux3;
    _payload->aux4 = aux4;
    _payload->aux5 = aux5;
    _payload->aux6 = aux6;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MANUAL_CONTROL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MANUAL_CONTROL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MANUAL_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MANUAL_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_control_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_manual_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_manual_control_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target, _payload->x, _payload->y, _payload->z, _payload->r, _payload->buttons, _payload->buttons2, _payload->enabled_extensions, _payload->s, _payload->t, _payload->aux1, _payload->aux2, _payload->aux3, _payload->aux4, _payload->aux5, _payload->aux6,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_control_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions, int16_t s, int16_t t, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6,
    fmav_status_t* _status)
{
    fmav_manual_control_t _payload;

    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.r = r;
    _payload.buttons = buttons;
    _payload.target = target;
    _payload.buttons2 = buttons2;
    _payload.enabled_extensions = enabled_extensions;
    _payload.s = s;
    _payload.t = t;
    _payload.aux1 = aux1;
    _payload.aux2 = aux2;
    _payload.aux3 = aux3;
    _payload.aux4 = aux4;
    _payload.aux5 = aux5;
    _payload.aux6 = aux6;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MANUAL_CONTROL,
        FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MANUAL_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_control_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_manual_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MANUAL_CONTROL,
        FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MANUAL_CONTROL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MANUAL_CONTROL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_manual_control_decode(fmav_manual_control_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MANUAL_CONTROL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_x(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_y(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_z(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_r(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_control_get_field_buttons(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_manual_control_get_field_target(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_control_get_field_buttons2(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_manual_control_get_field_enabled_extensions(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_s(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_t(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_aux1(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_aux2(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_aux3(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_aux4(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_aux5(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_manual_control_get_field_aux6(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MANUAL_CONTROL  69

#define mavlink_manual_control_t  fmav_manual_control_t

#define MAVLINK_MSG_ID_MANUAL_CONTROL_LEN  30
#define MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN  11
#define MAVLINK_MSG_ID_69_LEN  30
#define MAVLINK_MSG_ID_69_MIN_LEN  11

#define MAVLINK_MSG_ID_MANUAL_CONTROL_CRC  243
#define MAVLINK_MSG_ID_69_CRC  243




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_manual_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions, int16_t s, int16_t t, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_manual_control_pack(
        _msg, sysid, compid,
        target, x, y, z, r, buttons, buttons2, enabled_extensions, s, t, aux1, aux2, aux3, aux4, aux5, aux6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_manual_control_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_manual_control_t* _payload)
{
    return mavlink_msg_manual_control_pack(
        sysid,
        compid,
        _msg,
        _payload->target, _payload->x, _payload->y, _payload->z, _payload->r, _payload->buttons, _payload->buttons2, _payload->enabled_extensions, _payload->s, _payload->t, _payload->aux1, _payload->aux2, _payload->aux3, _payload->aux4, _payload->aux5, _payload->aux6);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_manual_control_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target, int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions, int16_t s, int16_t t, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6)
{
    return fmav_msg_manual_control_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target, x, y, z, r, buttons, buttons2, enabled_extensions, s, t, aux1, aux2, aux3, aux4, aux5, aux6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_manual_control_decode(const mavlink_message_t* msg, mavlink_manual_control_t* payload)
{
    fmav_msg_manual_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MANUAL_CONTROL_H
