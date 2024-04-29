//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CELLULAR_CONFIG_H
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_H


//----------------------------------------
//-- Message CELLULAR_CONFIG
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_cellular_config_t {
    uint8_t enable_lte;
    uint8_t enable_pin;
    char pin[16];
    char new_pin[16];
    char apn[32];
    char puk[16];
    uint8_t roaming;
    uint8_t response;
}) fmav_cellular_config_t;


#define FASTMAVLINK_MSG_ID_CELLULAR_CONFIG  336

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX  84
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA  245

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FLAGS  0
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FRAME_LEN_MAX  109

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_ENABLE_LTE_OFS  0
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_ENABLE_PIN_OFS  1
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_OFS  2
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_OFS  18
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_OFS  34
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_OFS  66
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_ROAMING_OFS  82
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_RESPONSE_OFS  83


//----------------------------------------
//-- Message CELLULAR_CONFIG pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response,
    fmav_status_t* _status)
{
    fmav_cellular_config_t* _payload = (fmav_cellular_config_t*)_msg->payload;

    _payload->enable_lte = enable_lte;
    _payload->enable_pin = enable_pin;
    _payload->roaming = roaming;
    _payload->response = response;
    memcpy(&(_payload->pin), pin, sizeof(char)*16);
    memcpy(&(_payload->new_pin), new_pin, sizeof(char)*16);
    memcpy(&(_payload->apn), apn, sizeof(char)*32);
    memcpy(&(_payload->puk), puk, sizeof(char)*16);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CELLULAR_CONFIG;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_config_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cellular_config_pack(
        _msg, sysid, compid,
        _payload->enable_lte, _payload->enable_pin, _payload->pin, _payload->new_pin, _payload->apn, _payload->puk, _payload->roaming, _payload->response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response,
    fmav_status_t* _status)
{
    fmav_cellular_config_t* _payload = (fmav_cellular_config_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->enable_lte = enable_lte;
    _payload->enable_pin = enable_pin;
    _payload->roaming = roaming;
    _payload->response = response;
    memcpy(&(_payload->pin), pin, sizeof(char)*16);
    memcpy(&(_payload->new_pin), new_pin, sizeof(char)*16);
    memcpy(&(_payload->apn), apn, sizeof(char)*32);
    memcpy(&(_payload->puk), puk, sizeof(char)*16);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CELLULAR_CONFIG;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CELLULAR_CONFIG >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CELLULAR_CONFIG >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_config_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cellular_config_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->enable_lte, _payload->enable_pin, _payload->pin, _payload->new_pin, _payload->apn, _payload->puk, _payload->roaming, _payload->response,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response,
    fmav_status_t* _status)
{
    fmav_cellular_config_t _payload;

    _payload.enable_lte = enable_lte;
    _payload.enable_pin = enable_pin;
    _payload.roaming = roaming;
    _payload.response = response;
    memcpy(&(_payload.pin), pin, sizeof(char)*16);
    memcpy(&(_payload.new_pin), new_pin, sizeof(char)*16);
    memcpy(&(_payload.apn), apn, sizeof(char)*32);
    memcpy(&(_payload.puk), puk, sizeof(char)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CELLULAR_CONFIG,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_config_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CELLULAR_CONFIG,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CELLULAR_CONFIG decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_cellular_config_decode(fmav_cellular_config_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_config_get_field_enable_lte(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_config_get_field_enable_pin(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_config_get_field_roaming(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[82]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_config_get_field_response(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[83]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_cellular_config_get_field_pin_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[2]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_cellular_config_get_field_pin(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_NUM) return 0;
    return ((char*)&(msg->payload[2]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_cellular_config_get_field_new_pin_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[18]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_cellular_config_get_field_new_pin(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_NUM) return 0;
    return ((char*)&(msg->payload[18]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_cellular_config_get_field_apn_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[34]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_cellular_config_get_field_apn(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_NUM) return 0;
    return ((char*)&(msg->payload[34]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_cellular_config_get_field_puk_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[66]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_cellular_config_get_field_puk(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_NUM) return 0;
    return ((char*)&(msg->payload[66]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CELLULAR_CONFIG  336

#define mavlink_cellular_config_t  fmav_cellular_config_t

#define MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN  84
#define MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN  84
#define MAVLINK_MSG_ID_336_LEN  84
#define MAVLINK_MSG_ID_336_MIN_LEN  84

#define MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC  245
#define MAVLINK_MSG_ID_336_CRC  245

#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_LEN 16
#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_LEN 16
#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_LEN 32
#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_config_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_cellular_config_pack(
        _msg, sysid, compid,
        enable_lte, enable_pin, pin, new_pin, apn, puk, roaming, response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_config_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_cellular_config_t* _payload)
{
    return mavlink_msg_cellular_config_pack(
        sysid,
        compid,
        _msg,
        _payload->enable_lte, _payload->enable_pin, _payload->pin, _payload->new_pin, _payload->apn, _payload->puk, _payload->roaming, _payload->response);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_config_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response)
{
    return fmav_msg_cellular_config_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        enable_lte, enable_pin, pin, new_pin, apn, puk, roaming, response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_cellular_config_decode(const mavlink_message_t* msg, mavlink_cellular_config_t* payload)
{
    fmav_msg_cellular_config_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CELLULAR_CONFIG_H
