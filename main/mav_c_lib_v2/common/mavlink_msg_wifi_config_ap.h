//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WIFI_CONFIG_AP_H
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_H


//----------------------------------------
//-- Message WIFI_CONFIG_AP
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_wifi_config_ap_t {
    char ssid[32];
    char password[64];
    int8_t mode;
    int8_t response;
}) fmav_wifi_config_ap_t;


#define FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP  299

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX  98
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA  19

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FLAGS  0
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FRAME_LEN_MAX  123

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_NUM  64 // number of elements in array
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_LEN  64 // length of array = number of bytes

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_OFS  0
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_OFS  32
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_MODE_OFS  96
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_RESPONSE_OFS  97


//----------------------------------------
//-- Message WIFI_CONFIG_AP pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, const char* password, int8_t mode, int8_t response,
    fmav_status_t* _status)
{
    fmav_wifi_config_ap_t* _payload = (fmav_wifi_config_ap_t*)_msg->payload;

    _payload->mode = mode;
    _payload->response = response;
    memcpy(&(_payload->ssid), ssid, sizeof(char)*32);
    memcpy(&(_payload->password), password, sizeof(char)*64);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wifi_config_ap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wifi_config_ap_pack(
        _msg, sysid, compid,
        _payload->ssid, _payload->password, _payload->mode, _payload->response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, const char* password, int8_t mode, int8_t response,
    fmav_status_t* _status)
{
    fmav_wifi_config_ap_t* _payload = (fmav_wifi_config_ap_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mode = mode;
    _payload->response = response;
    memcpy(&(_payload->ssid), ssid, sizeof(char)*32);
    memcpy(&(_payload->password), password, sizeof(char)*64);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wifi_config_ap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wifi_config_ap_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->ssid, _payload->password, _payload->mode, _payload->response,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, const char* password, int8_t mode, int8_t response,
    fmav_status_t* _status)
{
    fmav_wifi_config_ap_t _payload;

    _payload.mode = mode;
    _payload.response = response;
    memcpy(&(_payload.ssid), ssid, sizeof(char)*32);
    memcpy(&(_payload.password), password, sizeof(char)*64);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_wifi_config_ap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message WIFI_CONFIG_AP decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wifi_config_ap_decode(fmav_wifi_config_ap_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_wifi_config_ap_get_field_mode(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[96]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_wifi_config_ap_get_field_response(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[97]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_wifi_config_ap_get_field_ssid_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_wifi_config_ap_get_field_ssid(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_NUM) return 0;
    return ((char*)&(msg->payload[0]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_wifi_config_ap_get_field_password_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[32]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_wifi_config_ap_get_field_password(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_NUM) return 0;
    return ((char*)&(msg->payload[32]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WIFI_CONFIG_AP  299

#define mavlink_wifi_config_ap_t  fmav_wifi_config_ap_t

#define MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN  98
#define MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN  96
#define MAVLINK_MSG_ID_299_LEN  98
#define MAVLINK_MSG_ID_299_MIN_LEN  96

#define MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC  19
#define MAVLINK_MSG_ID_299_CRC  19

#define MAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_LEN 32
#define MAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_LEN 64


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wifi_config_ap_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const char* ssid, const char* password, int8_t mode, int8_t response)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_wifi_config_ap_pack(
        _msg, sysid, compid,
        ssid, password, mode, response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wifi_config_ap_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_wifi_config_ap_t* _payload)
{
    return mavlink_msg_wifi_config_ap_pack(
        sysid,
        compid,
        _msg,
        _payload->ssid, _payload->password, _payload->mode, _payload->response);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wifi_config_ap_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, const char* password, int8_t mode, int8_t response)
{
    return fmav_msg_wifi_config_ap_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        ssid, password, mode, response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_wifi_config_ap_decode(const mavlink_message_t* msg, mavlink_wifi_config_ap_t* payload)
{
    fmav_msg_wifi_config_ap_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WIFI_CONFIG_AP_H
