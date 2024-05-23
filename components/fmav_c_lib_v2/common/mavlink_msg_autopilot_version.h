//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AUTOPILOT_VERSION_H
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_H


//----------------------------------------
//-- Message AUTOPILOT_VERSION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_autopilot_version_t {
    uint64_t capabilities;
    uint64_t uid;
    uint32_t flight_sw_version;
    uint32_t middleware_sw_version;
    uint32_t os_sw_version;
    uint32_t board_version;
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t flight_custom_version[8];
    uint8_t middleware_custom_version[8];
    uint8_t os_custom_version[8];
    uint8_t uid2[18];
}) fmav_autopilot_version_t;


#define FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION  148

#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX  78
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_CRCEXTRA  178

#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FLAGS  0
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FRAME_LEN_MAX  103

#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID2_NUM  18 // number of elements in array
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID2_LEN  18 // length of array = number of bytes

#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_CAPABILITIES_OFS  0
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID_OFS  8
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_SW_VERSION_OFS  16
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_SW_VERSION_OFS  20
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_SW_VERSION_OFS  24
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_BOARD_VERSION_OFS  28
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_VENDOR_ID_OFS  32
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_PRODUCT_ID_OFS  34
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_OFS  36
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_OFS  44
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_OFS  52
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID2_OFS  60


//----------------------------------------
//-- Message AUTOPILOT_VERSION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2,
    fmav_status_t* _status)
{
    fmav_autopilot_version_t* _payload = (fmav_autopilot_version_t*)_msg->payload;

    _payload->capabilities = capabilities;
    _payload->uid = uid;
    _payload->flight_sw_version = flight_sw_version;
    _payload->middleware_sw_version = middleware_sw_version;
    _payload->os_sw_version = os_sw_version;
    _payload->board_version = board_version;
    _payload->vendor_id = vendor_id;
    _payload->product_id = product_id;
    memcpy(&(_payload->flight_custom_version), flight_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->middleware_custom_version), middleware_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->os_custom_version), os_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->uid2), uid2, sizeof(uint8_t)*18);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_VERSION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_version_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_version_pack(
        _msg, sysid, compid,
        _payload->capabilities, _payload->flight_sw_version, _payload->middleware_sw_version, _payload->os_sw_version, _payload->board_version, _payload->flight_custom_version, _payload->middleware_custom_version, _payload->os_custom_version, _payload->vendor_id, _payload->product_id, _payload->uid, _payload->uid2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2,
    fmav_status_t* _status)
{
    fmav_autopilot_version_t* _payload = (fmav_autopilot_version_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->capabilities = capabilities;
    _payload->uid = uid;
    _payload->flight_sw_version = flight_sw_version;
    _payload->middleware_sw_version = middleware_sw_version;
    _payload->os_sw_version = os_sw_version;
    _payload->board_version = board_version;
    _payload->vendor_id = vendor_id;
    _payload->product_id = product_id;
    memcpy(&(_payload->flight_custom_version), flight_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->middleware_custom_version), middleware_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->os_custom_version), os_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->uid2), uid2, sizeof(uint8_t)*18);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_version_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_version_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->capabilities, _payload->flight_sw_version, _payload->middleware_sw_version, _payload->os_sw_version, _payload->board_version, _payload->flight_custom_version, _payload->middleware_custom_version, _payload->os_custom_version, _payload->vendor_id, _payload->product_id, _payload->uid, _payload->uid2,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2,
    fmav_status_t* _status)
{
    fmav_autopilot_version_t _payload;

    _payload.capabilities = capabilities;
    _payload.uid = uid;
    _payload.flight_sw_version = flight_sw_version;
    _payload.middleware_sw_version = middleware_sw_version;
    _payload.os_sw_version = os_sw_version;
    _payload.board_version = board_version;
    _payload.vendor_id = vendor_id;
    _payload.product_id = product_id;
    memcpy(&(_payload.flight_custom_version), flight_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload.middleware_custom_version), middleware_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload.os_custom_version), os_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload.uid2), uid2, sizeof(uint8_t)*18);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_version_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AUTOPILOT_VERSION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_autopilot_version_decode(fmav_autopilot_version_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_autopilot_version_get_field_capabilities(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_autopilot_version_get_field_uid(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_autopilot_version_get_field_flight_sw_version(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_autopilot_version_get_field_middleware_sw_version(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_autopilot_version_get_field_os_sw_version(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_autopilot_version_get_field_board_version(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_get_field_vendor_id(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_get_field_product_id(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_autopilot_version_get_field_flight_custom_version_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[36]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_version_get_field_flight_custom_version(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_NUM) return 0;
    return ((uint8_t*)&(msg->payload[36]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_autopilot_version_get_field_middleware_custom_version_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[44]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_version_get_field_middleware_custom_version(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_NUM) return 0;
    return ((uint8_t*)&(msg->payload[44]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_autopilot_version_get_field_os_custom_version_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[52]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_version_get_field_os_custom_version(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_NUM) return 0;
    return ((uint8_t*)&(msg->payload[52]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_autopilot_version_get_field_uid2_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[60]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_version_get_field_uid2(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID2_NUM) return 0;
    return ((uint8_t*)&(msg->payload[60]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION  148

#define mavlink_autopilot_version_t  fmav_autopilot_version_t

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN  78
#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_MIN_LEN  60
#define MAVLINK_MSG_ID_148_LEN  78
#define MAVLINK_MSG_ID_148_MIN_LEN  60

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC  178
#define MAVLINK_MSG_ID_148_CRC  178

#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_LEN 8
#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_LEN 8
#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_LEN 8
#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID2_LEN 18


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_version_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_autopilot_version_pack(
        _msg, sysid, compid,
        capabilities, flight_sw_version, middleware_sw_version, os_sw_version, board_version, flight_custom_version, middleware_custom_version, os_custom_version, vendor_id, product_id, uid, uid2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_version_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_autopilot_version_t* _payload)
{
    return mavlink_msg_autopilot_version_pack(
        sysid,
        compid,
        _msg,
        _payload->capabilities, _payload->flight_sw_version, _payload->middleware_sw_version, _payload->os_sw_version, _payload->board_version, _payload->flight_custom_version, _payload->middleware_custom_version, _payload->os_custom_version, _payload->vendor_id, _payload->product_id, _payload->uid, _payload->uid2);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_version_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2)
{
    return fmav_msg_autopilot_version_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        capabilities, flight_sw_version, middleware_sw_version, os_sw_version, board_version, flight_custom_version, middleware_custom_version, os_custom_version, vendor_id, product_id, uid, uid2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_autopilot_version_decode(const mavlink_message_t* msg, mavlink_autopilot_version_t* payload)
{
    fmav_msg_autopilot_version_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AUTOPILOT_VERSION_H
