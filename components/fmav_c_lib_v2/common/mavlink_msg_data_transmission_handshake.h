//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_H
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_H


//----------------------------------------
//-- Message DATA_TRANSMISSION_HANDSHAKE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_data_transmission_handshake_t {
    uint32_t size;
    uint16_t width;
    uint16_t height;
    uint16_t packets;
    uint8_t type;
    uint8_t payload;
    uint8_t jpg_quality;
}) fmav_data_transmission_handshake_t;


#define FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE  130

#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX  13
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA  29

#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FLAGS  0
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FRAME_LEN_MAX  38



#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_SIZE_OFS  0
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_WIDTH_OFS  4
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_HEIGHT_OFS  6
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_PACKETS_OFS  8
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_TYPE_OFS  10
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_PAYLOAD_OFS  11
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_JPG_QUALITY_OFS  12


//----------------------------------------
//-- Message DATA_TRANSMISSION_HANDSHAKE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality,
    fmav_status_t* _status)
{
    fmav_data_transmission_handshake_t* _payload = (fmav_data_transmission_handshake_t*)_msg->payload;

    _payload->size = size;
    _payload->width = width;
    _payload->height = height;
    _payload->packets = packets;
    _payload->type = type;
    _payload->payload = payload;
    _payload->jpg_quality = jpg_quality;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_transmission_handshake_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data_transmission_handshake_pack(
        _msg, sysid, compid,
        _payload->type, _payload->size, _payload->width, _payload->height, _payload->packets, _payload->payload, _payload->jpg_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality,
    fmav_status_t* _status)
{
    fmav_data_transmission_handshake_t* _payload = (fmav_data_transmission_handshake_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->size = size;
    _payload->width = width;
    _payload->height = height;
    _payload->packets = packets;
    _payload->type = type;
    _payload->payload = payload;
    _payload->jpg_quality = jpg_quality;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_transmission_handshake_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data_transmission_handshake_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->type, _payload->size, _payload->width, _payload->height, _payload->packets, _payload->payload, _payload->jpg_quality,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality,
    fmav_status_t* _status)
{
    fmav_data_transmission_handshake_t _payload;

    _payload.size = size;
    _payload.width = width;
    _payload.height = height;
    _payload.packets = packets;
    _payload.type = type;
    _payload.payload = payload;
    _payload.jpg_quality = jpg_quality;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_transmission_handshake_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DATA_TRANSMISSION_HANDSHAKE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data_transmission_handshake_decode(fmav_data_transmission_handshake_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_data_transmission_handshake_get_field_size(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_get_field_width(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_get_field_height(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_get_field_packets(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_transmission_handshake_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_transmission_handshake_get_field_payload(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_transmission_handshake_get_field_jpg_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE  130

#define mavlink_data_transmission_handshake_t  fmav_data_transmission_handshake_t

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN  13
#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_MIN_LEN  13
#define MAVLINK_MSG_ID_130_LEN  13
#define MAVLINK_MSG_ID_130_MIN_LEN  13

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_CRC  29
#define MAVLINK_MSG_ID_130_CRC  29




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_transmission_handshake_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_data_transmission_handshake_pack(
        _msg, sysid, compid,
        type, size, width, height, packets, payload, jpg_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_transmission_handshake_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_data_transmission_handshake_t* _payload)
{
    return mavlink_msg_data_transmission_handshake_pack(
        sysid,
        compid,
        _msg,
        _payload->type, _payload->size, _payload->width, _payload->height, _payload->packets, _payload->payload, _payload->jpg_quality);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_transmission_handshake_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality)
{
    return fmav_msg_data_transmission_handshake_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        type, size, width, height, packets, payload, jpg_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_data_transmission_handshake_decode(const mavlink_message_t* msg, mavlink_data_transmission_handshake_t* payload)
{
    fmav_msg_data_transmission_handshake_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_H
