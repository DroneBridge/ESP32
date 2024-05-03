//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_H
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_H


//----------------------------------------
//-- Message GIMBAL_MANAGER_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_manager_status_t {
    uint32_t time_boot_ms;
    uint32_t flags;
    uint8_t gimbal_device_id;
    uint8_t primary_control_sysid;
    uint8_t primary_control_compid;
    uint8_t secondary_control_sysid;
    uint8_t secondary_control_compid;
}) fmav_gimbal_manager_status_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS  281

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX  13
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA  48

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FRAME_LEN_MAX  38



#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_FLAGS_OFS  4
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_GIMBAL_DEVICE_ID_OFS  8
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_PRIMARY_CONTROL_SYSID_OFS  9
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_PRIMARY_CONTROL_COMPID_OFS  10
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_SECONDARY_CONTROL_SYSID_OFS  11
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_SECONDARY_CONTROL_COMPID_OFS  12


//----------------------------------------
//-- Message GIMBAL_MANAGER_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_status_t* _payload = (fmav_gimbal_manager_status_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->flags = flags;
    _payload->gimbal_device_id = gimbal_device_id;
    _payload->primary_control_sysid = primary_control_sysid;
    _payload->primary_control_compid = primary_control_compid;
    _payload->secondary_control_sysid = secondary_control_sysid;
    _payload->secondary_control_compid = secondary_control_compid;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_status_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->flags, _payload->gimbal_device_id, _payload->primary_control_sysid, _payload->primary_control_compid, _payload->secondary_control_sysid, _payload->secondary_control_compid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_status_t* _payload = (fmav_gimbal_manager_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->flags = flags;
    _payload->gimbal_device_id = gimbal_device_id;
    _payload->primary_control_sysid = primary_control_sysid;
    _payload->primary_control_compid = primary_control_compid;
    _payload->secondary_control_sysid = secondary_control_sysid;
    _payload->secondary_control_compid = secondary_control_compid;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->flags, _payload->gimbal_device_id, _payload->primary_control_sysid, _payload->primary_control_compid, _payload->secondary_control_sysid, _payload->secondary_control_compid,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_status_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.flags = flags;
    _payload.gimbal_device_id = gimbal_device_id;
    _payload.primary_control_sysid = primary_control_sysid;
    _payload.primary_control_compid = primary_control_compid;
    _payload.secondary_control_sysid = secondary_control_sysid;
    _payload.secondary_control_compid = secondary_control_compid;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_MANAGER_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_manager_status_decode(fmav_gimbal_manager_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_manager_status_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_manager_status_get_field_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_gimbal_device_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_primary_control_sysid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_primary_control_compid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_secondary_control_sysid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_secondary_control_compid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS  281

#define mavlink_gimbal_manager_status_t  fmav_gimbal_manager_status_t

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN  13
#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN  13
#define MAVLINK_MSG_ID_281_LEN  13
#define MAVLINK_MSG_ID_281_MIN_LEN  13

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC  48
#define MAVLINK_MSG_ID_281_CRC  48




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_manager_status_pack(
        _msg, sysid, compid,
        time_boot_ms, flags, gimbal_device_id, primary_control_sysid, primary_control_compid, secondary_control_sysid, secondary_control_compid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gimbal_manager_status_t* _payload)
{
    return mavlink_msg_gimbal_manager_status_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->flags, _payload->gimbal_device_id, _payload->primary_control_sysid, _payload->primary_control_compid, _payload->secondary_control_sysid, _payload->secondary_control_compid);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid)
{
    return fmav_msg_gimbal_manager_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, flags, gimbal_device_id, primary_control_sysid, primary_control_compid, secondary_control_sysid, secondary_control_compid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_manager_status_decode(const mavlink_message_t* msg, mavlink_gimbal_manager_status_t* payload)
{
    fmav_msg_gimbal_manager_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_H
