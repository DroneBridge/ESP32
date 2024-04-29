//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVCAN_NODE_INFO_H
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_H


//----------------------------------------
//-- Message UAVCAN_NODE_INFO
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavcan_node_info_t {
    uint64_t time_usec;
    uint32_t uptime_sec;
    uint32_t sw_vcs_commit;
    char name[80];
    uint8_t hw_version_major;
    uint8_t hw_version_minor;
    uint8_t hw_unique_id[16];
    uint8_t sw_version_major;
    uint8_t sw_version_minor;
}) fmav_uavcan_node_info_t;


#define FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO  311

#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX  116
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_CRCEXTRA  95

#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FLAGS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FRAME_LEN_MAX  141

#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_NAME_NUM  80 // number of elements in array
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_NAME_LEN  80 // length of array = number of bytes
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_UNIQUE_ID_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_UNIQUE_ID_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_UPTIME_SEC_OFS  8
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_SW_VCS_COMMIT_OFS  12
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_NAME_OFS  16
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_VERSION_MAJOR_OFS  96
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_VERSION_MINOR_OFS  97
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_UNIQUE_ID_OFS  98
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_SW_VERSION_MAJOR_OFS  114
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_SW_VERSION_MINOR_OFS  115


//----------------------------------------
//-- Message UAVCAN_NODE_INFO pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, const char* name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t* hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit,
    fmav_status_t* _status)
{
    fmav_uavcan_node_info_t* _payload = (fmav_uavcan_node_info_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->uptime_sec = uptime_sec;
    _payload->sw_vcs_commit = sw_vcs_commit;
    _payload->hw_version_major = hw_version_major;
    _payload->hw_version_minor = hw_version_minor;
    _payload->sw_version_major = sw_version_major;
    _payload->sw_version_minor = sw_version_minor;
    memcpy(&(_payload->name), name, sizeof(char)*80);
    memcpy(&(_payload->hw_unique_id), hw_unique_id, sizeof(uint8_t)*16);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_UAVCAN_NODE_INFO_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavcan_node_info_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->uptime_sec, _payload->name, _payload->hw_version_major, _payload->hw_version_minor, _payload->hw_unique_id, _payload->sw_version_major, _payload->sw_version_minor, _payload->sw_vcs_commit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, const char* name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t* hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit,
    fmav_status_t* _status)
{
    fmav_uavcan_node_info_t* _payload = (fmav_uavcan_node_info_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->uptime_sec = uptime_sec;
    _payload->sw_vcs_commit = sw_vcs_commit;
    _payload->hw_version_major = hw_version_major;
    _payload->hw_version_minor = hw_version_minor;
    _payload->sw_version_major = sw_version_major;
    _payload->sw_version_minor = sw_version_minor;
    memcpy(&(_payload->name), name, sizeof(char)*80);
    memcpy(&(_payload->hw_unique_id), hw_unique_id, sizeof(uint8_t)*16);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavcan_node_info_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->uptime_sec, _payload->name, _payload->hw_version_major, _payload->hw_version_minor, _payload->hw_unique_id, _payload->sw_version_major, _payload->sw_version_minor, _payload->sw_vcs_commit,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, const char* name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t* hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit,
    fmav_status_t* _status)
{
    fmav_uavcan_node_info_t _payload;

    _payload.time_usec = time_usec;
    _payload.uptime_sec = uptime_sec;
    _payload.sw_vcs_commit = sw_vcs_commit;
    _payload.hw_version_major = hw_version_major;
    _payload.hw_version_minor = hw_version_minor;
    _payload.sw_version_major = sw_version_major;
    _payload.sw_version_minor = sw_version_minor;
    memcpy(&(_payload.name), name, sizeof(char)*80);
    memcpy(&(_payload.hw_unique_id), hw_unique_id, sizeof(uint8_t)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UAVCAN_NODE_INFO decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavcan_node_info_decode(fmav_uavcan_node_info_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_uavcan_node_info_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_uavcan_node_info_get_field_uptime_sec(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_uavcan_node_info_get_field_sw_vcs_commit(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavcan_node_info_get_field_hw_version_major(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[96]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavcan_node_info_get_field_hw_version_minor(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[97]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavcan_node_info_get_field_sw_version_major(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[114]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavcan_node_info_get_field_sw_version_minor(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[115]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_uavcan_node_info_get_field_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[16]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_uavcan_node_info_get_field_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_NAME_NUM) return 0;
    return ((char*)&(msg->payload[16]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_uavcan_node_info_get_field_hw_unique_id_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[98]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavcan_node_info_get_field_hw_unique_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_UNIQUE_ID_NUM) return 0;
    return ((uint8_t*)&(msg->payload[98]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO  311

#define mavlink_uavcan_node_info_t  fmav_uavcan_node_info_t

#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN  116
#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN  116
#define MAVLINK_MSG_ID_311_LEN  116
#define MAVLINK_MSG_ID_311_MIN_LEN  116

#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC  95
#define MAVLINK_MSG_ID_311_CRC  95

#define MAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_NAME_LEN 80
#define MAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_UNIQUE_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavcan_node_info_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint32_t uptime_sec, const char* name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t* hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavcan_node_info_pack(
        _msg, sysid, compid,
        time_usec, uptime_sec, name, hw_version_major, hw_version_minor, hw_unique_id, sw_version_major, sw_version_minor, sw_vcs_commit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavcan_node_info_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_uavcan_node_info_t* _payload)
{
    return mavlink_msg_uavcan_node_info_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->uptime_sec, _payload->name, _payload->hw_version_major, _payload->hw_version_minor, _payload->hw_unique_id, _payload->sw_version_major, _payload->sw_version_minor, _payload->sw_vcs_commit);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavcan_node_info_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, const char* name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t* hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit)
{
    return fmav_msg_uavcan_node_info_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, uptime_sec, name, hw_version_major, hw_version_minor, hw_unique_id, sw_version_major, sw_version_minor, sw_vcs_commit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavcan_node_info_decode(const mavlink_message_t* msg, mavlink_uavcan_node_info_t* payload)
{
    fmav_msg_uavcan_node_info_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVCAN_NODE_INFO_H
