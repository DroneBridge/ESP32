//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_BATTERY_STATUS_H
#define FASTMAVLINK_MSG_BATTERY_STATUS_H


//----------------------------------------
//-- Message BATTERY_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_battery_status_t {
    int32_t current_consumed;
    int32_t energy_consumed;
    int16_t temperature;
    uint16_t voltages[10];
    int16_t current_battery;
    uint8_t id;
    uint8_t battery_function;
    uint8_t type;
    int8_t battery_remaining;
    int32_t time_remaining;
    uint8_t charge_state;
    uint16_t voltages_ext[4];
    uint8_t mode;
    uint32_t fault_bitmask;
}) fmav_battery_status_t;


#define FASTMAVLINK_MSG_ID_BATTERY_STATUS  147

#define FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX  54
#define FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA  154

#define FASTMAVLINK_MSG_BATTERY_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_BATTERY_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_BATTERY_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_BATTERY_STATUS_FRAME_LEN_MAX  79

#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_NUM  10 // number of elements in array
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN  8 // length of array = number of bytes

#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_CURRENT_CONSUMED_OFS  0
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_ENERGY_CONSUMED_OFS  4
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_TEMPERATURE_OFS  8
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_OFS  10
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_CURRENT_BATTERY_OFS  30
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_ID_OFS  32
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_BATTERY_FUNCTION_OFS  33
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_TYPE_OFS  34
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_BATTERY_REMAINING_OFS  35
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_TIME_REMAINING_OFS  36
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_CHARGE_STATE_OFS  40
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_OFS  41
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_MODE_OFS  49
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_FAULT_BITMASK_OFS  50


//----------------------------------------
//-- Message BATTERY_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask,
    fmav_status_t* _status)
{
    fmav_battery_status_t* _payload = (fmav_battery_status_t*)_msg->payload;

    _payload->current_consumed = current_consumed;
    _payload->energy_consumed = energy_consumed;
    _payload->temperature = temperature;
    _payload->current_battery = current_battery;
    _payload->id = id;
    _payload->battery_function = battery_function;
    _payload->type = type;
    _payload->battery_remaining = battery_remaining;
    _payload->time_remaining = time_remaining;
    _payload->charge_state = charge_state;
    _payload->mode = mode;
    _payload->fault_bitmask = fault_bitmask;
    memcpy(&(_payload->voltages), voltages, sizeof(uint16_t)*10);
    memcpy(&(_payload->voltages_ext), voltages_ext, sizeof(uint16_t)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_BATTERY_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery_status_pack(
        _msg, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->temperature, _payload->voltages, _payload->current_battery, _payload->current_consumed, _payload->energy_consumed, _payload->battery_remaining, _payload->time_remaining, _payload->charge_state, _payload->voltages_ext, _payload->mode, _payload->fault_bitmask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask,
    fmav_status_t* _status)
{
    fmav_battery_status_t* _payload = (fmav_battery_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->current_consumed = current_consumed;
    _payload->energy_consumed = energy_consumed;
    _payload->temperature = temperature;
    _payload->current_battery = current_battery;
    _payload->id = id;
    _payload->battery_function = battery_function;
    _payload->type = type;
    _payload->battery_remaining = battery_remaining;
    _payload->time_remaining = time_remaining;
    _payload->charge_state = charge_state;
    _payload->mode = mode;
    _payload->fault_bitmask = fault_bitmask;
    memcpy(&(_payload->voltages), voltages, sizeof(uint16_t)*10);
    memcpy(&(_payload->voltages_ext), voltages_ext, sizeof(uint16_t)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_BATTERY_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->temperature, _payload->voltages, _payload->current_battery, _payload->current_consumed, _payload->energy_consumed, _payload->battery_remaining, _payload->time_remaining, _payload->charge_state, _payload->voltages_ext, _payload->mode, _payload->fault_bitmask,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask,
    fmav_status_t* _status)
{
    fmav_battery_status_t _payload;

    _payload.current_consumed = current_consumed;
    _payload.energy_consumed = energy_consumed;
    _payload.temperature = temperature;
    _payload.current_battery = current_battery;
    _payload.id = id;
    _payload.battery_function = battery_function;
    _payload.type = type;
    _payload.battery_remaining = battery_remaining;
    _payload.time_remaining = time_remaining;
    _payload.charge_state = charge_state;
    _payload.mode = mode;
    _payload.fault_bitmask = fault_bitmask;
    memcpy(&(_payload.voltages), voltages, sizeof(uint16_t)*10);
    memcpy(&(_payload.voltages_ext), voltages_ext, sizeof(uint16_t)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_BATTERY_STATUS,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_BATTERY_STATUS,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message BATTERY_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_battery_status_decode(fmav_battery_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_battery_status_get_field_current_consumed(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_battery_status_get_field_energy_consumed(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_battery_status_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_battery_status_get_field_current_battery(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_status_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_status_get_field_battery_function(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[33]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_status_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_battery_status_get_field_battery_remaining(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_battery_status_get_field_time_remaining(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_status_get_field_charge_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_status_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[49]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_battery_status_get_field_fault_bitmask(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_battery_status_get_field_voltages_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[10]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_get_field_voltages(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_NUM) return 0;
    return ((uint16_t*)&(msg->payload[10]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_battery_status_get_field_voltages_ext_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[41]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_get_field_voltages_ext(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_NUM) return 0;
    return ((uint16_t*)&(msg->payload[41]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_BATTERY_STATUS  147

#define mavlink_battery_status_t  fmav_battery_status_t

#define MAVLINK_MSG_ID_BATTERY_STATUS_LEN  54
#define MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN  36
#define MAVLINK_MSG_ID_147_LEN  54
#define MAVLINK_MSG_ID_147_MIN_LEN  36

#define MAVLINK_MSG_ID_BATTERY_STATUS_CRC  154
#define MAVLINK_MSG_ID_147_CRC  154

#define MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN 10
#define MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_battery_status_pack(
        _msg, sysid, compid,
        id, battery_function, type, temperature, voltages, current_battery, current_consumed, energy_consumed, battery_remaining, time_remaining, charge_state, voltages_ext, mode, fault_bitmask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_battery_status_t* _payload)
{
    return mavlink_msg_battery_status_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->battery_function, _payload->type, _payload->temperature, _payload->voltages, _payload->current_battery, _payload->current_consumed, _payload->energy_consumed, _payload->battery_remaining, _payload->time_remaining, _payload->charge_state, _payload->voltages_ext, _payload->mode, _payload->fault_bitmask);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask)
{
    return fmav_msg_battery_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, battery_function, type, temperature, voltages, current_battery, current_consumed, energy_consumed, battery_remaining, time_remaining, charge_state, voltages_ext, mode, fault_bitmask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_battery_status_decode(const mavlink_message_t* msg, mavlink_battery_status_t* payload)
{
    fmav_msg_battery_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_BATTERY_STATUS_H
