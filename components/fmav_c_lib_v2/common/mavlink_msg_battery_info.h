//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_BATTERY_INFO_H
#define FASTMAVLINK_MSG_BATTERY_INFO_H


//----------------------------------------
//-- Message BATTERY_INFO
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_battery_info_t {
    float discharge_minimum_voltage;
    float charging_minimum_voltage;
    float resting_minimum_voltage;
    float charging_maximum_voltage;
    float charging_maximum_current;
    float nominal_voltage;
    float discharge_maximum_current;
    float discharge_maximum_burst_current;
    float design_capacity;
    float full_charge_capacity;
    uint16_t cycle_count;
    uint16_t weight;
    uint8_t id;
    uint8_t battery_function;
    uint8_t type;
    uint8_t state_of_health;
    uint8_t cells_in_series;
    char manufacture_date[9];
    char serial_number[32];
    char name[50];
}) fmav_battery_info_t;


#define FASTMAVLINK_MSG_ID_BATTERY_INFO  370

#define FASTMAVLINK_MSG_BATTERY_INFO_PAYLOAD_LEN_MAX  140
#define FASTMAVLINK_MSG_BATTERY_INFO_CRCEXTRA  26

#define FASTMAVLINK_MSG_BATTERY_INFO_FLAGS  0
#define FASTMAVLINK_MSG_BATTERY_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_BATTERY_INFO_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_BATTERY_INFO_FRAME_LEN_MAX  165

#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_MANUFACTURE_DATE_NUM  9 // number of elements in array
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_MANUFACTURE_DATE_LEN  9 // length of array = number of bytes
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_SERIAL_NUMBER_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_NAME_NUM  50 // number of elements in array
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_NAME_LEN  50 // length of array = number of bytes

#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_DISCHARGE_MINIMUM_VOLTAGE_OFS  0
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_CHARGING_MINIMUM_VOLTAGE_OFS  4
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_RESTING_MINIMUM_VOLTAGE_OFS  8
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_CHARGING_MAXIMUM_VOLTAGE_OFS  12
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_CHARGING_MAXIMUM_CURRENT_OFS  16
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_NOMINAL_VOLTAGE_OFS  20
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_DISCHARGE_MAXIMUM_CURRENT_OFS  24
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_DISCHARGE_MAXIMUM_BURST_CURRENT_OFS  28
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_DESIGN_CAPACITY_OFS  32
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_FULL_CHARGE_CAPACITY_OFS  36
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_CYCLE_COUNT_OFS  40
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_WEIGHT_OFS  42
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_ID_OFS  44
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_BATTERY_FUNCTION_OFS  45
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_TYPE_OFS  46
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_STATE_OF_HEALTH_OFS  47
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_CELLS_IN_SERIES_OFS  48
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_MANUFACTURE_DATE_OFS  49
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_SERIAL_NUMBER_OFS  58
#define FASTMAVLINK_MSG_BATTERY_INFO_FIELD_NAME_OFS  90


//----------------------------------------
//-- Message BATTERY_INFO pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_info_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, uint8_t state_of_health, uint8_t cells_in_series, uint16_t cycle_count, uint16_t weight, float discharge_minimum_voltage, float charging_minimum_voltage, float resting_minimum_voltage, float charging_maximum_voltage, float charging_maximum_current, float nominal_voltage, float discharge_maximum_current, float discharge_maximum_burst_current, float design_capacity, float full_charge_capacity, const char* manufacture_date, const char* serial_number, const char* name,
    fmav_status_t* _status)
{
    fmav_battery_info_t* _payload = (fmav_battery_info_t*)_msg->payload;

    _payload->discharge_minimum_voltage = discharge_minimum_voltage;
    _payload->charging_minimum_voltage = charging_minimum_voltage;
    _payload->resting_minimum_voltage = resting_minimum_voltage;
    _payload->charging_maximum_voltage = charging_maximum_voltage;
    _payload->charging_maximum_current = charging_maximum_current;
    _payload->nominal_voltage = nominal_voltage;
    _payload->discharge_maximum_current = discharge_maximum_current;
    _payload->discharge_maximum_burst_current = discharge_maximum_burst_current;
    _payload->design_capacity = design_capacity;
    _payload->full_charge_capacity = full_charge_capacity;
    _payload->cycle_count = cycle_count;
    _payload->weight = weight;
    _payload->id = id;
    _payload->battery_function = battery_function;
    _payload->type = type;
    _payload->state_of_health = state_of_health;
    _payload->cells_in_series = cells_in_series;
    memcpy(&(_payload->manufacture_date), manufacture_date, sizeof(char)*9);
    memcpy(&(_payload->serial_number), serial_number, sizeof(char)*32);
    memcpy(&(_payload->name), name, sizeof(char)*50);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_BATTERY_INFO;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_BATTERY_INFO_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_BATTERY_INFO_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_info_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery_info_pack(
        _msg, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->state_of_health, _payload->cells_in_series, _payload->cycle_count, _payload->weight, _payload->discharge_minimum_voltage, _payload->charging_minimum_voltage, _payload->resting_minimum_voltage, _payload->charging_maximum_voltage, _payload->charging_maximum_current, _payload->nominal_voltage, _payload->discharge_maximum_current, _payload->discharge_maximum_burst_current, _payload->design_capacity, _payload->full_charge_capacity, _payload->manufacture_date, _payload->serial_number, _payload->name,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_info_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, uint8_t state_of_health, uint8_t cells_in_series, uint16_t cycle_count, uint16_t weight, float discharge_minimum_voltage, float charging_minimum_voltage, float resting_minimum_voltage, float charging_maximum_voltage, float charging_maximum_current, float nominal_voltage, float discharge_maximum_current, float discharge_maximum_burst_current, float design_capacity, float full_charge_capacity, const char* manufacture_date, const char* serial_number, const char* name,
    fmav_status_t* _status)
{
    fmav_battery_info_t* _payload = (fmav_battery_info_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->discharge_minimum_voltage = discharge_minimum_voltage;
    _payload->charging_minimum_voltage = charging_minimum_voltage;
    _payload->resting_minimum_voltage = resting_minimum_voltage;
    _payload->charging_maximum_voltage = charging_maximum_voltage;
    _payload->charging_maximum_current = charging_maximum_current;
    _payload->nominal_voltage = nominal_voltage;
    _payload->discharge_maximum_current = discharge_maximum_current;
    _payload->discharge_maximum_burst_current = discharge_maximum_burst_current;
    _payload->design_capacity = design_capacity;
    _payload->full_charge_capacity = full_charge_capacity;
    _payload->cycle_count = cycle_count;
    _payload->weight = weight;
    _payload->id = id;
    _payload->battery_function = battery_function;
    _payload->type = type;
    _payload->state_of_health = state_of_health;
    _payload->cells_in_series = cells_in_series;
    memcpy(&(_payload->manufacture_date), manufacture_date, sizeof(char)*9);
    memcpy(&(_payload->serial_number), serial_number, sizeof(char)*32);
    memcpy(&(_payload->name), name, sizeof(char)*50);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_BATTERY_INFO;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY_INFO >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY_INFO >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_BATTERY_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_info_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery_info_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->state_of_health, _payload->cells_in_series, _payload->cycle_count, _payload->weight, _payload->discharge_minimum_voltage, _payload->charging_minimum_voltage, _payload->resting_minimum_voltage, _payload->charging_maximum_voltage, _payload->charging_maximum_current, _payload->nominal_voltage, _payload->discharge_maximum_current, _payload->discharge_maximum_burst_current, _payload->design_capacity, _payload->full_charge_capacity, _payload->manufacture_date, _payload->serial_number, _payload->name,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_info_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, uint8_t state_of_health, uint8_t cells_in_series, uint16_t cycle_count, uint16_t weight, float discharge_minimum_voltage, float charging_minimum_voltage, float resting_minimum_voltage, float charging_maximum_voltage, float charging_maximum_current, float nominal_voltage, float discharge_maximum_current, float discharge_maximum_burst_current, float design_capacity, float full_charge_capacity, const char* manufacture_date, const char* serial_number, const char* name,
    fmav_status_t* _status)
{
    fmav_battery_info_t _payload;

    _payload.discharge_minimum_voltage = discharge_minimum_voltage;
    _payload.charging_minimum_voltage = charging_minimum_voltage;
    _payload.resting_minimum_voltage = resting_minimum_voltage;
    _payload.charging_maximum_voltage = charging_maximum_voltage;
    _payload.charging_maximum_current = charging_maximum_current;
    _payload.nominal_voltage = nominal_voltage;
    _payload.discharge_maximum_current = discharge_maximum_current;
    _payload.discharge_maximum_burst_current = discharge_maximum_burst_current;
    _payload.design_capacity = design_capacity;
    _payload.full_charge_capacity = full_charge_capacity;
    _payload.cycle_count = cycle_count;
    _payload.weight = weight;
    _payload.id = id;
    _payload.battery_function = battery_function;
    _payload.type = type;
    _payload.state_of_health = state_of_health;
    _payload.cells_in_series = cells_in_series;
    memcpy(&(_payload.manufacture_date), manufacture_date, sizeof(char)*9);
    memcpy(&(_payload.serial_number), serial_number, sizeof(char)*32);
    memcpy(&(_payload.name), name, sizeof(char)*50);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_BATTERY_INFO,
        FASTMAVLINK_MSG_BATTERY_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_info_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_BATTERY_INFO,
        FASTMAVLINK_MSG_BATTERY_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_INFO_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message BATTERY_INFO decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_battery_info_decode(fmav_battery_info_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_BATTERY_INFO_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_BATTERY_INFO_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_BATTERY_INFO_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_BATTERY_INFO_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_discharge_minimum_voltage(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_charging_minimum_voltage(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_resting_minimum_voltage(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_charging_maximum_voltage(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_charging_maximum_current(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_nominal_voltage(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_discharge_maximum_current(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_discharge_maximum_burst_current(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_design_capacity(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_info_get_field_full_charge_capacity(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_info_get_field_cycle_count(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_info_get_field_weight(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_info_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_info_get_field_battery_function(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[45]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_info_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[46]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_info_get_field_state_of_health(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[47]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_info_get_field_cells_in_series(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_battery_info_get_field_manufacture_date_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[49]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_battery_info_get_field_manufacture_date(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_BATTERY_INFO_FIELD_MANUFACTURE_DATE_NUM) return 0;
    return ((char*)&(msg->payload[49]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_battery_info_get_field_serial_number_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[58]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_battery_info_get_field_serial_number(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_BATTERY_INFO_FIELD_SERIAL_NUMBER_NUM) return 0;
    return ((char*)&(msg->payload[58]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_battery_info_get_field_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[90]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_battery_info_get_field_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_BATTERY_INFO_FIELD_NAME_NUM) return 0;
    return ((char*)&(msg->payload[90]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_BATTERY_INFO  370

#define mavlink_battery_info_t  fmav_battery_info_t

#define MAVLINK_MSG_ID_BATTERY_INFO_LEN  140
#define MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN  140
#define MAVLINK_MSG_ID_370_LEN  140
#define MAVLINK_MSG_ID_370_MIN_LEN  140

#define MAVLINK_MSG_ID_BATTERY_INFO_CRC  26
#define MAVLINK_MSG_ID_370_CRC  26

#define MAVLINK_MSG_BATTERY_INFO_FIELD_MANUFACTURE_DATE_LEN 9
#define MAVLINK_MSG_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN 32
#define MAVLINK_MSG_BATTERY_INFO_FIELD_NAME_LEN 50


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_info_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, uint8_t battery_function, uint8_t type, uint8_t state_of_health, uint8_t cells_in_series, uint16_t cycle_count, uint16_t weight, float discharge_minimum_voltage, float charging_minimum_voltage, float resting_minimum_voltage, float charging_maximum_voltage, float charging_maximum_current, float nominal_voltage, float discharge_maximum_current, float discharge_maximum_burst_current, float design_capacity, float full_charge_capacity, const char* manufacture_date, const char* serial_number, const char* name)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_battery_info_pack(
        _msg, sysid, compid,
        id, battery_function, type, state_of_health, cells_in_series, cycle_count, weight, discharge_minimum_voltage, charging_minimum_voltage, resting_minimum_voltage, charging_maximum_voltage, charging_maximum_current, nominal_voltage, discharge_maximum_current, discharge_maximum_burst_current, design_capacity, full_charge_capacity, manufacture_date, serial_number, name,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_info_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_battery_info_t* _payload)
{
    return mavlink_msg_battery_info_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->battery_function, _payload->type, _payload->state_of_health, _payload->cells_in_series, _payload->cycle_count, _payload->weight, _payload->discharge_minimum_voltage, _payload->charging_minimum_voltage, _payload->resting_minimum_voltage, _payload->charging_maximum_voltage, _payload->charging_maximum_current, _payload->nominal_voltage, _payload->discharge_maximum_current, _payload->discharge_maximum_burst_current, _payload->design_capacity, _payload->full_charge_capacity, _payload->manufacture_date, _payload->serial_number, _payload->name);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_info_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, uint8_t state_of_health, uint8_t cells_in_series, uint16_t cycle_count, uint16_t weight, float discharge_minimum_voltage, float charging_minimum_voltage, float resting_minimum_voltage, float charging_maximum_voltage, float charging_maximum_current, float nominal_voltage, float discharge_maximum_current, float discharge_maximum_burst_current, float design_capacity, float full_charge_capacity, const char* manufacture_date, const char* serial_number, const char* name)
{
    return fmav_msg_battery_info_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, battery_function, type, state_of_health, cells_in_series, cycle_count, weight, discharge_minimum_voltage, charging_minimum_voltage, resting_minimum_voltage, charging_maximum_voltage, charging_maximum_current, nominal_voltage, discharge_maximum_current, discharge_maximum_burst_current, design_capacity, full_charge_capacity, manufacture_date, serial_number, name,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_battery_info_decode(const mavlink_message_t* msg, mavlink_battery_info_t* payload)
{
    fmav_msg_battery_info_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_BATTERY_INFO_H
