//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GENERATOR_STATUS_H
#define FASTMAVLINK_MSG_GENERATOR_STATUS_H


//----------------------------------------
//-- Message GENERATOR_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_generator_status_t {
    uint64_t status;
    float battery_current;
    float load_current;
    float power_generated;
    float bus_voltage;
    float bat_current_setpoint;
    uint32_t runtime;
    int32_t time_until_maintenance;
    uint16_t generator_speed;
    int16_t rectifier_temperature;
    int16_t generator_temperature;
}) fmav_generator_status_t;


#define FASTMAVLINK_MSG_ID_GENERATOR_STATUS  373

#define FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_GENERATOR_STATUS_CRCEXTRA  117

#define FASTMAVLINK_MSG_GENERATOR_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_GENERATOR_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GENERATOR_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GENERATOR_STATUS_FRAME_LEN_MAX  67



#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_STATUS_OFS  0
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_BATTERY_CURRENT_OFS  8
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_LOAD_CURRENT_OFS  12
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_POWER_GENERATED_OFS  16
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_BUS_VOLTAGE_OFS  20
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_BAT_CURRENT_SETPOINT_OFS  24
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_RUNTIME_OFS  28
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_TIME_UNTIL_MAINTENANCE_OFS  32
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_GENERATOR_SPEED_OFS  36
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_RECTIFIER_TEMPERATURE_OFS  38
#define FASTMAVLINK_MSG_GENERATOR_STATUS_FIELD_GENERATOR_TEMPERATURE_OFS  40


//----------------------------------------
//-- Message GENERATOR_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance,
    fmav_status_t* _status)
{
    fmav_generator_status_t* _payload = (fmav_generator_status_t*)_msg->payload;

    _payload->status = status;
    _payload->battery_current = battery_current;
    _payload->load_current = load_current;
    _payload->power_generated = power_generated;
    _payload->bus_voltage = bus_voltage;
    _payload->bat_current_setpoint = bat_current_setpoint;
    _payload->runtime = runtime;
    _payload->time_until_maintenance = time_until_maintenance;
    _payload->generator_speed = generator_speed;
    _payload->rectifier_temperature = rectifier_temperature;
    _payload->generator_temperature = generator_temperature;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GENERATOR_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_GENERATOR_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_generator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_generator_status_pack(
        _msg, sysid, compid,
        _payload->status, _payload->generator_speed, _payload->battery_current, _payload->load_current, _payload->power_generated, _payload->bus_voltage, _payload->rectifier_temperature, _payload->bat_current_setpoint, _payload->generator_temperature, _payload->runtime, _payload->time_until_maintenance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance,
    fmav_status_t* _status)
{
    fmav_generator_status_t* _payload = (fmav_generator_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->status = status;
    _payload->battery_current = battery_current;
    _payload->load_current = load_current;
    _payload->power_generated = power_generated;
    _payload->bus_voltage = bus_voltage;
    _payload->bat_current_setpoint = bat_current_setpoint;
    _payload->runtime = runtime;
    _payload->time_until_maintenance = time_until_maintenance;
    _payload->generator_speed = generator_speed;
    _payload->rectifier_temperature = rectifier_temperature;
    _payload->generator_temperature = generator_temperature;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GENERATOR_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GENERATOR_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GENERATOR_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GENERATOR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_generator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_generator_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->status, _payload->generator_speed, _payload->battery_current, _payload->load_current, _payload->power_generated, _payload->bus_voltage, _payload->rectifier_temperature, _payload->bat_current_setpoint, _payload->generator_temperature, _payload->runtime, _payload->time_until_maintenance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance,
    fmav_status_t* _status)
{
    fmav_generator_status_t _payload;

    _payload.status = status;
    _payload.battery_current = battery_current;
    _payload.load_current = load_current;
    _payload.power_generated = power_generated;
    _payload.bus_voltage = bus_voltage;
    _payload.bat_current_setpoint = bat_current_setpoint;
    _payload.runtime = runtime;
    _payload.time_until_maintenance = time_until_maintenance;
    _payload.generator_speed = generator_speed;
    _payload.rectifier_temperature = rectifier_temperature;
    _payload.generator_temperature = generator_temperature;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GENERATOR_STATUS,
        FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GENERATOR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_generator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GENERATOR_STATUS,
        FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GENERATOR_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GENERATOR_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_generator_status_decode(fmav_generator_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_generator_status_get_field_status(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_generator_status_get_field_battery_current(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_generator_status_get_field_load_current(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_generator_status_get_field_power_generated(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_generator_status_get_field_bus_voltage(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_generator_status_get_field_bat_current_setpoint(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_generator_status_get_field_runtime(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_generator_status_get_field_time_until_maintenance(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_get_field_generator_speed(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_generator_status_get_field_rectifier_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_generator_status_get_field_generator_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GENERATOR_STATUS  373

#define mavlink_generator_status_t  fmav_generator_status_t

#define MAVLINK_MSG_ID_GENERATOR_STATUS_LEN  42
#define MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN  42
#define MAVLINK_MSG_ID_373_LEN  42
#define MAVLINK_MSG_ID_373_MIN_LEN  42

#define MAVLINK_MSG_ID_GENERATOR_STATUS_CRC  117
#define MAVLINK_MSG_ID_373_CRC  117




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_generator_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_generator_status_pack(
        _msg, sysid, compid,
        status, generator_speed, battery_current, load_current, power_generated, bus_voltage, rectifier_temperature, bat_current_setpoint, generator_temperature, runtime, time_until_maintenance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_generator_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_generator_status_t* _payload)
{
    return mavlink_msg_generator_status_pack(
        sysid,
        compid,
        _msg,
        _payload->status, _payload->generator_speed, _payload->battery_current, _payload->load_current, _payload->power_generated, _payload->bus_voltage, _payload->rectifier_temperature, _payload->bat_current_setpoint, _payload->generator_temperature, _payload->runtime, _payload->time_until_maintenance);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_generator_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance)
{
    return fmav_msg_generator_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        status, generator_speed, battery_current, load_current, power_generated, bus_voltage, rectifier_temperature, bat_current_setpoint, generator_temperature, runtime, time_until_maintenance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_generator_status_decode(const mavlink_message_t* msg, mavlink_generator_status_t* payload)
{
    fmav_msg_generator_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GENERATOR_STATUS_H
