/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2024 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

#ifndef DB_ESP32_DB_MAVLINK_MSGS_H
#define DB_ESP32_DB_MAVLINK_MSGS_H

#include "db_serial.h"
#include "fastmavlink/c_library/common/common.h"
#include <stdint.h>

// typedef struct  {
//     char mavlink_parameter_name[16];
//     void* parameter_internal_variable;
//     MAV_PARAM_TYPE mavlink_parameter_data_type;
// } db_internal_parameters_t;

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/

/***************************************************************************************************************************
 * Based on the system architecture and configured wifi mode the ESP32 may have a different role and system id. Returns the
 * best fitting component ID for the specific role.
 * @return component ID for ESP32
 **************************************************************************************************************************/
uint8_t db_get_mav_comp_id();

/***************************************************************************************************************************
 * Return the Mavlink system ID. Set by handle_mavlink_message()
 * @return system ID for ESP32
 **************************************************************************************************************************/
uint8_t db_get_mav_sys_id();

/***************************************************************************************************************************
 * Creates and writes Mavlink heartbeat message to supplied buffer
 * @param[out] buff        Pointer to the buffer where the packed MAVLink message will be stored.
 *                         The buffer must be large enough to hold the full frame.
 * @param[in,out] fmav_status Pointer to the MAVLink status structure, used for message framing.
 *                         It keeps track of the sequence number and parsing state.
 *
 * @return The length of the packed MAVLink message in bytes.
 **************************************************************************************************************************/
uint16_t db_create_heartbeat(uint8_t *buff, fmav_status_t *fmav_status);

/***************************************************************************************************************************
 * Creates a mavlink PARAM_VALUE message inside the provided buffer using the provided parameter
 *
 * @param buff Buffer to write the mavlink message to
 * @param fmav_status fastmavlink status stucture
 * @param param_index Index of the parameter to send
 * @param value The value of the parameter to be sent -> will be converted to IEEE 745
 * @param type The MAV_PARAM_TYPE
 * @param param_id The name of the parameter (ID)
 * @return Length of the mavlink message inside the buffer
 **************************************************************************************************************************/
uint16_t db_get_mavmsg_param(uint8_t *buff, fmav_status_t *fmav_status, uint16_t param_index, float_int_union *value,
                             uint8_t type, char *param_id);

/***************************************************************************************************************************
 * Gets the mavlink parameter value as float_int_union based on the parameter ID from the internal variable.
 * Maps MAVLink parameter names to the internal variable names.
 *
 * @param float_int IEEE 754 storage for the retrieved value
 * @param param_id  Parameter name you want the value of
 * @param param_index Index of the parameter. May be -1 if requested parameter shall be found based on param_id
 * @return MAV_TYPE of the parameter. Returns 0 if the parameter was not found
 **************************************************************************************************************************/
MAV_TYPE db_mav_get_parameter_value(float_int_union *float_int, char *param_id, int16_t param_index);

/***************************************************************************************************************************
 * Writes the parameter received via mavlink PARAM_SET to the internal variable and triggers write to NVS.
 * For some parameters to become effective the ESP32 still needs to be rebooted!
 *
 * @param param_set_payload
 * @return 1 in case of success and 0 in case of failure
 **************************************************************************************************************************/
bool db_write_mavlink_parameter(fmav_param_set_t *param_set_payload);

/***************************************************************************************************************************
 * Called when a MSG_ID_COMMAND_LONG was received. Handles the command processing.
 * @param the_command The structure of the received command
 * @param the_msg The mavlink message that was parsed (containing the_command)
 * @param status fastmavlink parser status structure
 * @param buff Supply output buffer for sending data
 * @param origin Origin of the command as defined by DB_MAVLINK_DATA_ORIGIN
 * @param tcp_clients List of connected tcp clients as sockets
 * @param udp_conns List of connected UDP clients
 **************************************************************************************************************************/
void db_process_mavlink_command(fmav_command_long_t *the_command, fmav_message_t *the_msg, fmav_status_t *status,
                                uint8_t *buff, enum DB_MAVLINK_DATA_ORIGIN origin, int *tcp_clients,
                                udp_conn_list_t *udp_conns);

/***************************************************************************************************************************
 * Expects GCS to have system ID 255.
 * Processes Mavlink messages and sends the radio status message and heartbeat to the GCS on every heartbeat received via
 * UART
 *
 * We expect the FC to be connected to serial port when in WiFi-AP or in WiFi-Client Mode.
 * We expect the GCS to be connected to serial port when in AP-LR or ESP-NOW GND mode.
 *
 * @param new_msg Message to process
 * @param tcp_clients List of connected tcp clients
 * @param udp_conns List of connected UDP clients
 * @param fmav_status fastmavlink library parser status - setup once by the parser for a specific link/interface
 * @param origin Indicates from what kind of input/link we received the new message.
 **************************************************************************************************************************/
void handle_mavlink_message(fmav_message_t *new_msg, int *tcp_clients, udp_conn_list_t *udp_conns,
                            fmav_status_t *fmav_status, enum DB_MAVLINK_DATA_ORIGIN origin);

#endif // DB_ESP32_DB_MAVLINK_MSGS_H
