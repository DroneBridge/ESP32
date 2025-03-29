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

#include <stdint.h>
#include "db_serial.h"
#include "common/common.h"

uint8_t db_get_mav_comp_id();
uint8_t db_get_mav_sys_id();
uint16_t db_create_heartbeat(uint8_t *buff, fmav_status_t *fmav_status);
uint16_t db_get_mavmsg_param_value(uint8_t *buff, fmav_status_t *fmav_status, uint16_t param_index, float_int_union *value, uint8_t type, char *param_id);
MAV_PARAM_TYPE db_mav_get_parameter_value(float_int_union *float_int, const char *param_id, const int16_t param_index);
bool db_write_mavlink_parameter(const fmav_param_set_t *param_set_payload);
void db_process_mavlink_command(fmav_command_long_t *the_command,
                                fmav_message_t *the_msg,
                                fmav_status_t *status,
                                uint8_t *buff,
                                enum DB_MAVLINK_DATA_ORIGIN origin, int *tcp_clients, udp_conn_list_t *udp_conns);
void handle_mavlink_message(fmav_message_t *new_msg, int *tcp_clients, udp_conn_list_t *udp_conns,
                            fmav_status_t *fmav_status,
                            enum DB_MAVLINK_DATA_ORIGIN origin);

#endif //DB_ESP32_DB_MAVLINK_MSGS_H
