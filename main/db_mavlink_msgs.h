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
#include "common/common.h"
#include "db_serial.h"

//typedef struct  {
//    char mavlink_parameter_name[16];
//    void* parameter_internal_variable;
//    MAV_PARAM_TYPE mavlink_parameter_data_type;
//} db_internal_parameters_t;

uint8_t db_get_mav_comp_id();
uint8_t db_get_mav_sys_id();
uint16_t db_get_mavmsg_param(uint8_t *buff, fmav_status_t *fmav_status, uint16_t param_index, float_int_union *value, uint8_t type, char *param_id);
MAV_TYPE db_mav_get_parameter_value(float_int_union *float_int, char *param_id);
bool db_write_mavlink_parameter(fmav_param_set_t *param_set_payload);

#endif //DB_ESP32_DB_MAVLINK_MSGS_H
