/*
 *   This file is part of DroneBridge: https://github.com/seeul8er/DroneBridge
 *
 *   Copyright 2018 Wolfgang Christl
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

#ifndef DB_ESP32_DB_COMM_PROTOCOL_H
#define DB_ESP32_DB_COMM_PROTOCOL_H


#define DB_COMM_ORIGIN_GND "groundstation"
#define DB_COMM_ORIGIN_UAV "drone"
#define DB_COMM_TYPE_SETTINGS_SUCCESS "settingssuccess"
#define DB_COMM_TYPE_ERROR "error"
#define DB_COMM_TYPE_PING_REQUEST "pingrequest"
#define DB_COMM_TYPE_PING_RESPONSE "pingresponse"
#define DB_COMM_TYPE_SYS_IDENT_REQUEST "system_ident_req"
#define DB_COMM_TYPE_SYS_IDENT_RESPONSE "system_ident_rsp"
#define DB_COMM_TYPE_SETTINGS_CHANGE "settingschange"
#define DB_COMM_TYPE_CAMSELECT "camselect"
#define DB_COMM_TYPE_ADJUSTRC "adjustrc"
#define DB_COMM_TYPE_MSP "mspcommand"
#define DB_COMM_TYPE_ACK "ack"
#define DB_COMM_TYPE_SETTINGS_REQUEST "settingsrequest"
#define DB_COMM_TYPE_SETTINGS_RESPONSE "settingsresponse"
#define DB_COMM_REQUEST_TYPE_WBC "wbc"
#define DB_COMM_REQUEST_TYPE_DB "db"

#define DB_COMM_DST_GND 1
#define DB_COMM_DST_GND_UAV 2
#define DB_COMM_DST_PER 3
#define DB_COMM_DST_GCS 4
#define DB_COMM_DST_UAV 5


#define DB_COMM_KEY_TYPE "type"
#define DB_COMM_KEY_ORIGIN "origin"
#define DB_COMM_KEY_ID "id"
#define DB_COMM_KEY_DEST "destination"
#define DB_COMM_KEY_HARDWID "HID"   // Hardware ID
#define DB_COMM_KEY_FIRMWID "FID"   // Firmware version

#define DB_COMM_CHANGE_DB "db"
#define DB_COMM_CHANGE_DBESP32 "dbesp32"

#define DB_ESP32_FID 101

#endif //DB_ESP32_DB_COMM_PROTOCOL_H
