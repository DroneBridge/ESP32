/*
 *   This file is part of DroneBridge: https://github.com/seeul8er/DroneBridge
 *
 *   Copyright 2017 Wolfgang Christl
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

#include <stdint.h>
#include <time.h>
#include <net/if.h>

#ifndef DB_PROTOCOL_H_INCLUDED
#define DB_PROTOCOL_H_INCLUDED

#define RADIOTAP_LENGTH         13
#define DB_RAW_V2_HEADER_LENGTH 10
#define DB_MAX_ADAPTERS 4

#define MSP_DATA_LENTH          34      // size of MSP v1
#define MSP_V2_DATA_LENGTH      37      // size of MSP v2 frame
#define DB_RC_DATA_LENGTH		16		// size of DB_RC frame
#define DATA_UNI_LENGTH         2048	// max payload length for raw protocol
#define DB_RAW_OFFSET			14      // when adhering the 802.11 header the payload is offset to not be overwritten by SQN
#define MAX_DB_DATA_LENGTH		(RADIOTAP_LENGTH + DB_RAW_V2_HEADER_LENGTH + DATA_UNI_LENGTH) // max length of a db raw packet
#define ETHER_TYPE              0x88ab

#define DEFAULT_DB_MODE         'm'
#define DEFAULT_DB_IF           "18a6f716a511"
#define DEFAULT_V2_COMMID		0xc8

#define DB_FRAMETYPE_RTS        1
#define DB_FRAMETYPE_DATA       2
#define DB_FRAMETYPE_BEACON     3
#define DB_FRAMETYPE_DEFAULT    DB_FRAMETYPE_DATA

#define NUM_CHANNELS            14      // max number of channels sent over DroneBridge control module (ground)
#define DB_RC_NUM_CHANNELS      12      // number of channels supported by DroneBridge RC protocol

#define DB_PORT_CONTROLLER  0x01
#define DB_PORT_TELEMETRY   0x02  // deprecated. Use proxy port for bidirectional telemetry
#define DB_PORT_VIDEO       0x03
#define DB_PORT_COMM		0x04
#define DB_PORT_STATUS		0x05
#define DB_PORT_PROXY		0x06
#define DB_PORT_RC			0x07

#define DB_DIREC_DRONE      0x01 // packet to/for drone
#define DB_DIREC_GROUND   	0x03 // packet to/for ground station

#define APP_PORT_STATUS     1602 // for all kinds of status protocol messages. Same port on ground station and app
#define APP_PORT_COMM       1603
#define APP_PORT_TELEMETRY  1604 // accepts MAVLink and LTM telemetry messages. Non MAVLink telemetry messages get rerouted internally to APP_PORT_PROXY
#define PORT_TCP_SYSLOG_SERVER 1605
#define APP_PORT_PROXY 		5760 // use this port for all MAVLink messages (TCP)
#define APP_PORT_PROXY_UDP	14550 // use this port for all MAVLink messages (UDP)
#define APP_PORT_VIDEO      5000 // app accepts raw H.264 streams
#define APP_PORT_VIDEO_FEC  5001 // app accepts raw DroneBridge video stream data, performs FEC on Android device

#define DB_MAVLINK_SYS_ID	69
#define	MAX_PENUMBRA_INTERFACES 8

#define DB_UNIX_DOMAIN_VIDEO_PATH   "/tmp/db_video_out"
#define DB_AP_CLIENT_IP             "192.168.2.1"   // default IP address of GCS connected via WiFi AP

#define DB_SYS_HID_ESP32 1

#endif // DB_PROTOCOL_H_INCLUDED
