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

#ifndef DB_PROTOCOL_H_INCLUDED
#define DB_PROTOCOL_H_INCLUDED

#define RADIOTAP_LENGTH         13
#define DB_RAW_V2_HEADER_LENGTH 10

#define MSP_DATA_LENTH          34      // size of MSP v1
#define MSP_V2_DATA_LENGTH      37      // size of MSP v2 frame
#define DB_RC_DATA_LENGTH		16		// size of DB_RC frame
#define DATA_UNI_LENGTH         2048	// max frame length
#define ETHER_TYPE              0x88ab

#define DEFAULT_DB_MODE         'm'
#define DEFAULT_DB_IF           "18a6f716a511"
#define DEFAULT_V2_COMMID		0x01
#define DEFAULT_BITRATE_OPTION  4

#define NUM_CHANNELS            14      // max number of channels sent over DroneBridge control module (ground)
#define DB_RC_NUM_CHANNELS      12      // number of channels supported by DroneBridge RC protocol

#define DB_PORT_CONTROLLER  0x01
#define DB_PORT_TELEMETRY   0x02
#define DB_PORT_VIDEO       0x03
#define DB_PORT_COMM		0x04
#define DB_PORT_STATUS		0x05
#define DB_PORT_PROXY		0x06
#define DB_PORT_RC			0x07

#define DB_DIREC_DRONE      0x01 // packet to/for drone
#define DB_DIREC_GROUND   	0x03 // packet to/for groundstation

#define APP_PORT_COMM       1603
#define APP_PORT_TELEMETRY  1604 // accepts MAVLink and LTM telemetry messages. Non MAVLink telemetry messages get rerouted internally to APP_PORT_PROXY
#define APP_PORT_PROXY 		1607 // use this port for all non telemetry MAVLink messages and all MSP messages
#define APP_PORT_STATUS     1608


struct data_uni{
	uint8_t bytes[DATA_UNI_LENGTH];
};
struct data_rc_status_update {
	int8_t bytes[6]; // [0] = rssi drone side; [1] = packets_received_per_second; [rest] = to make it long enough!?
};
struct radiotap_header {
	uint8_t bytes[RADIOTAP_LENGTH];
};
// DroneBridge raw protocol v1 header (deprecated)
struct db_80211_header {
	uint8_t frame_control_field[4];
	uint8_t odd;
	uint8_t direction_dstmac;
	uint8_t comm_id[4];
	uint8_t src_mac_bytes[6];
	uint8_t version_bytes;
	uint8_t port_bytes;
	uint8_t direction_bytes;
	uint8_t payload_length_bytes[2];
	uint8_t crc_bytes;
	uint8_t undefined[2];
};
// DroneBridge raw protocol v2 header
struct db_raw_v2_header {
	uint8_t fcf_duration[4];
	uint8_t direction;
	uint8_t comm_id;
	uint8_t port;
	uint8_t payload_length[2];
	uint8_t seq_num;
};

typedef struct {
	uint8_t ident[2];
	uint8_t message_id;
	uint8_t mode;
	uint8_t packetloss_rc; // [%]
    int8_t rssi_drone;
	int8_t rssi_ground;
	uint32_t damaged_blocks_wbc;
	uint32_t lost_packets_wbc;
	uint32_t kbitrate_wbc;
	uint8_t crc;
} __attribute__((packed)) DB_SYSTEM_STATUS_MESSAGE;

typedef struct {
	uint8_t ident[2];
	uint8_t message_id;
	uint16_t channels[NUM_CHANNELS];
} __attribute__((packed)) DB_RC_STATUS_MESSAGE;

typedef struct {
	uint16_t ch[NUM_CHANNELS];
} __attribute__((packed)) db_rc_values;

#endif // DB_PROTOCOL_H_INCLUDED
