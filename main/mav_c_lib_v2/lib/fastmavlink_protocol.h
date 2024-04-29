//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_PROTOCOL_H
#define FASTMAVLINK_PROTOCOL_H


#define FASTMAVLINK_MAVLINK_VERSION   3 // this is the fixed mavlink version, as send in HEARTBEAT


#define FASTMAVLINK_MAGIC_V1          0xFE
#define FASTMAVLINK_MAGIC_V2          0xFD

#define FASTMAVLINK_HEADER_V1_LEN     6
#define FASTMAVLINK_HEADER_V2_LEN     10
#define FASTMAVLINK_PAYLOAD_LEN_MAX   255
#define FASTMAVLINK_CHECKSUM_LEN      2
#define FASTMAVLINK_SIGNATURE_LEN     13

#define FASTMAVLINK_FRAME_LEN_MAX     280 // = HEADER_V2_LEN + PAYLOAD_LEN_MAX + CHECKSUM_LEN + SIGNATURE_LEN


#endif // FASTMAVLINK_PROTOCOL_H


