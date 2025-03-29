//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// In the below, all tokens are listed which you can use to affect and modify the
// behavior of the fastMavlink library.
//
// The general usage is that the respective token is defined in the user code before
// the fastMavlink library is included. For instance:
//
// #define FASTMAVLINK_ALWAYS_ZEROFILL  0
// #include "path_to_fastmavlink/xyzdialect/xyzdialect.h"
//------------------------------

#pragma once
#ifndef FASTMAVLINK_CONFIG_H
#define FASTMAVLINK_CONFIG_H


#ifndef FASTMAVLINK_RAM_SECTION
  #define FASTMAVLINK_RAM_SECTION  static
#endif


#ifndef FASTMAVLINK_FUNCTION_DECORATOR
  #define FASTMAVLINK_FUNCTION_DECORATOR  static inline
#endif


#ifndef FASTMAVLINK_PACK
  #ifdef __GNUC__
    #define FASTMAVLINK_PACK( __Declaration__ ) __Declaration__ __attribute__((packed))
  #else
    #define FASTMAVLINK_PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
  #endif
#endif


// Allows to overwrite the value which is reported in the mavlink_version field of the
// HEARTBEAT message. You should usually not need this.
#ifndef FASTMAVLINK_HEARTBEAT_MAVLINK_VERSION
  #define FASTMAVLINK_HEARTBEAT_MAVLINK_VERSION  3
#endif


// Allows to control zero-filling of payloads.
//
// The receive/parser functions always zero-fill the payload, as this is required for
// correct operation of the unpack/decode/get_field functions. The message generator
// functions in contrast, i.e., the pack/encode functions, do not need to do this for
// correct operation when used in the typical use case, which consists of generating
// and then sending the message. However, when unpack/decode/get_field functions are called
// on the generated messages, this can cause malfunction, as they require zero-filled payloads.
// Zero-filling is somewhat costly.
// Hence this flag allows you to control if zero-filling should always be done or not.
// It is enabled (= 1) per default, to prevent unexpected results for the novice. This has
// a performance cost, so disable for best performance and call fmav_msg_zerofill() when it
// is needed.
#ifndef FASTMAVLINK_ALWAYS_ZEROFILL
  #define FASTMAVLINK_ALWAYS_ZEROFILL  1 // always zero-fills payload
#endif


// Allows to modify the message entry structure.
//
// When defined, the msgid field in the fmav_message_entry_t structure is declared as
// uint32_t:24, which makes the fmav_message_entry_t structure exactly 8 bytes wide.
// Depending on the system and compiler (= alignment) this can lead to a much reduced
// RAM footprint for the message entries list, but also can result in larger cpu time.
//
// Per default this token is not defined, which for most cases should be ok.
/*
#define FASTMAVLINK_MESSAGE_ENTRY_MSGID24BIT
*/


// Allows to modify the message entries list.
//
// If only relatively few MAVLink messages are used, efficiency can be much improved, both
// memory and computational time wise, by limiting the message entries to those messages
// which are explicitly required.
//
// This has two effects:
// First, the flash space can be much reduced since not all message entries are stored but
// only those which are needed.
// Second, cpu time for parsing can be much reduced since the list of message entries is
// searched for the received message, and a shorter list obviously relates to fast search time.
// 
// It can be achieved by commenting out in the file xyzdialect_msg_entries.h all those
// message entries which are not needed.
//
// Alternatively, and usually better, one can define one's own FASTMAVLINK_MESSAGE_CRCS
// using the required message entry tokens. It is then MOST important to keep the sequence
// in order since otherwise the default binary search will fail. For instance:
/*
// #include "path_to_fastmavlink/xyzdialect/fmav_msg_entries.h"
// #define FASTMAVLINK_MESSAGE_CRCS {\
//     FASTMAVLINK_MSG_ENTRY_PARAM_REQUEST_READ,\
//     FASTMAVLINK_MSG_ENTRY_PARAM_REQUEST_LIST,\
//     FASTMAVLINK_MSG_ENTRY_PARAM_SET,\
//     FASTMAVLINK_MSG_ENTRY_COMMAND_LONG,\
//     FASTMAVLINK_MSG_ENTRY_AUTOPILOT_VERSION_REQUEST }
// #include "path_to_fastmavlink/xyzdialect/xyzdialect.h"
*/
// Note that (usually) one only needs to include those messages which should be received/parsed,
// since the message entries list is searched only by the parser. Therefore, the number of
// message entries to include can often be surprisingly small.
/*
#define FASTMAVLINK_MESSAGE_CRCS
*/


// Allows to specify a serial handler function to directly write to the Tx buffer.
//
// You need to specify a function void fmav_serial_write_char(uint8_t) in your code
// before including the fastMavlink library.
/*
#define FASTMAVLINK_SERIAL_WRITE_CHAR
*/


// Allows to overwrite the build date of the library. You should usually not need this.
/*
#define FASTMAVLINK_BUILD_DATE
*/


// Allows to overwrite the version specified in the dialect xml file. You should usually
// not need this.
/*
#define FASTMAVLINK_DIALECT_VERSION
*/


// Allows to exclude the enum definitions. You should usually not need this. Can be useful
// e.g. when fastMavlink is used together with pymavlink-mavgen, such as in tests.
/*
#define FASTMAVLINK_EXCLUDE_ENUMS
*/


// Allows to ignore compiler warning "-Waddress-of-packed-member"
#define FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER



// Allows to use a faster lookup table based crc calculation.
/*
#define FASTMAVLINK_CRC_LOOKUP_TABLE
*/


// Allows to reset the parser when a header error is detected.
//
// The MAVLink protocol provides no means to detect errors in the header, which can lead to the successful
// parsing of invalid frames. The incompat and compat flags however can - currently - assume only specific
// values which can be used as heuristics to reset the parser. This increases the chance for the parser to
// synchronize with the data stream.
/*
#define FASTMAVLINK_RESET_PARSER_ON_FLAGS_ERROR
*/


#endif // FASTMAVLINK_CONFIG_H


