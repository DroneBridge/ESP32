//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// API:
// (support functions not listed as they usually shall not be directly used)
//
// receive into frame buf:
//   uint8_t  fmav_parse_to_frame_buf(fmav_result_t* result, uint8_t* buf, fmav_status_t* status, uint8_t c)
//   uint8_t  fmav_check_frame_buf(fmav_result_t* result, uint8_t* buf)
//   void     fmav_frame_buf_to_msg(fmav_message_t* msg, fmav_result_t* result, uint8_t* buf)
//   uint8_t  fmav_parse_and_check_to_frame_buf(fmav_result_t* result, uint8_t* buf, fmav_status_t* status, uint8_t c)
//   uint8_t  fmav_parse_to_msg_wbuf(fmav_message_t* msg, uint8_t* buf, fmav_status_t* status, uint8_t c)
//
// receive into msg:
//   uint8_t  fmav_parse_to_msg(fmav_message_t* msg, fmav_status_t* status, uint8_t c)
//
// send from msg:
//   uint16_t fmav_msg_to_frame_buf(uint8_t* buf, fmav_message_t* msg)
//   uint16_t fmav_msg_to_frame_buf_wresult(fmav_result_t* result, uint8_t* buf, fmav_message_t* msg)
//   uint16_t fmav_msg_to_serial(fmav_message_t* msg)
//
// helper:
//   uint8_t  fmav_msg_is_v2(fmav_message_t* msg)
//   uint16_t fmav_msg_frame_len(fmav_message_t* msg)
//   void     fmav_msg_zerofill(fmav_message_t* msg)
//   uint8_t  fmav_msg_get_target_sysid(fmav_message_t* msg)
//   uint8_t  fmav_msg_get_target_compid(fmav_message_t* msg)
//   uint8_t  fmav_msg_is_for_me(uint8_t my_sysid, uint8_t my_compid, fmav_message_t* msg)
//   uint8_t  fmav_msg_result_is_for_me(uint8_t my_sysid, uint8_t my_compid, fmav_result_t* result)
//   void     fmav_status_reset_rx(fmav_status_t* status)
//   void     fmav_status_reset_tx(fmav_status_t* status)
//   void     fmav_status_reset(fmav_status_t* status)
//   void     fmav_init(void)
//------------------------------

#pragma once
#ifndef FASTMAVLINK_FUNCTIONS_H
#define FASTMAVLINK_FUNCTIONS_H


#include <stdint.h>
#include "../fastmavlink_config.h"
#include "fastmavlink_crc.h"
#include "fastmavlink_protocol.h"
#include "fastmavlink_types.h"


//------------------------------
//-- Support functions
//------------------------------
// const    fmav_message_entry_t* fmav_get_message_entry(uint32_t msgid)
// uint32_t fmav_get_message_entry_num(void)
// const    fmav_message_entry_t* fmav_get_message_entry_by_index(uint32_t i)
// uint8_t  fmav_payload_len_wo_trailing_zeros(const uint8_t* payload, uint8_t len)
// uint16_t fmav_finalize_msg(fmav_message_t* msg, fmav_status_t* status)
// uint16_t fmav_finalize_frame_buf(
//              uint8_t* buf, uint8_t payload_max_len, uint8_t crc_extra, fmav_status_t* status)
// uint16_t fmav_finalize_serial(
//              uint8_t sysid, uint8_t compid, uint8_t* payload,
//              uint32_t msgid, uint8_t payload_max_len, uint8_t crc_extra, fmav_status_t* status)
// uint8_t  fmav_check_msg(fmav_message_t* msg, fmav_status_t* status)
// void     fmav_parse_reset(fmav_status_t* status)

static const fmav_message_entry_t _fmav_message_crcs[] = FASTMAVLINK_MESSAGE_CRCS;


// this function is taken from the pymavlink-mavgen project
// https://github.com/ArduPilot/pymavlink/tree/master/generator
FASTMAVLINK_FUNCTION_DECORATOR const fmav_message_entry_t* fmav_get_message_entry(uint32_t msgid)
{
    //static const fmav_message_entry_t _fmav_message_crcs[] = FASTMAVLINK_MESSAGE_CRCS;
    uint32_t low = 0;
    uint32_t high = sizeof(_fmav_message_crcs)/sizeof(_fmav_message_crcs[0]) - 1;

    while (low < high) {
        uint32_t mid = (low + 1 + high)/2;
        if (msgid < _fmav_message_crcs[mid].msgid) {
            high = mid - 1;
            continue;
        }
        if (msgid > _fmav_message_crcs[mid].msgid) {
            low = mid;
            continue;
        }
        low = mid;
        break;
    }
    if (_fmav_message_crcs[low].msgid != msgid) {
        return NULL;
    }
    return &_fmav_message_crcs[low];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_get_message_entry_num(void)
{
    return sizeof(_fmav_message_crcs)/sizeof(_fmav_message_crcs[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR const fmav_message_entry_t* fmav_get_message_entry_by_index(uint32_t i)
{
    return &(_fmav_message_crcs[i]);
}


// this function is taken from the pymavlink-mavgen project
// https://github.com/ArduPilot/pymavlink/tree/master/generator
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_payload_len_wo_trailing_zeros(const uint8_t* payload, uint8_t len)
{
    while (len > 1 && payload[len-1] == 0) {
        len--;
    }
    return len;
}


// used in message generators to finalize the message fields
// msg payload will not be zero-filled
// we thus should use the generators only to send, not to digest
// call helper fmav_msg_zerofill() afterwards to zero-fill if really needed
// or set the flag FASTMAVLINK_ALWAYS_ZEROFILL to 1
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_finalize_msg(fmav_message_t* msg, fmav_status_t* status)
{
    uint8_t headbuf[FASTMAVLINK_HEADER_V2_LEN];

    headbuf[0] = msg->magic = FASTMAVLINK_MAGIC_V2;
    headbuf[1] = msg->len = fmav_payload_len_wo_trailing_zeros(msg->payload, msg->payload_max_len); // msg->payload_max_len was set in generator
    headbuf[2] = msg->incompat_flags = 0;
    headbuf[3] = msg->compat_flags = 0;
    headbuf[4] = msg->seq = status->tx_seq;
    status->tx_seq++;
    headbuf[5] = msg->sysid; // msg->sysid was set in generator
    headbuf[6] = msg->compid; // msg->compid was set in generator
    headbuf[7] = (uint8_t)msg->msgid; // msg->msgid was set in generator
    headbuf[8] = (uint8_t)((msg->msgid) >> 8);
    headbuf[9] = (uint8_t)((msg->msgid) >> 16);

    uint16_t crc = fmav_crc_calculate(&(headbuf[1]), FASTMAVLINK_HEADER_V2_LEN - 1);
    fmav_crc_accumulate_buf(&crc, msg->payload, msg->len);
    fmav_crc_accumulate(&crc, msg->crc_extra); // msg->crc_extra was set in generator
    msg->checksum = crc;

    // msg->target_sysid was set in generator
    // msg->target_compid was set in generator
    // msg->crc_extra was set in generator
    // msg->payload_max_len was set in generator

#if FASTMAVLINK_ALWAYS_ZEROFILL
    // ensure that payload is zero-filled
    if (msg->len < msg->payload_max_len) {
        memset(&(msg->payload[msg->len]), 0, msg->payload_max_len - msg->len);
    }
#endif

    return (uint16_t)msg->len + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;
}


// used in message generators to finalize the frame buf
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_finalize_frame_buf(
    uint8_t* buf, uint8_t payload_max_len, uint8_t crc_extra, fmav_status_t* status)
{
    uint16_t len = fmav_payload_len_wo_trailing_zeros(&buf[10], payload_max_len);

    buf[0] = FASTMAVLINK_MAGIC_V2;
    buf[1] = (uint8_t)len;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = status->tx_seq;
    status->tx_seq++;
    // buf[5] // sysid was set in generator
    // buf[6] // compid was set in generator
    // buf[7] // msgid was set in generator
    // buf[8]
    // buf[9]

    uint16_t crc = fmav_crc_calculate(&(buf[1]), FASTMAVLINK_HEADER_V2_LEN - 1);
    fmav_crc_accumulate_buf(&crc, &buf[10], len);
    fmav_crc_accumulate(&crc, crc_extra); // msg->crc_extra was set in generator

    buf[len + FASTMAVLINK_HEADER_V2_LEN] = (uint8_t)crc;
    buf[len + FASTMAVLINK_HEADER_V2_LEN + 1] = (uint8_t)(crc >> 8);

    return len + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

// used in message generators
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_finalize_serial(
    uint8_t sysid, uint8_t compid, uint8_t* payload,
    uint32_t msgid, uint8_t payload_max_len, uint8_t crc_extra, fmav_status_t* status)
{
    uint16_t len = fmav_payload_len_wo_trailing_zeros(payload, payload_max_len);
    uint16_t crc;
    fmav_crc_init(&crc);

    fmav_serial_write_char(FASTMAVLINK_MAGIC_V2);
    fmav_serial_write_char(len);  fmav_crc_accumulate(&crc, len);
    fmav_serial_write_char(0);  fmav_crc_accumulate(&crc, 0);
    fmav_serial_write_char(0);  fmav_crc_accumulate(&crc, 0);
    fmav_serial_write_char(status->tx_seq);  fmav_crc_accumulate(&crc, status->tx_seq);
    status->tx_seq++;
    fmav_serial_write_char(sysid);  fmav_crc_accumulate(&crc, sysid);
    fmav_serial_write_char(compid);  fmav_crc_accumulate(&crc, compid);
    fmav_serial_write_char(msgid);  fmav_crc_accumulate(&crc, msgid);
    fmav_serial_write_char(msgid >> 8);  fmav_crc_accumulate(&crc, msgid >> 8);
    fmav_serial_write_char(msgid >> 16);  fmav_crc_accumulate(&crc, msgid >> 16);

    for(uint16_t i = 0; i < len; i++) {
        fmav_serial_write_char(payload[i]);  fmav_crc_accumulate(&crc, payload[i]);
    }

    fmav_crc_accumulate(&crc, crc_extra);

    fmav_serial_write_char(crc);
    fmav_serial_write_char(crc >> 8);

    return len + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;
}

#endif


// used in fmav_parse_to_msg() below
// returns MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
// msg payload will be zero-filled, so that fmav_parse_to_msg() returns proper msg
// the checks are only possible if msgid is known, and also the targets are only then
// known, else we need to assume broadcast
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_check_msg(fmav_message_t* msg, fmav_status_t* status)
{
    msg->target_sysid = 0;
    msg->target_compid = 0;

    const fmav_message_entry_t* msg_entry = fmav_get_message_entry(msg->msgid);
    if (!msg_entry) {
        msg->res = FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN;
        return FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN;
    }

    // check length
// note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
//    if (msg->len > msg_entry->payload_max_len) {
//        msg->res = FASTMAVLINK_PARSE_RESULT_LENGTH_ERROR;
//        return FASTMAVLINK_PARSE_RESULT_LENGTH_ERROR;
//    }

    // check crc
    fmav_crc_accumulate(&(status->rx_crc), msg_entry->crc_extra);
    if (status->rx_crc != msg->checksum) {
        msg->res = FASTMAVLINK_PARSE_RESULT_CRC_ERROR;
        return FASTMAVLINK_PARSE_RESULT_CRC_ERROR;
    }

    // zero-fill msg payload
    // this among others also ensures that the target fields are set properly
    if (msg->len < msg_entry->payload_max_len) {
        memset(&(msg->payload[msg->len]), 0, msg_entry->payload_max_len - msg->len);
    }

    // get further metadata
    if (msg_entry->flags & FASTMAVLINK_MESSAGE_ENTRY_FLAGS_HAS_TARGET_SYSTEM) {
        msg->target_sysid = msg->payload[msg_entry->target_system_ofs];
    }
    if (msg_entry->flags & FASTMAVLINK_MESSAGE_ENTRY_FLAGS_HAS_TARGET_COMPONENT) {
        msg->target_compid = msg->payload[msg_entry->target_component_ofs];
    }
    msg->crc_extra = msg_entry->crc_extra;
    msg->payload_max_len = msg_entry->payload_max_len;

    msg->res = FASTMAVLINK_PARSE_RESULT_OK;
    return FASTMAVLINK_PARSE_RESULT_OK;
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_parse_reset(fmav_status_t* status)
{
    status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE;
    status->rx_cnt = 0;
    status->rx_header_len = 0;
    status->rx_frame_len = 0;
}


//------------------------------
//-- Receive handlers
//------------------------------
// uint8_t  fmav_parse_to_frame_buf(fmav_result_t* result, uint8_t* buf, fmav_status_t* status, uint8_t c)
// uint8_t  fmav_check_frame_buf(fmav_result_t* result, uint8_t* buf)
// void     fmav_frame_buf_to_msg(fmav_message_t* msg, fmav_result_t* result, uint8_t* buf)
// uint8_t  fmav_parse_and_check_to_frame_buf(fmav_result_t* result, uint8_t* buf, fmav_status_t* status, uint8_t c)
// uint8_t  fmav_parse_to_msg_wbuf(fmav_message_t* msg, uint8_t* buf, fmav_status_t* status, uint8_t c)
// uint8_t  fmav_parse_to_msg(fmav_message_t* msg, fmav_status_t* status, uint8_t c)

// returns NONE, HAS_HEADER, or OK
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_parse_to_frame_buf(fmav_result_t* result, uint8_t* buf, fmav_status_t* status, uint8_t c)
{
    if (status->rx_cnt >= FASTMAVLINK_FRAME_LEN_MAX) { // this should never happen, but play it safe
        status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE;
    }

    switch (status->rx_state) {
    case FASTMAVLINK_PARSE_STATE_IDLE:
        status->rx_cnt = 0;
        if (c == FASTMAVLINK_MAGIC_V2 || c == FASTMAVLINK_MAGIC_V1) {
            buf[status->rx_cnt++] = c;
            status->rx_state = FASTMAVLINK_PARSE_STATE_LEN;
        }
        result->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_LEN:
        buf[status->rx_cnt++] = c;
        if (buf[0] == FASTMAVLINK_MAGIC_V2) {
            status->rx_header_len = FASTMAVLINK_HEADER_V2_LEN;
            status->rx_frame_len = (uint16_t)c + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;
            status->rx_state = FASTMAVLINK_PARSE_STATE_INCOMPAT_FLAGS;
        } else {
            status->rx_header_len = FASTMAVLINK_HEADER_V1_LEN;
            status->rx_frame_len = (uint16_t)c + FASTMAVLINK_HEADER_V1_LEN + FASTMAVLINK_CHECKSUM_LEN;
            status->rx_state = FASTMAVLINK_FASTPARSE_STATE_FRAME;
        }
        result->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

#ifndef FASTMAVLINK_RESET_PARSER_ON_FLAGS_ERROR
    case FASTMAVLINK_PARSE_STATE_INCOMPAT_FLAGS:
        buf[status->rx_cnt++] = c;
        if (c & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
            status->rx_frame_len += FASTMAVLINK_SIGNATURE_LEN;
        }
        status->rx_state = FASTMAVLINK_FASTPARSE_STATE_FRAME;
        result->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;
#else
    case FASTMAVLINK_PARSE_STATE_INCOMPAT_FLAGS:
        buf[status->rx_cnt++] = c;
        if (c & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
            status->rx_frame_len += FASTMAVLINK_SIGNATURE_LEN;
        }
        status->rx_state = FASTMAVLINK_PARSE_STATE_COMPAT_FLAGS;
        if (c != 0 && c != FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE; // incorrect incompat flag, so reset
        }
        result->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_COMPAT_FLAGS:
        buf[status->rx_cnt++] = c;
        status->rx_state = FASTMAVLINK_FASTPARSE_STATE_FRAME;
        if (c != 0) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE; // incorrect compat flag, so reset
        }
        result->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;
#endif

    case FASTMAVLINK_FASTPARSE_STATE_FRAME:
        buf[status->rx_cnt++] = c;
        if (status->rx_cnt >= status->rx_frame_len) {
            status->rx_cnt = 0;
            status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE;
            result->res = FASTMAVLINK_PARSE_RESULT_OK;
            result->frame_len = status->rx_frame_len;
            return FASTMAVLINK_PARSE_RESULT_OK;
        }
        if (status->rx_cnt >= status->rx_header_len) {
            result->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
            return FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        }
        result->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;
    }

    // should never get to here
    fmav_parse_reset(status);
    result->res = FASTMAVLINK_PARSE_RESULT_NONE;
    return FASTMAVLINK_PARSE_RESULT_NONE;
}


// returns MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
// the checks are only possible if msgid is known, and also the targets are only then
// known, else we need to assume broadcast
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_check_frame_buf(fmav_result_t* result, uint8_t* buf)
{
    if (buf[0] == FASTMAVLINK_MAGIC_V2) {
        result->sysid = buf[5];
        result->compid = buf[6];
        result->msgid = buf[7] + ((uint32_t)buf[8] << 8) + ((uint32_t)buf[9] << 16);
        result->frame_len = (uint16_t)buf[1] + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;
        if (buf[2] & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) result->frame_len += FASTMAVLINK_SIGNATURE_LEN;
    } else {
        result->sysid = buf[3];
        result->compid = buf[4];
        result->msgid = buf[5];
        result->frame_len = (uint16_t)buf[1] + FASTMAVLINK_HEADER_V1_LEN + FASTMAVLINK_CHECKSUM_LEN;
    }

    result->target_sysid = 0;
    result->target_compid = 0;

    const fmav_message_entry_t* msg_entry = fmav_get_message_entry(result->msgid);
    if (!msg_entry) {
        result->res = FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN;
        return FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN;
    }

    // check length
// note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
//    if (buf[1] > msg_entry->payload_max_len) {
//        result->res = FASTMAVLINK_PARSE_RESULT_LENGTH_ERROR;
//        return FASTMAVLINK_PARSE_RESULT_LENGTH_ERROR;
//    }

    // we are going to need this twice
    uint16_t payload_pos = (buf[0] == FASTMAVLINK_MAGIC_V2) ? FASTMAVLINK_HEADER_V2_LEN : FASTMAVLINK_HEADER_V1_LEN;

    // check crc
    uint16_t crc_pos = payload_pos + buf[1];
    uint16_t crc_frame = fmav_crc_calculate(&(buf[1]), crc_pos - 1); // the magic byte is not crc'ed
    fmav_crc_accumulate(&crc_frame, msg_entry->crc_extra);
    uint16_t crc_msg = buf[crc_pos] + ((uint16_t)buf[crc_pos+1] << 8);
    if (crc_frame != crc_msg) {
        result->res = FASTMAVLINK_PARSE_RESULT_CRC_ERROR;
        return FASTMAVLINK_PARSE_RESULT_CRC_ERROR;
    }

    // get further metadata
    if (msg_entry->flags & FASTMAVLINK_MESSAGE_ENTRY_FLAGS_HAS_TARGET_SYSTEM) {
        // due to v2 zero trimming this field may not be in frame buffer, so check
        if (buf[1] > msg_entry->target_system_ofs) {
            result->target_sysid = buf[payload_pos + msg_entry->target_system_ofs];
        }
    }
    if (msg_entry->flags & FASTMAVLINK_MESSAGE_ENTRY_FLAGS_HAS_TARGET_COMPONENT) {
        // due to v2 zero trimming this field may not be in frame buffer, so check
        if (buf[1] > msg_entry->target_component_ofs) {
            result->target_compid = buf[payload_pos + msg_entry->target_component_ofs];
        }
    }
    result->crc_extra = msg_entry->crc_extra;
    result->payload_max_len = msg_entry->payload_max_len;

    result->res = FASTMAVLINK_PARSE_RESULT_OK;
    return FASTMAVLINK_PARSE_RESULT_OK;
}


// should be called only if parse result is OK
// msg payload is zero-filled
FASTMAVLINK_FUNCTION_DECORATOR void fmav_frame_buf_to_msg(fmav_message_t* msg, fmav_result_t* result, uint8_t* buf)
{
    uint16_t pos = 0;

    msg->magic = buf[0];
    msg->len = buf[1];
    if (msg->magic == FASTMAVLINK_MAGIC_V2) {
        msg->incompat_flags = buf[2];
        msg->compat_flags = buf[3];
        msg->seq = buf[4];
        msg->sysid = buf[5];
        msg->compid = buf[6];
        msg->msgid = buf[7] + ((uint32_t)buf[8] << 8) + ((uint32_t)buf[9] << 16);
        pos = 10;
    } else {
        msg->incompat_flags = 0;
        msg->compat_flags = 0;
        msg->seq = buf[2];
        msg->sysid = buf[3];
        msg->compid = buf[4];
        msg->msgid = buf[5];
        pos = 6;
    }
    memcpy(msg->payload, &(buf[pos]), msg->len);
    if (msg->len < result->payload_max_len) {
        memset(&(msg->payload[msg->len]), 0, result->payload_max_len - msg->len); // zero-fill
    }
    pos += msg->len;
    msg->checksum = buf[pos] + ((uint16_t)buf[pos+1] << 8);
    pos += FASTMAVLINK_CHECKSUM_LEN;
    if (msg->incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
        memcpy(msg->signature_a, &(buf[pos]), FASTMAVLINK_SIGNATURE_LEN);
        pos += FASTMAVLINK_SIGNATURE_LEN;
    }

    msg->target_sysid = result->target_sysid;
    msg->target_compid = result->target_compid;
    msg->crc_extra = result->crc_extra;
    msg->payload_max_len = result->payload_max_len;

    msg->res = result->res;
}


// convenience wrapper
// returns 0, or 1
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_parse_and_check_to_frame_buf(fmav_result_t* result, uint8_t* buf, fmav_status_t* status, uint8_t c)
{
    uint8_t res;

    res = fmav_parse_to_frame_buf(result, buf, status, c);
    // result can be NONE, HAS_HEADER, or OK
    if (res != FASTMAVLINK_PARSE_RESULT_OK) return 0;

    res = fmav_check_frame_buf(result, buf);
    // result can be MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
    if (res == FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN || res == FASTMAVLINK_PARSE_RESULT_OK) {
        return 1;
    }

    return 0;
}


// convenience wrapper
// returns 0, or 1
// msg payload is zero-filled
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_parse_to_msg_wbuf(fmav_message_t* msg, uint8_t* buf, fmav_status_t* status, uint8_t c)
{
    uint8_t res;
    fmav_result_t result;

    res = fmav_parse_to_frame_buf(&result, buf, status, c);
    msg->res = result.res;
    // result can be NONE, HAS_HEADER, or OK
    if (res != FASTMAVLINK_PARSE_RESULT_OK) return 0;

    res = fmav_check_frame_buf(&result, buf);
    msg->res = result.res;
    // result can be MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
    if (res != FASTMAVLINK_PARSE_RESULT_OK) return 0;

    fmav_frame_buf_to_msg(msg, &result, buf);

    return 1;
}


// returns NONE, HAS_HEADER, MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
// msg payload is zero-filled
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_parse_to_msg(fmav_message_t* msg, fmav_status_t* status, uint8_t c)
{
    if (status->rx_cnt >= FASTMAVLINK_FRAME_LEN_MAX) { // this should never happen, but play it safe
        status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE;
    }

    switch (status->rx_state) {
    case FASTMAVLINK_PARSE_STATE_IDLE:
        status->rx_cnt = 0;
        if (c == FASTMAVLINK_MAGIC_V2 || c == FASTMAVLINK_MAGIC_V1) {
            msg->magic = c;
            status->rx_cnt++;
            status->rx_state = FASTMAVLINK_PARSE_STATE_LEN;
        }
        msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_LEN:
        msg->len = c;
        status->rx_cnt++;
        fmav_crc_init(&(status->rx_crc));
        fmav_crc_accumulate(&(status->rx_crc), c);
        if (msg->magic == FASTMAVLINK_MAGIC_V2) {
            status->rx_header_len = FASTMAVLINK_HEADER_V2_LEN;
            status->rx_frame_len = (uint16_t)c + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;
            status->rx_state = FASTMAVLINK_PARSE_STATE_INCOMPAT_FLAGS;
        } else {
            msg->incompat_flags = 0;
            msg->compat_flags = 0;
            status->rx_header_len = FASTMAVLINK_HEADER_V1_LEN;
            status->rx_frame_len = (uint16_t)c + FASTMAVLINK_HEADER_V1_LEN + FASTMAVLINK_CHECKSUM_LEN;
            status->rx_state = FASTMAVLINK_PARSE_STATE_SEQ;
        }
        msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_INCOMPAT_FLAGS:
        msg->incompat_flags = c;
        status->rx_cnt++;
        fmav_crc_accumulate(&(status->rx_crc), c);
        if (msg->incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
            status->rx_frame_len += FASTMAVLINK_SIGNATURE_LEN;
        }
        status->rx_state = FASTMAVLINK_PARSE_STATE_COMPAT_FLAGS;
#ifdef FASTMAVLINK_RESET_PARSER_ON_FLAGS_ERROR
        if (c != 0 && c != FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE; // incorrect incompat flag, so reset
        }
#endif
        msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_COMPAT_FLAGS:
        msg->compat_flags = c;
        status->rx_cnt++;
        fmav_crc_accumulate(&(status->rx_crc), c);
        status->rx_state = FASTMAVLINK_PARSE_STATE_SEQ;
#ifdef FASTMAVLINK_RESET_PARSER_ON_FLAGS_ERROR
        if (c != 0) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE; // incorrect incompat flag, so reset
        }
#endif
        msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_SEQ:
        msg->seq = c;
        status->rx_cnt++;
        fmav_crc_accumulate(&(status->rx_crc), c);
        status->rx_state = FASTMAVLINK_PARSE_STATE_SYSID;
        msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_SYSID:
        msg->sysid = c;
        status->rx_cnt++;
        fmav_crc_accumulate(&(status->rx_crc), c);
        status->rx_state = FASTMAVLINK_PARSE_STATE_COMPID;
        msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_COMPID:
        msg->compid = c;
        status->rx_cnt++;
        fmav_crc_accumulate(&(status->rx_crc), c);
        status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_1;
        msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_MSGID_1:
        msg->msgid = c;
        status->rx_cnt++;
        fmav_crc_accumulate(&(status->rx_crc), c);
        if (msg->magic == FASTMAVLINK_MAGIC_V2) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_2;
            msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
            return FASTMAVLINK_PARSE_RESULT_NONE;
        }
        status->rx_state = FASTMAVLINK_PARSE_STATE_PAYLOAD;
        msg->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        return FASTMAVLINK_PARSE_RESULT_HAS_HEADER;

    case FASTMAVLINK_PARSE_STATE_MSGID_2:
        msg->msgid += ((uint32_t)c << 8);
        status->rx_cnt++;
        fmav_crc_accumulate(&(status->rx_crc), c);
        status->rx_state = FASTMAVLINK_PARSE_STATE_MSGID_3;
        msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
        return FASTMAVLINK_PARSE_RESULT_NONE;

    case FASTMAVLINK_PARSE_STATE_MSGID_3:
        msg->msgid += ((uint32_t)c << 16);
        status->rx_cnt++;
        fmav_crc_accumulate(&(status->rx_crc), c);
        status->rx_state = FASTMAVLINK_PARSE_STATE_PAYLOAD;
        msg->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        return FASTMAVLINK_PARSE_RESULT_HAS_HEADER;

    case FASTMAVLINK_PARSE_STATE_PAYLOAD:
        msg->payload[status->rx_cnt - status->rx_header_len] = c;
        status->rx_cnt++;
        fmav_crc_accumulate(&(status->rx_crc), c);
        if (status->rx_cnt >= status->rx_header_len + msg->len) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_CHECKSUM_1;
        }
        msg->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        return FASTMAVLINK_PARSE_RESULT_HAS_HEADER;

    case FASTMAVLINK_PARSE_STATE_CHECKSUM_1:
        msg->checksum = c;
        status->rx_cnt++;
        status->rx_state = FASTMAVLINK_PARSE_STATE_CHECKSUM_2;
        msg->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        return FASTMAVLINK_PARSE_RESULT_HAS_HEADER;

    case FASTMAVLINK_PARSE_STATE_CHECKSUM_2:
        msg->checksum += ((uint16_t)c << 8);
        status->rx_cnt++;
        if (msg->incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
            status->rx_state = FASTMAVLINK_PARSE_STATE_SIGNATURE;
            msg->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
            return FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        }
        status->rx_cnt = 0;
        status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE;
        fmav_check_msg(msg, status); // also zero-fills msg payload
        return msg->res;

    case FASTMAVLINK_PARSE_STATE_SIGNATURE:
        status->rx_cnt++;
        if (status->rx_cnt < status->rx_frame_len) {
            msg->res = FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
            return FASTMAVLINK_PARSE_RESULT_HAS_HEADER;
        }
        status->rx_cnt = 0;
        status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE;
        fmav_check_msg(msg, status); // also zero-fills msg payload
        return msg->res;
    }

    // should never get to here
    fmav_parse_reset(status);
    msg->res = FASTMAVLINK_PARSE_RESULT_NONE;
    return FASTMAVLINK_PARSE_RESULT_NONE;
}


//------------------------------
//-- Send handlers
//------------------------------
// uint16_t fmav_msg_to_frame_buf(uint8_t* buf, fmav_message_t* msg)
// uint16_t fmav_msg_to_frame_buf_wresult(fmav_result_t* result, uint8_t* buf, fmav_message_t* msg)
// uint16_t fmav_msg_to_serial(fmav_message_t* msg)

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_to_frame_buf(uint8_t* buf, fmav_message_t* msg)
{
    uint16_t pos = 0;

    buf[0] = msg->magic;
    buf[1] = msg->len;
    if (msg->magic == FASTMAVLINK_MAGIC_V2) {
        buf[2] = msg->incompat_flags;
        buf[3] = msg->compat_flags;
        buf[4] = msg->seq;
        buf[5] = msg->sysid;
        buf[6] = msg->compid;
        buf[7] = (uint8_t)msg->msgid;
        buf[8] = (uint8_t)((msg->msgid) >> 8);
        buf[9] = (uint8_t)((msg->msgid) >> 16);
        pos = 10;
    } else {
        msg->incompat_flags = 0; // should be already so, but play it safe
        buf[2] = msg->seq;
        buf[3] = msg->sysid;
        buf[4] = msg->compid;
        buf[5] = (uint8_t)msg->msgid;
        pos = 6;
    }
    memcpy(&(buf[pos]), msg->payload, msg->len);
    pos += msg->len;
    buf[pos++] = (uint8_t)msg->checksum;
    buf[pos++] = (uint8_t)((msg->checksum) >> 8);
    if (msg->incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
        memcpy(&(buf[pos]), msg->signature_a, FASTMAVLINK_SIGNATURE_LEN);
        pos += FASTMAVLINK_SIGNATURE_LEN;
    }

    return pos;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_to_frame_buf_wresult(fmav_result_t* result, uint8_t* buf, fmav_message_t* msg)
{
    result->frame_len = fmav_msg_to_frame_buf(buf, msg);

    result->msgid = msg->msgid;
    result->sysid = msg->sysid;
    result->compid = msg->compid;
    result->target_sysid = msg->target_sysid;
    result->target_compid = msg->target_compid;
    result->crc_extra = msg->crc_extra;
    result->payload_max_len = msg->payload_max_len;

    result->res = msg->res;

    return result->frame_len;
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_to_serial(fmav_message_t* msg)
{
    uint16_t pos = 0;

    fmav_serial_write_char(msg->magic);
    fmav_serial_write_char(msg->len);
    if (msg->magic == FASTMAVLINK_MAGIC_V2) {
        fmav_serial_write_char(msg->incompat_flags);
        fmav_serial_write_char(msg->compat_flags);
        fmav_serial_write_char(msg->seq);
        fmav_serial_write_char(msg->sysid);
        fmav_serial_write_char(msg->compid);
        fmav_serial_write_char(msg->msgid);
        fmav_serial_write_char((msg->msgid) >> 8);
        fmav_serial_write_char((msg->msgid) >> 16);
        pos = 10;
    } else {
        msg->incompat_flags = 0; // should be already so, but play it safe
        fmav_serial_write_char(msg->seq);
        fmav_serial_write_char(msg->sysid);
        fmav_serial_write_char(msg->compid);
        fmav_serial_write_char(msg->msgid);
        pos = 6;
    }
    for (uint16_t i = 0; i < msg->len; i++) {
        fmav_serial_write_char(msg->payload[i]);
    }
    pos += msg->len;
    fmav_serial_write_char(msg->checksum);
    fmav_serial_write_char((msg->checksum) >> 8);
    if (msg->incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
        for (uint16_t i = 0; i < FASTMAVLINK_SIGNATURE_LEN; i++) {
            fmav_serial_write_char(msg->signature_a[i]);
        }
        pos += FASTMAVLINK_SIGNATURE_LEN;
    }

    return pos;
}

#endif


//------------------------------
//-- Helper functions
//------------------------------
// uint8_t  fmav_msg_is_v2(fmav_message_t* msg)
// uint16_t fmav_msg_frame_len(fmav_message_t* msg)
// void     fmav_msg_zerofill(fmav_message_t* msg)
// uint8_t  fmav_msg_get_target_sysid(fmav_message_t* msg)
// uint8_t  fmav_msg_get_target_compid(fmav_message_t* msg)
// uint8_t  fmav_msg_is_for_me(uint8_t my_sysid, uint8_t my_compid, fmav_message_t* msg)
// uint8_t  fmav_msg_result_is_for_me(uint8_t my_sysid, uint8_t my_compid, fmav_result_t* result)

FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_is_v2(fmav_message_t* msg)
{
    return (msg->magic == FASTMAVLINK_MAGIC_V2) ? 1 : 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_frame_len(fmav_message_t* msg)
{
    return (uint16_t)msg->len +
           ((msg->magic == FASTMAVLINK_MAGIC_V2) ? FASTMAVLINK_HEADER_V2_LEN : FASTMAVLINK_HEADER_V1_LEN) +
           FASTMAVLINK_CHECKSUM_LEN +
           ((msg->incompat_flags & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) ? FASTMAVLINK_SIGNATURE_LEN : 0);
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_zerofill(fmav_message_t* msg)
{
    if (msg->len < msg->payload_max_len) {
        memset(&(msg->payload[msg->len]), 0, msg->payload_max_len - msg->len);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_get_target_sysid(fmav_message_t* msg)
{
    return msg->target_sysid;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_get_target_compid(fmav_message_t* msg)
{
    return msg->target_compid;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_is_for_me(uint8_t my_sysid, uint8_t my_compid, fmav_message_t* msg)
{
    // the message either has no target_sysid or is broadcast, so accept
    if (msg->target_sysid == 0) return 1;

    // the message has a target_sysid but it is not ours, so reject
    if (msg->target_sysid != my_sysid) return 0;

    // the message either has no target_compid or is broadcast, so accept
    if (msg->target_compid == 0) return 1;

    // the message has a target_compid and it is ours, so accept
    if (msg->target_compid == my_compid) return 1;

    // the message has a target_compid but it is not ours, so reject
    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_result_is_for_me(uint8_t my_sysid, uint8_t my_compid, fmav_result_t* result)
{
    if (result->target_sysid == 0) return 1;
    if (result->target_sysid != my_sysid) return 0;
    if (result->target_compid == 0) return 1;
    if (result->target_compid == my_compid) return 1;
    return 0;
}


//------------------------------
//-- Init
//------------------------------
// void fmav_status_reset_rx(fmav_status_t* status)
// void fmav_status_reset_tx(fmav_status_t* status)
// void fmav_status_reset(fmav_status_t* status)
// void fmav_init(void)

FASTMAVLINK_FUNCTION_DECORATOR void fmav_status_reset_rx(fmav_status_t* status)
{
    status->rx_state = FASTMAVLINK_PARSE_STATE_IDLE;
    status->rx_cnt = 0;
    status->rx_header_len = 0;
    status->rx_frame_len = 0;
    status->rx_crc = 0;
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_status_reset_tx(fmav_status_t* status)
{
    status->tx_seq = 0;
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_status_reset(fmav_status_t* status)
{
    fmav_status_reset_rx(status);
    fmav_status_reset_tx(status);
}


// call it once before using the library, even if it does not do anything currently
FASTMAVLINK_FUNCTION_DECORATOR void fmav_init(void)
{
}


#endif // FASTMAVLINK_FUNCTIONS_H
