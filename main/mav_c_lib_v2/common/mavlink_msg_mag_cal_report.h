//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MAG_CAL_REPORT_H
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_H


//----------------------------------------
//-- Message MAG_CAL_REPORT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mag_cal_report_t {
    float fitness;
    float ofs_x;
    float ofs_y;
    float ofs_z;
    float diag_x;
    float diag_y;
    float diag_z;
    float offdiag_x;
    float offdiag_y;
    float offdiag_z;
    uint8_t compass_id;
    uint8_t cal_mask;
    uint8_t cal_status;
    uint8_t autosaved;
    float orientation_confidence;
    uint8_t old_orientation;
    uint8_t new_orientation;
    float scale_factor;
}) fmav_mag_cal_report_t;


#define FASTMAVLINK_MSG_ID_MAG_CAL_REPORT  192

#define FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX  54
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_CRCEXTRA  36

#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FLAGS  0
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FRAME_LEN_MAX  79



#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_FITNESS_OFS  0
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_OFS_X_OFS  4
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_OFS_Y_OFS  8
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_OFS_Z_OFS  12
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_DIAG_X_OFS  16
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_DIAG_Y_OFS  20
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_DIAG_Z_OFS  24
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_OFFDIAG_X_OFS  28
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_OFFDIAG_Y_OFS  32
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_OFFDIAG_Z_OFS  36
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_COMPASS_ID_OFS  40
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_CAL_MASK_OFS  41
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_CAL_STATUS_OFS  42
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_AUTOSAVED_OFS  43
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_ORIENTATION_CONFIDENCE_OFS  44
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_OLD_ORIENTATION_OFS  48
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_NEW_ORIENTATION_OFS  49
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FIELD_SCALE_FACTOR_OFS  50


//----------------------------------------
//-- Message MAG_CAL_REPORT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z, float orientation_confidence, uint8_t old_orientation, uint8_t new_orientation, float scale_factor,
    fmav_status_t* _status)
{
    fmav_mag_cal_report_t* _payload = (fmav_mag_cal_report_t*)_msg->payload;

    _payload->fitness = fitness;
    _payload->ofs_x = ofs_x;
    _payload->ofs_y = ofs_y;
    _payload->ofs_z = ofs_z;
    _payload->diag_x = diag_x;
    _payload->diag_y = diag_y;
    _payload->diag_z = diag_z;
    _payload->offdiag_x = offdiag_x;
    _payload->offdiag_y = offdiag_y;
    _payload->offdiag_z = offdiag_z;
    _payload->compass_id = compass_id;
    _payload->cal_mask = cal_mask;
    _payload->cal_status = cal_status;
    _payload->autosaved = autosaved;
    _payload->orientation_confidence = orientation_confidence;
    _payload->old_orientation = old_orientation;
    _payload->new_orientation = new_orientation;
    _payload->scale_factor = scale_factor;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MAG_CAL_REPORT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MAG_CAL_REPORT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mag_cal_report_pack(
        _msg, sysid, compid,
        _payload->compass_id, _payload->cal_mask, _payload->cal_status, _payload->autosaved, _payload->fitness, _payload->ofs_x, _payload->ofs_y, _payload->ofs_z, _payload->diag_x, _payload->diag_y, _payload->diag_z, _payload->offdiag_x, _payload->offdiag_y, _payload->offdiag_z, _payload->orientation_confidence, _payload->old_orientation, _payload->new_orientation, _payload->scale_factor,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z, float orientation_confidence, uint8_t old_orientation, uint8_t new_orientation, float scale_factor,
    fmav_status_t* _status)
{
    fmav_mag_cal_report_t* _payload = (fmav_mag_cal_report_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->fitness = fitness;
    _payload->ofs_x = ofs_x;
    _payload->ofs_y = ofs_y;
    _payload->ofs_z = ofs_z;
    _payload->diag_x = diag_x;
    _payload->diag_y = diag_y;
    _payload->diag_z = diag_z;
    _payload->offdiag_x = offdiag_x;
    _payload->offdiag_y = offdiag_y;
    _payload->offdiag_z = offdiag_z;
    _payload->compass_id = compass_id;
    _payload->cal_mask = cal_mask;
    _payload->cal_status = cal_status;
    _payload->autosaved = autosaved;
    _payload->orientation_confidence = orientation_confidence;
    _payload->old_orientation = old_orientation;
    _payload->new_orientation = new_orientation;
    _payload->scale_factor = scale_factor;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MAG_CAL_REPORT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MAG_CAL_REPORT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MAG_CAL_REPORT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mag_cal_report_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->compass_id, _payload->cal_mask, _payload->cal_status, _payload->autosaved, _payload->fitness, _payload->ofs_x, _payload->ofs_y, _payload->ofs_z, _payload->diag_x, _payload->diag_y, _payload->diag_z, _payload->offdiag_x, _payload->offdiag_y, _payload->offdiag_z, _payload->orientation_confidence, _payload->old_orientation, _payload->new_orientation, _payload->scale_factor,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z, float orientation_confidence, uint8_t old_orientation, uint8_t new_orientation, float scale_factor,
    fmav_status_t* _status)
{
    fmav_mag_cal_report_t _payload;

    _payload.fitness = fitness;
    _payload.ofs_x = ofs_x;
    _payload.ofs_y = ofs_y;
    _payload.ofs_z = ofs_z;
    _payload.diag_x = diag_x;
    _payload.diag_y = diag_y;
    _payload.diag_z = diag_z;
    _payload.offdiag_x = offdiag_x;
    _payload.offdiag_y = offdiag_y;
    _payload.offdiag_z = offdiag_z;
    _payload.compass_id = compass_id;
    _payload.cal_mask = cal_mask;
    _payload.cal_status = cal_status;
    _payload.autosaved = autosaved;
    _payload.orientation_confidence = orientation_confidence;
    _payload.old_orientation = old_orientation;
    _payload.new_orientation = new_orientation;
    _payload.scale_factor = scale_factor;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MAG_CAL_REPORT,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MAG_CAL_REPORT,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MAG_CAL_REPORT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mag_cal_report_decode(fmav_mag_cal_report_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_fitness(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_ofs_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_ofs_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_ofs_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_diag_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_diag_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_diag_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_offdiag_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_offdiag_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_offdiag_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_report_get_field_compass_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_report_get_field_cal_mask(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[41]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_report_get_field_cal_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_report_get_field_autosaved(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[43]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_orientation_confidence(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_report_get_field_old_orientation(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_report_get_field_new_orientation(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[49]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_report_get_field_scale_factor(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[50]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MAG_CAL_REPORT  192

#define mavlink_mag_cal_report_t  fmav_mag_cal_report_t

#define MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN  54
#define MAVLINK_MSG_ID_MAG_CAL_REPORT_MIN_LEN  44
#define MAVLINK_MSG_ID_192_LEN  54
#define MAVLINK_MSG_ID_192_MIN_LEN  44

#define MAVLINK_MSG_ID_MAG_CAL_REPORT_CRC  36
#define MAVLINK_MSG_ID_192_CRC  36




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mag_cal_report_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z, float orientation_confidence, uint8_t old_orientation, uint8_t new_orientation, float scale_factor)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mag_cal_report_pack(
        _msg, sysid, compid,
        compass_id, cal_mask, cal_status, autosaved, fitness, ofs_x, ofs_y, ofs_z, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z, orientation_confidence, old_orientation, new_orientation, scale_factor,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mag_cal_report_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_mag_cal_report_t* _payload)
{
    return mavlink_msg_mag_cal_report_pack(
        sysid,
        compid,
        _msg,
        _payload->compass_id, _payload->cal_mask, _payload->cal_status, _payload->autosaved, _payload->fitness, _payload->ofs_x, _payload->ofs_y, _payload->ofs_z, _payload->diag_x, _payload->diag_y, _payload->diag_z, _payload->offdiag_x, _payload->offdiag_y, _payload->offdiag_z, _payload->orientation_confidence, _payload->old_orientation, _payload->new_orientation, _payload->scale_factor);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mag_cal_report_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z, float orientation_confidence, uint8_t old_orientation, uint8_t new_orientation, float scale_factor)
{
    return fmav_msg_mag_cal_report_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        compass_id, cal_mask, cal_status, autosaved, fitness, ofs_x, ofs_y, ofs_z, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z, orientation_confidence, old_orientation, new_orientation, scale_factor,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mag_cal_report_decode(const mavlink_message_t* msg, mavlink_mag_cal_report_t* payload)
{
    fmav_msg_mag_cal_report_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MAG_CAL_REPORT_H
