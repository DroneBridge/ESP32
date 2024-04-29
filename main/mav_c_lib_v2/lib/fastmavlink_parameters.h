//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// API:
//
// uint8_t fmav_param_value2union(fmav_param_union_t* param_union, float* param_value, uint8_t type)
// uint8_t fmav_param_union2value(float* param_value, fmav_param_union_t* param_union)
//
// functions working on a user-supplied fmav_param_list 
// uint8_t fmav_param_get_param_union(fmav_param_union_t* param_union, uint16_t index)
// uint8_t fmav_param_set_value(uint16_t index, float value)
// uint8_t fmav_param_get_param_value(fmav_param_value_t* payload, uint16_t index)
// uint8_t fmav_param_do_param_request_read(uint16_t* index, fmav_param_request_read_t* payload)
// uint8_t fmav_param_do_param_set(uint16_t* index, fmav_param_set_t* payload)
//------------------------------

#pragma once
#ifndef FASTMAVLINK_PARAMETERS_H
#define FASTMAVLINK_PARAMETERS_H

#ifndef FASTMAVLINK_PARAM_NUM
#error For fastmavlink_parameters.h, FASTMAVLINK_PARAM_NUM and fmav_param_list needs to be defined
#endif

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include "../fastmavlink_config.h"
#include "fastmavlink_types.h"


//------------------------------
//-- Defines
//------------------------------

#define FASTMAVLINK_PARAM_NAME_LEN  16


//------------------------------
//-- Support functions
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_value2union(fmav_param_union_t* param_union, void* param_value, uint8_t type)
{
    param_union->type = type;

    param_union->p_uint32 = 0; // this fills them all with 0
    switch( type ){
        case MAV_PARAM_TYPE_UINT8:  param_union->p_uint8  = *((uint8_t*) param_value); return 1;
        case MAV_PARAM_TYPE_INT8:   param_union->p_int8   = *((int8_t*)  param_value); return 1;
        case MAV_PARAM_TYPE_UINT16: param_union->p_uint16 = *((uint16_t*)param_value); return 1;
        case MAV_PARAM_TYPE_INT16:  param_union->p_int16  = *((int16_t*) param_value); return 1;
        case MAV_PARAM_TYPE_UINT32: param_union->p_uint32 = *((uint32_t*)param_value); return 1;
        case MAV_PARAM_TYPE_INT32:  param_union->p_int32  = *((int32_t*) param_value); return 1;
        case MAV_PARAM_TYPE_REAL32: param_union->p_float  = *((float*)   param_value); return 1;
    }

    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_union2value(void* param_value, fmav_param_union_t* param_union)
{
    switch( param_union->type ){
        case MAV_PARAM_TYPE_UINT8:  *((uint8_t*) param_value) = param_union->p_uint8;  return 1;
        case MAV_PARAM_TYPE_INT8:   *((int8_t*)  param_value) = param_union->p_int8;   return 1;
        case MAV_PARAM_TYPE_UINT16: *((uint16_t*)param_value) = param_union->p_uint16; return 1;
        case MAV_PARAM_TYPE_INT16:  *((int16_t*) param_value) = param_union->p_int16;  return 1;
        case MAV_PARAM_TYPE_UINT32: *((uint32_t*)param_value) = param_union->p_uint32; return 1;
        case MAV_PARAM_TYPE_INT32:  *((int32_t*) param_value) = param_union->p_int32;  return 1;
        case MAV_PARAM_TYPE_REAL32: *((float*)   param_value) = param_union->p_float;  return 1;
    }

    return 0;
}


//------------------------------
//-- Functions with pointer to parameter list
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_param_find_index_list(char* name, const fmav_param_entry_t* param_list, uint16_t param_num)
{
    char s[FASTMAVLINK_PARAM_NAME_LEN+1]; // +1 to have room to convert to C string

    memcpy(s, name, FASTMAVLINK_PARAM_NAME_LEN);
    s[FASTMAVLINK_PARAM_NAME_LEN] = '\0'; // make it a terminated C string also in case of 16
    
    // it technically would be better to do this, but we do not as it never should matter
    // memset(s, '\0', FASTMAVLINK_PARAM_NAME_LEN+1);
    // strncpy(s,name, FASTMAVLINK_PARAM_NAME_LEN);

    for (uint16_t i = 0; i < param_num; i++) {
        if (!strcmp(s, param_list[i].name)) return i;
    }

    return -1;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_get_param_union_list(fmav_param_union_t* param_union, uint16_t index, const fmav_param_entry_t* param_list, uint16_t param_num)
{
    if (index >= param_num) return 0;

    return fmav_param_value2union(param_union, param_list[index].ptr, param_list[index].type);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_set_value_list(uint16_t index, float value, const fmav_param_entry_t* param_list, uint16_t param_num)
{
    if (index >= param_num) return 0;

    fmav_param_union_t param_union;
    param_union.p_float = value;

    param_union.type = param_list[index].type;
    return fmav_param_union2value(param_list[index].ptr, &param_union);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_get_param_value_list(fmav_param_value_t* payload, uint16_t index, const fmav_param_entry_t* param_list, uint16_t param_num)
{
    if (index >= param_num) return 0;

    fmav_param_union_t param_union;
    if (!fmav_param_get_param_union_list(&param_union, index, param_list, param_num)) return 0;

    payload->param_value = param_union.p_float;
    payload->param_count = param_num;
    payload->param_index = index;
    payload->param_type = param_list[index].type;

    memset(payload->param_id, '\0', FASTMAVLINK_PARAM_NAME_LEN);
    // according to spec, strncpy(param_id, name, FASTMAVLINK_PARAM_NAME_LEN) should do 
    // it correctly, but some compilers throw a warning, so play it safe
    uint8_t len = strlen(param_list[index].name);
    if (len > FASTMAVLINK_PARAM_NAME_LEN) len = FASTMAVLINK_PARAM_NAME_LEN;
    memcpy(payload->param_id, param_list[index].name, len);

    return 1;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_param_request_read_list(uint16_t* index, fmav_param_request_read_t* payload, const fmav_param_entry_t* param_list, uint16_t param_num)
{
    int16_t i = (payload->param_index < 0) ? fmav_param_find_index_list(payload->param_id, param_list, param_num) : payload->param_index;

    if (i < 0) return 0; // not found

    if (i >= param_num) return 0;

    *index = i;
    return 1;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_param_set_list(uint16_t* index, fmav_param_set_t* payload, const fmav_param_entry_t* param_list, uint16_t param_num)
{
    int16_t i = fmav_param_find_index_list(payload->param_id, param_list, param_num);

    if (i < 0) return 0; // not found

    if (payload->param_type != param_list[i].type) return 0; // ups ...

    *index = i;
    return 1;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_cmd_do_set_parameter_list(uint16_t* index, float param1, uint16_t param_num)
{
    *index = param1;

    if (*index >= param_num) return 0;

    return 1;
}


//------------------------------
//-- Handler functions
//------------------------------

// returns >= 0 if found, -1 else
FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_param_find_index(char* name)
{
    return fmav_param_find_index_list(name, fmav_param_list, FASTMAVLINK_PARAM_NUM);
}


// returns 0: failed 1: ok
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_get_param_union(fmav_param_union_t* param_union, uint16_t index)
{
    return fmav_param_get_param_union_list(param_union, index, fmav_param_list, FASTMAVLINK_PARAM_NUM);
}


// returns 0: failed 1: ok
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_set_value(uint16_t index, float value)
{
    return fmav_param_set_value_list(index, value, fmav_param_list, FASTMAVLINK_PARAM_NUM);
}


// returns 0: failed 1: ok
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_get_param_value(fmav_param_value_t* payload, uint16_t index)
{
    return fmav_param_get_param_value_list(payload, index, fmav_param_list, FASTMAVLINK_PARAM_NUM);
}


// returns 0: failed 1: ok
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_param_request_read(uint16_t* index, fmav_param_request_read_t* payload)
{
    return fmav_param_do_param_request_read_list(index, payload, fmav_param_list, FASTMAVLINK_PARAM_NUM);
}


// returns 0: failed 1: ok
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_param_set(uint16_t* index, fmav_param_set_t* payload)
{
    return fmav_param_do_param_set_list(index, payload, fmav_param_list, FASTMAVLINK_PARAM_NUM);
}


// returns 0: failed 1: ok
FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_cmd_do_set_parameter(uint16_t* index, float param1)
{
    return fmav_param_do_cmd_do_set_parameter_list(index, param1, FASTMAVLINK_PARAM_NUM);
}


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_PARAMETERS_H
