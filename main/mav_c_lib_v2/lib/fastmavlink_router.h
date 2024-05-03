//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// API:
//
// requires that these tokens are defined in the outside:
//   FASTMAVLINK_ROUTER_LINKS_MAX
//   FASTMAVLINK_ROUTER_COMPONENTS_MAX
//
// this token can be defined in the outside to modify behavior:
//   FASTMAVLINK_ROUTER_LINK_PROPERTY_DEFAULT
//
// timeout mechanism to clear out dead components:
//   FASTMAVLINK_ROUTER_USE_TIMEOUT: define this token to enable the mechanism
//   FASTMAVLINK_ROUTER_TIMEOUT_100MS: defines the timeout in 100ms, default is 100 = 10 seconds
//   uint8_t fmav_router_time_100ms(void): must be provided externally
//
// void fmav_router_reset(void)
// void fmav_router_handle_message_by_id(
//     uint8_t link_of_msg,
//     uint8_t msgid,
//     uint8_t sysid, uint8_t compid,
//     uint8_t target_sysid, uint8_t target_compid
//     )
// void fmav_router_handle_message(uint8_t link_of_msg, fmav_result_t* result)
// void fmav_router_handle_message_by_msg(uint8_t link_of_msg, fmav_message_t* msg)
// uint8_t fmav_router_send_to_link(uint8_t link)
// void fmav_router_add_ourself(uint8_t sysid, uint8_t compid)
// void fmav_router_reset_link(uint8_t link)
// void fmav_router_set_link_properties(uint8_t link, uint8_t properties)
// void fmav_router_set_link_properties_all(uint8_t properties)
// void fmav_router_init(void)
//------------------------------

#pragma once
#ifndef FASTMAVLINK_ROUTER_H
#define FASTMAVLINK_ROUTER_H

#ifndef FASTMAVLINK_ROUTER_COMPONENTS_MAX
#error For fastmavlink_router.h, FASTMAVLINK_ROUTER_COMPONENTS_MAX needs to be defined
#endif
#ifndef FASTMAVLINK_ROUTER_LINKS_MAX
#error For fastmavlink_router.h, FASTMAVLINK_ROUTER_LINKS_MAX needs to be defined
#endif

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include "../fastmavlink_config.h"
#include "fastmavlink_types.h"


#ifndef FASTMAVLINK_ROUTER_LINK_PROPERTY_DEFAULT
#define FASTMAVLINK_ROUTER_LINK_PROPERTY_DEFAULT \
    (FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_ALWAYS_SEND_HEARTBEAT | \
     FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_DISCOVER_BY_HEARTBEAT | \
     FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_TIMEOUT)
#endif


#ifdef FASTMAVLINK_ROUTER_USE_TIMEOUT
    uint8_t fmav_router_time_100ms(void); // forward declaration
#ifndef FASTMAVLINK_ROUTER_TIMEOUT_100MS
    #define FASTMAVLINK_ROUTER_TIMEOUT_100MS  100 // 10 seconds
#endif
#endif


//------------------------------
//-- Structures & fields
//------------------------------

typedef struct _fmav_router_component_item {
    uint8_t valid;
    uint8_t sysid;
    uint8_t compid;
    uint8_t link; // 0 is ourself, 1 = COMM0, 2 = COMM1, ...
#ifdef FASTMAVLINK_ROUTER_USE_TIMEOUT
    uint8_t last_activity_100ms;
#endif
} fmav_router_component_item;


typedef enum {
    FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_DISCOVER_BY_HEARTBEAT = 0x01,
    FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_ALWAYS_SEND_HEARTBEAT = 0x02,
    FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_TIMEOUT = 0x04,
} fmav_router_link_property_flags_e;


FASTMAVLINK_RAM_SECTION fmav_router_component_item _fmav_router_component_list[FASTMAVLINK_ROUTER_COMPONENTS_MAX];

FASTMAVLINK_RAM_SECTION uint8_t _fmav_router_link_properties[FASTMAVLINK_ROUTER_LINKS_MAX];

FASTMAVLINK_RAM_SECTION uint8_t _fmav_router_send_to_link[FASTMAVLINK_ROUTER_LINKS_MAX];


//------------------------------
//-- Helpers
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_accept(uint8_t target_link, uint8_t target_sysid, uint8_t target_compid)
{
    // go through all components on the link and see if the one we are targeting at is there
    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (!_fmav_router_component_list[i].valid) continue; // empty entry

        if (_fmav_router_component_list[i].link != target_link) continue; // not our link

        if (target_sysid == 0) { // target link has seen at least one component, and target_sysid is broadcast, so ok
            return 1;
        }

        if (_fmav_router_component_list[i].sysid != target_sysid) continue; // not our system

        if (target_compid == 0) { // target_sysid is on the link, and target_compid is broadcast
            return 1;
        }

        if (_fmav_router_component_list[i].compid == target_compid) { // target_sysid and target_compid is on the link
            return 1;
        }
    }

    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_find_component(uint8_t sysid, uint8_t compid)
{
    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (!_fmav_router_component_list[i].valid) continue; // empty entry
        if (_fmav_router_component_list[i].sysid == sysid &&
            _fmav_router_component_list[i].compid == compid) {
            return 1;
        }
    }
    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_add_component(uint8_t link, uint8_t sysid, uint8_t compid)
{
    if (link >= FASTMAVLINK_ROUTER_LINKS_MAX) return 0;

    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (_fmav_router_component_list[i].valid) continue; // already occupied
        _fmav_router_component_list[i].valid = 1;
        _fmav_router_component_list[i].link = link;
        _fmav_router_component_list[i].sysid = sysid;
        _fmav_router_component_list[i].compid = compid;
#ifdef FASTMAVLINK_ROUTER_USE_TIMEOUT
        _fmav_router_component_list[i].last_activity_100ms = fmav_router_time_100ms();
#endif
        return 1;
    }
    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_find_or_add_component(uint8_t link, uint8_t sysid, uint8_t compid)
{
    if (fmav_router_find_component(sysid, compid)) return 1;
    // not found, so try to add
    if (fmav_router_add_component(link, sysid, compid)) return 1;
    // could not be added
    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_reset_component(uint8_t i)
{
    _fmav_router_component_list[i].valid = 0;
    _fmav_router_component_list[i].link = 0;
    _fmav_router_component_list[i].sysid = 0;
    _fmav_router_component_list[i].compid = 0;
#ifdef FASTMAVLINK_ROUTER_USE_TIMEOUT
    _fmav_router_component_list[i].last_activity_100ms = 0;
#endif
}


//------------------------------
//-- API
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_reset(void)
{
    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        fmav_router_reset_component(i);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_handle_message_by_id(
    uint8_t link_of_msg,
    uint8_t msgid,
    uint8_t sysid, uint8_t compid,
    uint8_t target_sysid, uint8_t target_compid)
{
    // should not occur, but play it safe
    if (link_of_msg >= FASTMAVLINK_ROUTER_LINKS_MAX) {
        for (uint8_t link = 0; link < FASTMAVLINK_ROUTER_LINKS_MAX; link++) {
            _fmav_router_send_to_link[link] = 0;
        }
        return;
    }

    // keep list of available components
    if (_fmav_router_link_properties[link_of_msg] & FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_DISCOVER_BY_HEARTBEAT) {
        // spy heartbeats as evidence of presence of component
        if (msgid == FASTMAVLINK_MSG_ID_HEARTBEAT) {
            fmav_router_find_or_add_component(link_of_msg, sysid, compid);
        }
    } else {
        // accept any message as evidence of presence of component
        fmav_router_find_or_add_component(link_of_msg, sysid, compid);
    }

    // update timeout for seen component, and clear out dead components
#ifdef FASTMAVLINK_ROUTER_USE_TIMEOUT
    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (!_fmav_router_component_list[i].valid) continue; // empty entry
        // update timeout for component
        if (_fmav_router_component_list[i].sysid == sysid && _fmav_router_component_list[i].compid == compid) {
            _fmav_router_component_list[i].last_activity_100ms = fmav_router_time_100ms();
            continue; // is alive, so no need to check if dead
        }
        // clear out if dead
        const uint8_t link = _fmav_router_component_list[i].link;
        if (_fmav_router_link_properties[link] & FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_TIMEOUT) {
            uint8_t dt = fmav_router_time_100ms() - _fmav_router_component_list[i].last_activity_100ms;
            if (dt > FASTMAVLINK_ROUTER_TIMEOUT_100MS) fmav_router_reset_component(i);
        }
    }
#endif

    // determine the links it has to be send to
    for (uint8_t link = 0; link < FASTMAVLINK_ROUTER_LINKS_MAX; link++) {
        _fmav_router_send_to_link[link] = 0;

        if (link == link_of_msg) continue; // origin of msg, don't reflect it back

        // send heartbeats to all links, which want it to be always send
        // otherwise messages are send out on a link only if at least one component was seen
        if ((msgid == FASTMAVLINK_MSG_ID_HEARTBEAT) &&
            (_fmav_router_link_properties[link] & FASTMAVLINK_ROUTER_LINK_PROPERTY_FLAG_ALWAYS_SEND_HEARTBEAT)) {
            _fmav_router_send_to_link[link] = 1;
            continue;
        }

        if (fmav_router_accept(link, target_sysid, target_compid)) {
          _fmav_router_send_to_link[link] = 1;
          continue;
        }
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_handle_message(uint8_t link_of_msg, fmav_result_t* result)
{
    fmav_router_handle_message_by_id(
        link_of_msg,
        result->msgid,
        result->sysid,
        result->compid,
        result->target_sysid,
        result->target_compid);
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_handle_message_by_msg(uint8_t link_of_msg, fmav_message_t* msg)
{
    fmav_router_handle_message_by_id(
        link_of_msg,
        msg->msgid,
        msg->sysid,
        msg->compid,
        msg->target_sysid,
        msg->target_compid);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_send_to_link(uint8_t link)
{
    if (link >= FASTMAVLINK_ROUTER_LINKS_MAX) return 0;
    return _fmav_router_send_to_link[link];
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_add_ourself(uint8_t sysid, uint8_t compid)
{
    fmav_router_add_component(0, sysid, compid); // we always add ourself as link 0
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_reset_link(uint8_t link)
{
    if (link >= FASTMAVLINK_ROUTER_LINKS_MAX) return;

    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (!_fmav_router_component_list[i].valid) continue; // empty entry
        if (_fmav_router_component_list[i].link == link) { // clear out
            fmav_router_reset_component(i);
        }
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_set_link_properties(uint8_t link, uint8_t properties)
{
    if (link >= FASTMAVLINK_ROUTER_LINKS_MAX) return;
    _fmav_router_link_properties[link] = properties;
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_set_link_properties_all(uint8_t properties)
{
    for (uint8_t link = 0; link < FASTMAVLINK_ROUTER_LINKS_MAX; link++) {
        _fmav_router_link_properties[link] = properties;
    }
}


// call it once before using the library
FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_init(void)
{
    for (uint8_t link = 0; link < FASTMAVLINK_ROUTER_LINKS_MAX; link++) {
        _fmav_router_link_properties[link] = FASTMAVLINK_ROUTER_LINK_PROPERTY_DEFAULT;
        _fmav_router_send_to_link[link] = 0;
    }

    fmav_router_reset();
}


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_ROUTER_H
