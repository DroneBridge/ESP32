/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2026 Wolfgang Christl
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

#ifndef DB_ESP32_DB_ESPNOW_BINDING_H
#define DB_ESP32_DB_ESPNOW_BINDING_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DB_ESPNOW_BIND_ROLE_NONE = 0,
    DB_ESPNOW_BIND_ROLE_GND = 1,
    DB_ESPNOW_BIND_ROLE_AIR = 2,
} db_espnow_bind_role_t;

typedef enum {
    DB_ESPNOW_BIND_REQUEST_NONE = 0,
    DB_ESPNOW_BIND_REQUEST_GND_ADD = 1,
    DB_ESPNOW_BIND_REQUEST_AIR = 2,
    DB_ESPNOW_BIND_REQUEST_GND_NEW_GROUP = 3,
} db_espnow_bind_request_t;

/**
 * Stores a one-boot ESP-NOW binding request for the selected role.
 *
 * @param role Role to assign after a successful binding session.
 * @return True when the request was persisted successfully.
 */
bool db_espnow_binding_request(db_espnow_bind_role_t role);

/**
 * Stores a one-boot GND binding request that replaces the current ESP-NOW group secret.
 *
 * @return True when the request was persisted successfully.
 */
bool db_espnow_binding_request_new_group(void);

/**
 * Generates and persists a new ESP-NOW group secret without starting binding.
 *
 * @return True when the new secret was stored successfully.
 */
bool db_espnow_binding_rotate_secret(void);

/**
 * Requests a normal exit from an active GND group-binding session.
 *
 * @return True when an active GND session accepted the stop request.
 */
bool db_espnow_binding_request_stop(void);

/**
 * Checks whether the dedicated binding runtime is currently active.
 *
 * @return True while binding owns application startup.
 */
bool db_espnow_binding_is_active(void);

/**
 * Clears a pending binding request and signals an active binding session to stop.
 */
void db_espnow_binding_cancel(void);

/**
 * Starts the dedicated ESP-NOW binding runtime when a persisted request exists.
 *
 * @return True when binding took ownership of application startup.
 */
bool db_espnow_binding_start_if_requested(void);

#endif // DB_ESP32_DB_ESPNOW_BINDING_H
