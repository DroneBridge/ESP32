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

#include <stddef.h>
#include <string.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <nvs.h>
#include <mbedtls/ecdh.h>
#include <mbedtls/gcm.h>
#include <mbedtls/sha256.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "db_espnow_binding.h"
#include "db_led_indicator.h"
#include "db_parameters.h"
#include "main.h"

#define TAG "DB_ESPNOW_BIND"
#define DB_BIND_NVS_NAMESPACE "settings"
#define DB_BIND_NVS_ROLE_KEY "bind_role"
#define DB_BIND_MAGIC_0 0x44U
#define DB_BIND_MAGIC_1 0x42U
#define DB_BIND_MAGIC_2 0x42U
#define DB_BIND_VERSION 2U
#define DB_BIND_TIMEOUT_MS 90000U
#define DB_BIND_CHANNEL_DWELL_MS 450U
#define DB_BIND_BEACON_INTERVAL_MS 300U
#define DB_BIND_EXCHANGE_TIMEOUT_MS 5000U
#define DB_BIND_SUCCESS_INDICATION_MS 700U
#define DB_BIND_ECDH_DATA_MAX 80U
#define DB_BIND_SESSION_LEN 8U
#define DB_BIND_EXCHANGE_LEN 8U
#define DB_BIND_SECRET_LEN 43U
#define DB_BIND_SECRET_STORAGE_LEN (DB_BIND_SECRET_LEN + 1U)
#define DB_BIND_SESSION_KEY_LEN 32U
#define DB_BIND_GCM_IV_LEN 12U
#define DB_BIND_GCM_TAG_LEN 16U
#define DB_BIND_QUEUE_SIZE 8U

static const uint8_t DB_BIND_BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

typedef enum { DB_BIND_FRAME_BEACON = 1, DB_BIND_FRAME_HELLO = 2, DB_BIND_FRAME_CONFIG = 3, DB_BIND_FRAME_ACK = 4 } db_bind_frame_type_t;

typedef struct {
    uint8_t magic[3];
    uint8_t version;
    uint8_t type;
    uint8_t session[DB_BIND_SESSION_LEN];
} __attribute__((packed)) db_bind_frame_header_t;

typedef struct {
    db_bind_frame_header_t header;
    uint8_t role;
    uint8_t channel;
    uint8_t exchange[DB_BIND_EXCHANGE_LEN];
    uint8_t ecdh_data_len;
    uint8_t ecdh_data[DB_BIND_ECDH_DATA_MAX];
} __attribute__((packed)) db_bind_discovery_frame_t;

typedef struct {
    uint8_t channel;
    uint8_t gnd_mode;
    uint8_t air_mode;
    uint8_t link_secret[DB_BIND_SECRET_STORAGE_LEN];
} __attribute__((packed)) db_bind_configuration_t;

typedef struct {
    db_bind_frame_header_t header;
    uint8_t target[ESP_NOW_ETH_ALEN];
    uint8_t exchange[DB_BIND_EXCHANGE_LEN];
    uint8_t iv[DB_BIND_GCM_IV_LEN];
    uint8_t tag[DB_BIND_GCM_TAG_LEN];
    uint8_t ciphertext[sizeof(db_bind_configuration_t)];
} __attribute__((packed)) db_bind_config_frame_t;

typedef struct {
    db_bind_frame_header_t header;
    uint8_t target[ESP_NOW_ETH_ALEN];
    uint8_t exchange[DB_BIND_EXCHANGE_LEN];
    uint8_t iv[DB_BIND_GCM_IV_LEN];
    uint8_t tag[DB_BIND_GCM_TAG_LEN];
    uint8_t ciphertext;
} __attribute__((packed)) db_bind_ack_frame_t;

typedef struct {
    uint8_t source[ESP_NOW_ETH_ALEN];
    uint8_t len;
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
} db_bind_receive_event_t;

typedef struct {
    db_espnow_bind_request_t request;
    db_espnow_bind_role_t role;
    QueueHandle_t receive_queue;
    uint8_t session[DB_BIND_SESSION_LEN];
    uint8_t exchange[DB_BIND_EXCHANGE_LEN];
    uint8_t local_mac[ESP_NOW_ETH_ALEN];
    uint8_t peer[ESP_NOW_ETH_ALEN];
    bool peer_known;
    bool exchange_active;
    volatile bool stop_requested;
    mbedtls_ecdh_context ecdh;
    uint8_t gnd_params[DB_BIND_ECDH_DATA_MAX];
    size_t gnd_params_len;
    uint8_t group_secret[DB_BIND_SECRET_STORAGE_LEN];
    uint8_t session_key[DB_BIND_SESSION_KEY_LEN];
    bool session_key_ready;
} db_bind_context_t;

static db_bind_context_t *db_bind_active_context = NULL;

/** Fills an mbedTLS random buffer from the ESP hardware random-number source. */
static int db_bind_rng(void *context, unsigned char *output, size_t output_len) {
    (void) context;
    esp_fill_random(output, output_len);
    return 0;
}

/** Checks whether a frame belongs to this binding protocol and group session. */
static bool db_bind_header_is_valid(const db_bind_frame_header_t *header, const uint8_t *session) {
    return header != NULL && session != NULL && header->magic[0] == DB_BIND_MAGIC_0 &&
           header->magic[1] == DB_BIND_MAGIC_1 && header->magic[2] == DB_BIND_MAGIC_2 &&
           header->version == DB_BIND_VERSION && memcmp(header->session, session, DB_BIND_SESSION_LEN) == 0;
}

/** Initializes a frame header for the supplied group session and frame type. */
static void db_bind_init_header(db_bind_frame_header_t *header, db_bind_frame_type_t type, const uint8_t *session) {
    header->magic[0] = DB_BIND_MAGIC_0;
    header->magic[1] = DB_BIND_MAGIC_1;
    header->magic[2] = DB_BIND_MAGIC_2;
    header->version = DB_BIND_VERSION;
    header->type = (uint8_t) type;
    memcpy(header->session, session, DB_BIND_SESSION_LEN);
}

/** Adds the ESP-NOW broadcast peer used by the binding runtime. */
static bool db_bind_add_broadcast_peer(void) {
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, DB_BIND_BROADCAST_MAC, ESP_NOW_ETH_ALEN);
    return esp_now_is_peer_exist(DB_BIND_BROADCAST_MAC) || esp_now_add_peer(&peer) == ESP_OK;
}

/** Reads the local station MAC address needed to target binding frames. */
static bool db_bind_read_local_mac(db_bind_context_t *context) {
    return esp_wifi_get_mac(WIFI_IF_STA, context->local_mac) == ESP_OK;
}

/** Queues a short binding frame received from the Wi-Fi task for task-context processing. */
static void db_bind_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (recv_info == NULL || recv_info->src_addr == NULL || recv_info->des_addr == NULL || data == NULL || len <= 0 ||
        len > ESP_NOW_MAX_DATA_LEN || memcmp(recv_info->des_addr, DB_BIND_BROADCAST_MAC, ESP_NOW_ETH_ALEN) != 0) {
        return;
    }
    db_bind_context_t *context = db_bind_active_context;
    if (context == NULL || context->receive_queue == NULL) {
        return;
    }
    db_bind_receive_event_t event = {0};
    memcpy(event.source, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    event.len = (uint8_t) len;
    memcpy(event.data, data, event.len);
    (void) xQueueSend(context->receive_queue, &event, 0);
}

/** Sends a binding frame via ESP-NOW broadcast. */
static bool db_bind_send(const void *frame, size_t frame_len) {
    return frame != NULL && frame_len > 0 && frame_len <= ESP_NOW_MAX_DATA_LEN &&
           esp_now_send(DB_BIND_BROADCAST_MAC, frame, frame_len) == ESP_OK;
}

/** Derives an AES-256 exchange key from ECDH output and both participant identities. */
static bool db_bind_derive_session_key(db_bind_context_t *context) {
    uint8_t shared_secret[DB_BIND_SESSION_KEY_LEN] = {0};
    size_t shared_secret_len = 0;
    const int result = mbedtls_ecdh_calc_secret(&context->ecdh, &shared_secret_len, shared_secret, sizeof(shared_secret), db_bind_rng, NULL);
    if (result != 0 || shared_secret_len == 0 || !context->peer_known) {
        ESP_LOGW(TAG, "ECDH shared-secret calculation failed: %d", result);
        return false;
    }
    uint8_t transcript[DB_BIND_SESSION_KEY_LEN + DB_BIND_SESSION_LEN + DB_BIND_EXCHANGE_LEN + 2 * ESP_NOW_ETH_ALEN] = {0};
    size_t offset = 0;
    memcpy(&transcript[offset], shared_secret, shared_secret_len);
    offset += shared_secret_len;
    memcpy(&transcript[offset], context->session, DB_BIND_SESSION_LEN);
    offset += DB_BIND_SESSION_LEN;
    memcpy(&transcript[offset], context->exchange, DB_BIND_EXCHANGE_LEN);
    offset += DB_BIND_EXCHANGE_LEN;
    const uint8_t *gnd_mac = context->role == DB_ESPNOW_BIND_ROLE_GND ? context->local_mac : context->peer;
    const uint8_t *air_mac = context->role == DB_ESPNOW_BIND_ROLE_GND ? context->peer : context->local_mac;
    memcpy(&transcript[offset], gnd_mac, ESP_NOW_ETH_ALEN);
    offset += ESP_NOW_ETH_ALEN;
    memcpy(&transcript[offset], air_mac, ESP_NOW_ETH_ALEN);
    offset += ESP_NOW_ETH_ALEN;
    const int hash_result = mbedtls_sha256(transcript, offset, context->session_key, 0);
    memset(shared_secret, 0, sizeof(shared_secret));
    memset(transcript, 0, sizeof(transcript));
    context->session_key_ready = hash_result == 0;
    return context->session_key_ready;
}

/** Encrypts and authenticates a configuration for its intended AIR recipient. */
static bool db_bind_encrypt_configuration(db_bind_context_t *context, db_bind_config_frame_t *frame,
                                          const db_bind_configuration_t *configuration) {
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    esp_fill_random(frame->iv, sizeof(frame->iv));
    const int set_key_result = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, context->session_key, 256);
    const int crypt_result = set_key_result == 0 ?
            mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, sizeof(*configuration), frame->iv, sizeof(frame->iv),
                                       (const uint8_t *) frame, offsetof(db_bind_config_frame_t, iv),
                                       (const uint8_t *) configuration, frame->ciphertext, sizeof(frame->tag), frame->tag) : -1;
    mbedtls_gcm_free(&gcm);
    return crypt_result == 0;
}

/** Decrypts and authenticates a received configuration for this AIR unit. */
static bool db_bind_decrypt_configuration(db_bind_context_t *context, const db_bind_config_frame_t *frame,
                                          db_bind_configuration_t *configuration) {
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    const int set_key_result = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, context->session_key, 256);
    const int decrypt_result = set_key_result == 0 ?
            mbedtls_gcm_auth_decrypt(&gcm, sizeof(*configuration), frame->iv, sizeof(frame->iv),
                                     (const uint8_t *) frame, offsetof(db_bind_config_frame_t, iv), frame->tag, sizeof(frame->tag),
                                     frame->ciphertext, (uint8_t *) configuration) : -1;
    mbedtls_gcm_free(&gcm);
    return decrypt_result == 0;
}

/** Sends an authenticated acknowledgement after the AIR unit persisted its configuration. */
static bool db_bind_send_ack(db_bind_context_t *context) {
    db_bind_ack_frame_t frame = {0};
    db_bind_init_header(&frame.header, DB_BIND_FRAME_ACK, context->session);
    memcpy(frame.target, context->peer, ESP_NOW_ETH_ALEN);
    memcpy(frame.exchange, context->exchange, DB_BIND_EXCHANGE_LEN);
    esp_fill_random(frame.iv, sizeof(frame.iv));
    const uint8_t acknowledgement = 0xa5U;
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    const int set_key_result = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, context->session_key, 256);
    const int crypt_result = set_key_result == 0 ?
            mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, sizeof(acknowledgement), frame.iv, sizeof(frame.iv),
                                       (const uint8_t *) &frame, offsetof(db_bind_ack_frame_t, iv), &acknowledgement,
                                       &frame.ciphertext, sizeof(frame.tag), frame.tag) : -1;
    mbedtls_gcm_free(&gcm);
    return crypt_result == 0 && db_bind_send(&frame, sizeof(frame));
}

/** Checks an acknowledgement received by the intended GND unit. */
static bool db_bind_ack_is_valid(db_bind_context_t *context, const db_bind_ack_frame_t *frame) {
    if (memcmp(frame->target, context->local_mac, ESP_NOW_ETH_ALEN) != 0 ||
        memcmp(frame->exchange, context->exchange, DB_BIND_EXCHANGE_LEN) != 0) {
        return false;
    }
    uint8_t acknowledgement = 0;
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    const int set_key_result = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, context->session_key, 256);
    const int decrypt_result = set_key_result == 0 ?
            mbedtls_gcm_auth_decrypt(&gcm, sizeof(acknowledgement), frame->iv, sizeof(frame->iv),
                                     (const uint8_t *) frame, offsetof(db_bind_ack_frame_t, iv), frame->tag, sizeof(frame->tag),
                                     &frame->ciphertext, &acknowledgement) : -1;
    mbedtls_gcm_free(&gcm);
    return decrypt_result == 0 && acknowledgement == 0xa5U;
}

/** Generates an ASCII Base64URL-compatible random link secret without padding. */
static void db_bind_generate_link_secret(char output[DB_BIND_SECRET_STORAGE_LEN]) {
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
    uint8_t random_bytes[DB_BIND_SECRET_LEN];
    esp_fill_random(random_bytes, sizeof(random_bytes));
    for (size_t index = 0; index < sizeof(random_bytes); index++) {
        output[index] = alphabet[random_bytes[index] & 0x3fU];
    }
    output[DB_BIND_SECRET_LEN] = '\0';
    memset(random_bytes, 0, sizeof(random_bytes));
}

/** Persists a group secret in the normal settings namespace. */
static bool db_bind_store_group_secret(const char secret[DB_BIND_SECRET_STORAGE_LEN]) {
    if (secret == NULL || strlen(secret) != DB_BIND_SECRET_LEN) {
        return false;
    }
    memcpy(DB_PARAM_ESPNOW_LINK_SECRET, secret, DB_BIND_SECRET_STORAGE_LEN);
    db_write_settings_to_nvs();
    return true;
}

/** Prepares the durable secret used for a GND group-binding session. */
static bool db_bind_prepare_gnd_group(db_bind_context_t *context) {
    const bool replace_group = context->request == DB_ESPNOW_BIND_REQUEST_GND_NEW_GROUP;
    if (replace_group || strlen(DB_PARAM_ESPNOW_LINK_SECRET) != DB_BIND_SECRET_LEN) {
        db_bind_generate_link_secret((char *) context->group_secret);
        return db_bind_store_group_secret((const char *) context->group_secret);
    }
    memcpy(context->group_secret, DB_PARAM_ESPNOW_LINK_SECRET, DB_BIND_SECRET_STORAGE_LEN);
    return true;
}

/** Applies an authenticated binding configuration and persists it for the next boot. */
static bool db_bind_apply_configuration(const db_bind_configuration_t *configuration, db_espnow_bind_role_t local_role) {
    if (configuration->channel < 1 || configuration->channel > 13 ||
        configuration->gnd_mode != DB_WIFI_MODE_ESPNOW_GND || configuration->air_mode != DB_WIFI_MODE_ESPNOW_AIR ||
        local_role == DB_ESPNOW_BIND_ROLE_NONE || configuration->link_secret[DB_BIND_SECRET_LEN] != '\0') {
        return false;
    }
    memcpy(DB_PARAM_ESPNOW_LINK_SECRET, configuration->link_secret, DB_BIND_SECRET_STORAGE_LEN);
    DB_PARAM_CHANNEL = configuration->channel;
    DB_RADIO_MODE_DESIGNATED = local_role == DB_ESPNOW_BIND_ROLE_GND ? DB_WIFI_MODE_ESPNOW_GND : DB_WIFI_MODE_ESPNOW_AIR;
    db_write_settings_to_nvs();
    return true;
}

/** Clears the transient bind request from NVS so normal startup resumes. */
static void db_bind_clear_request(void) {
    nvs_handle_t handle;
    if (nvs_open(DB_BIND_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        (void) nvs_erase_key(handle, DB_BIND_NVS_ROLE_KEY);
        (void) nvs_commit(handle);
        nvs_close(handle);
    }
}

/** Displays a binding LED state for the requested time while keeping the task watchdog serviced. */
static void db_bind_show_led_state(db_status_led_binding_state_t state, uint32_t duration_ms) {
    db_status_led_set_binding_state(state);
    const TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
    while ((int32_t) (xTaskGetTickCount() - end) < 0) {
        db_status_led_process();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/** Discards queued receive events after an exchange to avoid replaying stale hellos. */
static void db_bind_drain_receive_queue(db_bind_context_t *context) {
    db_bind_receive_event_t discarded;
    while (xQueueReceive(context->receive_queue, &discarded, 0) == pdTRUE) {
    }
}

/** Ends the active exchange and erases its sensitive transient key material. */
static void db_bind_reset_exchange(db_bind_context_t *context) {
    memset(context->exchange, 0, sizeof(context->exchange));
    memset(context->peer, 0, sizeof(context->peer));
    memset(context->session_key, 0, sizeof(context->session_key));
    context->peer_known = false;
    context->session_key_ready = false;
    context->exchange_active = false;
}

/** Sends the periodic GND discovery beacon for the active group session. */
static void db_bind_send_beacon(const db_bind_context_t *context) {
    db_bind_discovery_frame_t beacon = {0};
    db_bind_init_header(&beacon.header, DB_BIND_FRAME_BEACON, context->session);
    beacon.role = DB_ESPNOW_BIND_ROLE_GND;
    beacon.channel = DB_PARAM_CHANNEL;
    beacon.ecdh_data_len = (uint8_t) context->gnd_params_len;
    memcpy(beacon.ecdh_data, context->gnd_params, context->gnd_params_len);
    (void) db_bind_send(&beacon, sizeof(beacon));
}

/** Starts a GND-side exchange with one AIR hello frame. */
static bool db_bind_start_gnd_exchange(db_bind_context_t *context, const db_bind_receive_event_t *event,
                                       db_bind_configuration_t *configuration, db_bind_config_frame_t *config_frame) {
    const db_bind_discovery_frame_t *hello = (const db_bind_discovery_frame_t *) event->data;
    if (hello->role != DB_ESPNOW_BIND_ROLE_AIR || hello->channel != DB_PARAM_CHANNEL || hello->ecdh_data_len == 0 ||
        hello->ecdh_data_len > DB_BIND_ECDH_DATA_MAX ||
        mbedtls_ecdh_read_public(&context->ecdh, hello->ecdh_data, hello->ecdh_data_len) != 0) {
        return false;
    }
    memcpy(context->peer, event->source, ESP_NOW_ETH_ALEN);
    memcpy(context->exchange, hello->exchange, DB_BIND_EXCHANGE_LEN);
    context->peer_known = true;
    if (!db_bind_derive_session_key(context)) {
        db_bind_reset_exchange(context);
        return false;
    }
    configuration->channel = DB_PARAM_CHANNEL;
    configuration->gnd_mode = DB_WIFI_MODE_ESPNOW_GND;
    configuration->air_mode = DB_WIFI_MODE_ESPNOW_AIR;
    memcpy(configuration->link_secret, context->group_secret, DB_BIND_SECRET_STORAGE_LEN);
    db_bind_init_header(&config_frame->header, DB_BIND_FRAME_CONFIG, context->session);
    memcpy(config_frame->target, context->peer, ESP_NOW_ETH_ALEN);
    memcpy(config_frame->exchange, context->exchange, DB_BIND_EXCHANGE_LEN);
    if (!db_bind_encrypt_configuration(context, config_frame, configuration) || !db_bind_send(config_frame, sizeof(*config_frame))) {
        db_bind_reset_exchange(context);
        return false;
    }
    context->exchange_active = true;
    return true;
}

/** Runs the GND side of a user-ended multi-AIR binding session. */
static bool db_bind_run_gnd(db_bind_context_t *context) {
    if (!db_bind_prepare_gnd_group(context) || mbedtls_ecdh_setup(&context->ecdh, MBEDTLS_ECP_DP_CURVE25519) != 0 ||
        mbedtls_ecdh_make_params(&context->ecdh, &context->gnd_params_len, context->gnd_params,
                                 sizeof(context->gnd_params), db_bind_rng, NULL) != 0) {
        ESP_LOGE(TAG, "Failed to prepare GND group binding");
        return false;
    }
    TickType_t last_beacon = 0;
    TickType_t last_configuration = 0;
    TickType_t exchange_started = 0;
    bool bound_any = false;
    db_bind_configuration_t configuration = {0};
    db_bind_config_frame_t config_frame = {0};
    while (!context->stop_requested) {
        const TickType_t now = xTaskGetTickCount();
        db_status_led_process();
        if (last_beacon == 0 || now - last_beacon >= pdMS_TO_TICKS(DB_BIND_BEACON_INTERVAL_MS)) {
            db_bind_send_beacon(context);
            last_beacon = now;
        }
        if (context->exchange_active && now - last_configuration >= pdMS_TO_TICKS(DB_BIND_BEACON_INTERVAL_MS)) {
            (void) db_bind_send(&config_frame, sizeof(config_frame));
            last_configuration = now;
        }
        if (context->exchange_active && now - exchange_started >= pdMS_TO_TICKS(DB_BIND_EXCHANGE_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "AIR binding exchange timed out");
            db_bind_reset_exchange(context);
            memset(&configuration, 0, sizeof(configuration));
            memset(&config_frame, 0, sizeof(config_frame));
            db_bind_drain_receive_queue(context);
            db_status_led_set_binding_state(DB_STATUS_LED_BINDING_SEARCHING);
            continue;
        }
        db_bind_receive_event_t event;
        if (xQueueReceive(context->receive_queue, &event, pdMS_TO_TICKS(50)) != pdTRUE || event.len < sizeof(db_bind_frame_header_t)) {
            continue;
        }
        const db_bind_frame_header_t *header = (const db_bind_frame_header_t *) event.data;
        if (!db_bind_header_is_valid(header, context->session)) {
            continue;
        }
        if (!context->exchange_active && header->type == DB_BIND_FRAME_HELLO && event.len == sizeof(db_bind_discovery_frame_t)) {
            if (db_bind_start_gnd_exchange(context, &event, &configuration, &config_frame)) {
                exchange_started = xTaskGetTickCount();
                last_configuration = exchange_started;
                db_status_led_set_binding_state(DB_STATUS_LED_BINDING_NEGOTIATING);
            }
        } else if (context->exchange_active && header->type == DB_BIND_FRAME_ACK && event.len == sizeof(db_bind_ack_frame_t) &&
                   memcmp(context->peer, event.source, ESP_NOW_ETH_ALEN) == 0 &&
                   db_bind_ack_is_valid(context, (const db_bind_ack_frame_t *) event.data)) {
            const bool saved = db_bind_apply_configuration(&configuration, DB_ESPNOW_BIND_ROLE_GND);
            if (saved) {
                bound_any = true;
                db_bind_show_led_state(DB_STATUS_LED_BINDING_SUCCESS, DB_BIND_SUCCESS_INDICATION_MS);
            }
            db_bind_reset_exchange(context);
            memset(&configuration, 0, sizeof(configuration));
            memset(&config_frame, 0, sizeof(config_frame));
            db_bind_drain_receive_queue(context);
            db_status_led_set_binding_state(DB_STATUS_LED_BINDING_SEARCHING);
        }
    }
    memset(&configuration, 0, sizeof(configuration));
    memset(&config_frame, 0, sizeof(config_frame));
    return bound_any;
}

/** Runs the AIR side of the physical ESP-NOW binding protocol. */
static bool db_bind_run_air(db_bind_context_t *context) {
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(DB_BIND_TIMEOUT_MS);
    uint8_t channel = 1;
    bool channel_locked = false;
    TickType_t last_hello = 0;
    bool hello_ready = false;
    db_bind_discovery_frame_t hello = {0};
    while ((int32_t) (xTaskGetTickCount() - deadline) < 0) {
        if (!channel_locked) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
        }
        const TickType_t dwell_end = xTaskGetTickCount() + pdMS_TO_TICKS(channel_locked ? 100U : DB_BIND_CHANNEL_DWELL_MS);
        while ((int32_t) (xTaskGetTickCount() - dwell_end) < 0) {
            db_status_led_process();
            db_bind_receive_event_t event;
            if (xQueueReceive(context->receive_queue, &event, pdMS_TO_TICKS(50)) != pdTRUE || event.len < sizeof(db_bind_frame_header_t)) {
                continue;
            }
            const db_bind_frame_header_t *header = (const db_bind_frame_header_t *) event.data;
            if (header->magic[0] != DB_BIND_MAGIC_0 || header->magic[1] != DB_BIND_MAGIC_1 ||
                header->magic[2] != DB_BIND_MAGIC_2 || header->version != DB_BIND_VERSION) {
                continue;
            }
            if (!channel_locked && header->type == DB_BIND_FRAME_BEACON && event.len == sizeof(db_bind_discovery_frame_t)) {
                const db_bind_discovery_frame_t *beacon = (const db_bind_discovery_frame_t *) event.data;
                const unsigned char *ecdh_parameters = beacon->ecdh_data;
                const unsigned char *ecdh_parameters_end = beacon->ecdh_data + beacon->ecdh_data_len;
                if (beacon->role != DB_ESPNOW_BIND_ROLE_GND || beacon->channel != channel || beacon->ecdh_data_len == 0 ||
                    beacon->ecdh_data_len > DB_BIND_ECDH_DATA_MAX ||
                    mbedtls_ecdh_read_params(&context->ecdh, &ecdh_parameters, ecdh_parameters_end) != 0 ||
                    ecdh_parameters != ecdh_parameters_end) {
                    continue;
                }
                memcpy(context->session, header->session, DB_BIND_SESSION_LEN);
                memcpy(context->peer, event.source, ESP_NOW_ETH_ALEN);
                context->peer_known = true;
                esp_fill_random(context->exchange, sizeof(context->exchange));
                uint8_t public_data[DB_BIND_ECDH_DATA_MAX] = {0};
                size_t public_data_len = 0;
                if (mbedtls_ecdh_make_public(&context->ecdh, &public_data_len, public_data, sizeof(public_data), db_bind_rng, NULL) != 0 ||
                    !db_bind_derive_session_key(context)) {
                    return false;
                }
                db_bind_init_header(&hello.header, DB_BIND_FRAME_HELLO, context->session);
                hello.role = DB_ESPNOW_BIND_ROLE_AIR;
                hello.channel = channel;
                memcpy(hello.exchange, context->exchange, DB_BIND_EXCHANGE_LEN);
                hello.ecdh_data_len = (uint8_t) public_data_len;
                memcpy(hello.ecdh_data, public_data, public_data_len);
                (void) db_bind_send(&hello, sizeof(hello));
                last_hello = xTaskGetTickCount();
                hello_ready = true;
                channel_locked = true;
                db_status_led_set_binding_state(DB_STATUS_LED_BINDING_NEGOTIATING);
            } else if (channel_locked && db_bind_header_is_valid(header, context->session) &&
                       header->type == DB_BIND_FRAME_CONFIG && event.len == sizeof(db_bind_config_frame_t) &&
                       memcmp(context->peer, event.source, ESP_NOW_ETH_ALEN) == 0) {
                const db_bind_config_frame_t *config_frame = (const db_bind_config_frame_t *) event.data;
                if (memcmp(config_frame->target, context->local_mac, ESP_NOW_ETH_ALEN) != 0 ||
                    memcmp(config_frame->exchange, context->exchange, DB_BIND_EXCHANGE_LEN) != 0) {
                    continue;
                }
                db_bind_configuration_t configuration = {0};
                const bool valid = db_bind_decrypt_configuration(context, config_frame, &configuration);
                if (!valid) {
                    continue;
                }
                const bool saved = db_bind_apply_configuration(&configuration, DB_ESPNOW_BIND_ROLE_AIR);
                memset(&configuration, 0, sizeof(configuration));
                if (saved) {
                    (void) db_bind_send_ack(context);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                return saved;
            }
        }
        if (channel_locked && hello_ready && xTaskGetTickCount() - last_hello >= pdMS_TO_TICKS(DB_BIND_BEACON_INTERVAL_MS)) {
            (void) db_bind_send(&hello, sizeof(hello));
            last_hello = xTaskGetTickCount();
        }
        if (!channel_locked) {
            channel = channel == 13 ? 1 : channel + 1;
        }
    }
    return false;
}

/** Deinitializes binding-only ESP-NOW resources and erases sensitive session material. */
static void db_bind_cleanup(db_bind_context_t *context) {
    if (context->receive_queue != NULL) {
        vQueueDelete(context->receive_queue);
        context->receive_queue = NULL;
    }
    if (db_bind_active_context == context) {
        db_bind_active_context = NULL;
    }
    mbedtls_ecdh_free(&context->ecdh);
    memset(context->session_key, 0, sizeof(context->session_key));
    memset(context->group_secret, 0, sizeof(context->group_secret));
    (void) esp_now_unregister_recv_cb();
    (void) esp_now_deinit();
}

/** Executes the binding runtime and restarts into either the newly bound or prior configuration. */
static void db_bind_task(void *argument) {
    db_bind_context_t context = {.request = (db_espnow_bind_request_t) (uintptr_t) argument};
    context.role = context.request == DB_ESPNOW_BIND_REQUEST_AIR ? DB_ESPNOW_BIND_ROLE_AIR : DB_ESPNOW_BIND_ROLE_GND;
    mbedtls_ecdh_init(&context.ecdh);
    context.receive_queue = xQueueCreate(DB_BIND_QUEUE_SIZE, sizeof(db_bind_receive_event_t));
    if (context.receive_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create receive queue");
        db_bind_clear_request();
        esp_restart();
        vTaskDelete(NULL);
        return;
    }
    db_bind_active_context = &context;
    esp_fill_random(context.session, sizeof(context.session));
    db_status_led_set_binding_state(DB_STATUS_LED_BINDING_SEARCHING);
    const uint8_t initial_channel = context.role == DB_ESPNOW_BIND_ROLE_GND ? DB_PARAM_CHANNEL : 1;
    db_init_wifi_espnow_channel(initial_channel);
    const bool initialized = esp_now_init() == ESP_OK && db_bind_add_broadcast_peer() &&
                             esp_now_register_recv_cb(db_bind_receive_callback) == ESP_OK && db_bind_read_local_mac(&context);
    bool bound = false;
    if (initialized) {
        bound = context.role == DB_ESPNOW_BIND_ROLE_GND ? db_bind_run_gnd(&context) : db_bind_run_air(&context);
    } else {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW binding runtime");
    }
    db_bind_cleanup(&context);
    db_bind_clear_request();
    db_bind_show_led_state(bound ? DB_STATUS_LED_BINDING_SUCCESS : DB_STATUS_LED_BINDING_FAILURE, bound ? 1000U : 1500U);
    esp_restart();
}

/** Stores a one-boot ESP-NOW binding request for the selected role. */
bool db_espnow_binding_request(db_espnow_bind_role_t role) {
    if (role != DB_ESPNOW_BIND_ROLE_GND && role != DB_ESPNOW_BIND_ROLE_AIR) {
        return false;
    }
    const db_espnow_bind_request_t request = role == DB_ESPNOW_BIND_ROLE_GND ?
            DB_ESPNOW_BIND_REQUEST_GND_ADD : DB_ESPNOW_BIND_REQUEST_AIR;
    nvs_handle_t handle;
    if (nvs_open(DB_BIND_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    const esp_err_t result = nvs_set_u8(handle, DB_BIND_NVS_ROLE_KEY, (uint8_t) request);
    const esp_err_t commit_result = result == ESP_OK ? nvs_commit(handle) : result;
    nvs_close(handle);
    return commit_result == ESP_OK;
}

/** Stores a one-boot GND request that deliberately replaces the current group secret. */
bool db_espnow_binding_request_new_group(void) {
    nvs_handle_t handle;
    if (nvs_open(DB_BIND_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    const esp_err_t result = nvs_set_u8(handle, DB_BIND_NVS_ROLE_KEY, DB_ESPNOW_BIND_REQUEST_GND_NEW_GROUP);
    const esp_err_t commit_result = result == ESP_OK ? nvs_commit(handle) : result;
    nvs_close(handle);
    return commit_result == ESP_OK;
}

/** Generates and persists a new group secret without starting a binding session. */
bool db_espnow_binding_rotate_secret(void) {
    if (db_bind_active_context != NULL) {
        return false;
    }
    char secret[DB_BIND_SECRET_STORAGE_LEN] = {0};
    db_bind_generate_link_secret(secret);
    const bool saved = db_bind_store_group_secret(secret);
    memset(secret, 0, sizeof(secret));
    return saved;
}

/** Requests that an active GND group-binding session finish normally. */
bool db_espnow_binding_request_stop(void) {
    if (db_bind_active_context == NULL || db_bind_active_context->role != DB_ESPNOW_BIND_ROLE_GND) {
        return false;
    }
    db_bind_active_context->stop_requested = true;
    return true;
}

/** Reports whether the dedicated binding runtime currently owns application startup. */
bool db_espnow_binding_is_active(void) {
    return db_bind_active_context != NULL;
}

/** Cancels a binding request so a reset cannot re-enter the binding runtime. */
void db_espnow_binding_cancel(void) {
    if (db_bind_active_context != NULL) {
        db_bind_active_context->stop_requested = true;
    }
    db_bind_clear_request();
}

/** Starts the dedicated ESP-NOW binding runtime when a persisted request exists. */
bool db_espnow_binding_start_if_requested(void) {
    nvs_handle_t handle;
    uint8_t stored_request = DB_ESPNOW_BIND_REQUEST_NONE;
    if (nvs_open(DB_BIND_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    const esp_err_t result = nvs_get_u8(handle, DB_BIND_NVS_ROLE_KEY, &stored_request);
    nvs_close(handle);
    if (result != ESP_OK || (stored_request != DB_ESPNOW_BIND_REQUEST_GND_ADD &&
                             stored_request != DB_ESPNOW_BIND_REQUEST_AIR &&
                             stored_request != DB_ESPNOW_BIND_REQUEST_GND_NEW_GROUP)) {
        return false;
    }
    const BaseType_t task_created = xTaskCreate(db_bind_task, "espnow_bind", 12288, (void *) (uintptr_t) stored_request, 5, NULL);
    return task_created == pdPASS;
}
