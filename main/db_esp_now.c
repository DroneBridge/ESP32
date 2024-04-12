#include <sys/cdefs.h>
/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2024 Wolfgang Christl
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

#include <esp_err.h>
#include <esp_now.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <string.h>
#include <esp_crc.h>
#include <mbedtls/gcm.h>
#include <mbedtls/md.h>
#include <mbedtls/pkcs5.h>
#include "db_esp_now.h"
#include "globals.h"
#include "main.h"

#define TAG "DB_ESPNOW"

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static QueueHandle_t db_espnow_send_recv_callback_queue;    // Queue that contains ESP-NOW callback results

mbedtls_gcm_context aes;
uint8_t const size_packet_header = sizeof(db_esp_now_packet_header_t);
uint8_t const size_protected_data = sizeof(db_esp_now_packet_protected_data_t);

/**
 * Steps to process received ESPNOW data. ESP firmware ensures that only correct packets are forwared to us
 * 1. Check sequence number
 * 2. Decrypt payload
 * ToDo: Do some FEC
 * 3. Write payload to uart-queue so it can be processed by the control_espnow task
 *
 * @param data
 * @param data_len
 * @return
 */
int db_espnow_process_rcv_data(uint8_t *data, uint16_t data_len) {
    db_esp_now_packet_t *db_esp_now_packet = (db_esp_now_packet_t *) data;
    if (data_len < sizeof(db_esp_now_packet_t)) {
        ESP_LOGE(TAG, "Receive ESP-NOW data too short, len:%d", data_len);
        return -1;
    }
    return -1;
}

/**
 * Genrates a AES key from a password using pkcs5 - pbkdf2 and mbedTLS
 *
 * @param password
 * @param key Output buffer for the key
 * @param keylen Length of the aes key to be generated
 */
void generate_pkcs5_key(const char* password, unsigned char* key, size_t keylen) {
    mbedtls_md_context_t mdctx;
    const mbedtls_md_info_t* md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256); // Use SHA-256 for key derivation

    // Initialize the context
    mbedtls_md_init(&mdctx);
    int ret = mbedtls_md_setup(&mdctx, md_info, 0);
    switch (ret) {
        case 0:
            // success
            break;
        case MBEDTLS_ERR_MD_BAD_INPUT_DATA:
            ESP_LOGW(TAG, "mbedtls_md_setup returned MBEDTLS_ERR_MD_BAD_INPUT_DATA");
            break;
        case MBEDTLS_ERR_MD_ALLOC_FAILED:
            ESP_LOGW(TAG, "mbedtls_md_setup returned MBEDTLS_ERR_MD_ALLOC_FAILED");
            break;
        default:
            ESP_LOGW(TAG, "mbedtls_md_setup returned unknown error: %i", ret);
            break;
    }
    static unsigned char generic_salt[] = "GenSalt147894562";    // Generic on purpose. No need for salt in this application
    ret = mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256,
                                        (const unsigned char *)password, strlen(password),
                                        generic_salt, strlen((char*) generic_salt), 10000, keylen, key);
    switch (ret) {
        case 0:
            // success
            break;
        case MBEDTLS_ERR_PKCS5_BAD_INPUT_DATA:
            ESP_LOGW(TAG, "MBEDTLS_ERR_PKCS5_BAD_INPUT_DATA");
            break;
        case MBEDTLS_ERR_PKCS5_INVALID_FORMAT:
            ESP_LOGW(TAG, "MBEDTLS_ERR_PKCS5_INVALID_FORMAT");
            break;
        case MBEDTLS_ERR_PKCS5_FEATURE_UNAVAILABLE:
            ESP_LOGW(TAG, "MBEDTLS_ERR_PKCS5_FEATURE_UNAVAILABLE");
            break;
        case MBEDTLS_ERR_PKCS5_PASSWORD_MISMATCH:
            ESP_LOGW(TAG, "MBEDTLS_ERR_PKCS5_PASSWORD_MISMATCH");
            break;
        default:
            ESP_LOGW(TAG, "mbedtls_pkcs5_pbkdf2_hmac returned unknown error: %02x", ret);
            break;
    }
    // Clean up
    mbedtls_md_free(&mdctx);
}

static void db_espnow_deinit(db_espnow_send_param_t *send_param) {
    free(send_param);
    vSemaphoreDelete(db_espnow_send_recv_callback_queue);
    vSemaphoreDelete(db_espnow_send_queue); // ToDo: Check if that is a good idea - used by other task
    esp_now_deinit();
}

/**
 * Encrypts and authenticates a DroneBridge for ESP32 ESP-NOW packet with its payload using AES-GCM 256
 * Beware we are using the same key for both communication directions.
 * @param db_esp_now_packet Packet containing payload data and header info. AES IV & TAG will be filled
 * @return 0 on success - payload is encrypted and part of the db_esp_now_packet
 */
int db_encrypt_payload(db_esp_now_packet_t* db_esp_now_packet) {
    // Generate random IV
    esp_fill_random(db_esp_now_packet->db_esp_now_packet_header.aes_iv, DB_ESPNOW_AES_IV_LEN);
    int ret = mbedtls_gcm_crypt_and_tag(
            &aes,
            MBEDTLS_GCM_ENCRYPT,
            size_protected_data,    // ToDo: Make ESP-NOW packet length variable
            db_esp_now_packet->db_esp_now_packet_header.aes_iv, DB_ESPNOW_AES_IV_LEN,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_header, size_packet_header,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_protected_data,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_protected_data,
            DB_ESPNOW_AES_TAG_LEN, db_esp_now_packet->tag
    );

    if (ret == 0) {
        return 0;
    } else {
        if (ret == MBEDTLS_ERR_GCM_BAD_INPUT) {
            ESP_LOGW(TAG, "mbedtls_gcm_crypt_and_tag returned MBEDTLS_ERR_GCM_BAD_INPUT");
        } else {
            ESP_LOGW(TAG, "mbedtls_gcm_crypt_and_tag returned unknown error");
        }
        return -1;
    }
}

/**
 * Decrypt the DroneBridge ESP-NOW packet payload and authenticate its content using AES-GCM 256
 * @param db_esp_now_packet
 * @param decrypt_out_buffer payload length and payload as an array of bytes
 * @return returns result of mbedtls_gcm_auth_decrypt e.g. 0 on success and valid data
 */
int db_decrypt_payload(db_esp_now_packet_t* db_esp_now_packet, db_esp_now_packet_protected_data_t* decrypt_out_buffer) {
    int ret_decrypt = mbedtls_gcm_auth_decrypt(
            &aes,
            size_protected_data,    // ToDo: Make ESP-NOW packet length variable
            db_esp_now_packet->db_esp_now_packet_header.aes_iv, DB_ESPNOW_AES_IV_LEN,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_header, size_packet_header,
            db_esp_now_packet->tag, DB_ESPNOW_AES_TAG_LEN,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_protected_data,
            (uint8_t*) decrypt_out_buffer
    );
    switch (ret_decrypt) {
        case 0:
            // Successfully decrypted and authenticated
            break;
        case MBEDTLS_ERR_GCM_AUTH_FAILED:
            ESP_LOGW(TAG, "MBEDTLS_ERR_GCM_AUTH_FAILED: Authenticated decryption failed."); break;
        case MBEDTLS_ERR_GCM_BAD_INPUT:
            ESP_LOGW(TAG, "MBEDTLS_ERR_GCM_BAD_INPUT: Bad input parameters to function."); break;
        default:
            ESP_LOGW(TAG, "Unknown mbedtls_gcm_auth_decrypt error %i", ret_decrypt); break;
    }
    return ret_decrypt;
}

/**
 * Tries to read one entry from the ESP-NOW send queue (filled by the UART task) and sends the data via broadcast.
 * Only call this function when the last ESP-NOW send callback has returned! Otherwise order of packets is not guaranteed
 *
 * @return false if no packet was sent, true if packet was sent (actual sending will be confirmed by the send-callback)
 */
bool db_read_uart_queue_and_send() {
    db_esp_now_send_event_t evt;
    if (xQueueReceive(db_espnow_send_queue, &evt, 0) == pdTRUE) {
        static const uint8_t PACKET_LEN = sizeof(db_esp_now_packet_t); // ToDO: Make ESP-NOW packet length variable
        static db_esp_now_packet_t db_esp_now_packet = {.db_esp_now_packet_header.seq_num = 0,
                                                        .db_esp_now_packet_header.packet_type = 0};   // make static so it is gets instanced only once
        if (DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_GND) {
            db_esp_now_packet.db_esp_now_packet_header.origin = DB_ESPNOW_ORIGIN_GND;
        } else {
            db_esp_now_packet.db_esp_now_packet_header.origin = DB_ESPNOW_ORIGIN_AIR;
        }
        // ToDo: Make sure the payload is padded properly

        db_esp_now_packet.db_esp_now_packet_protected_data.payload_length_decrypted = evt.data_length;
        memcpy(db_esp_now_packet.db_esp_now_packet_protected_data.payload, evt.send_buf, evt.data_length);  //ToDo: Check if memcpy can be removed
        db_encrypt_payload(&db_esp_now_packet);
        // peer addr set to NULL so all peers in the peer-list get the packet
        if (esp_now_send(NULL, (const uint8_t *) &db_esp_now_packet, PACKET_LEN) != ESP_OK) {
            ESP_LOGE(TAG, "Error sending ESP-NOW data!");
            return false;
        } else {
            if (db_esp_now_packet.db_esp_now_packet_header.seq_num < UINT32_MAX)
                db_esp_now_packet.db_esp_now_packet_header.seq_num++;    // packet is static so just increase number once we sent it
            else
                db_esp_now_packet.db_esp_now_packet_header.seq_num = 0;  // catch overflow in case someone got crazy
            return true;
        }
    } else {
        // nothing to do - Queue is empty
    }
    return false;
}

/**
 * ESP-NOW sending callback function is called in WiFi task.
 * Do not do lengthy operations from this task. Instead, post necessary data to a queue and handle it from the control
 * module task.
 * Note that too short interval between sending two ESP-NOW data may lead to disorder of sending callback function.
 * So, it is recommended that sending the next ESP-NOW data after the sending callback function of the previous sending
 * has returned.
 */
static void db_esp_now_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send callback arg error");
        return;
    }

    if (status == ESP_NOW_SEND_FAIL) {
        ESP_LOGE(TAG, "Failed to send ESP-NOW packet - MAC: %02xh:%02xh:%02xh:%02xh:%02xh:%02xh",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    }

    db_espnow_event_t evt;
    db_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    evt.id = DB_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(db_espnow_send_recv_callback_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send to send queue fail");
    }
}

/**
 * ESP-NOW receiving callback function is called in WiFi task. This is the entry point when new data is incoming.
 * Do not do lengthy operations from this task. Instead, post necessary data to a queue and handle it from the control
 * module task
 */
static void db_esp_now_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    db_espnow_event_t evt;
    db_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *src_addr = recv_info->src_addr;
    uint8_t *des_addr = recv_info->des_addr;

    if (src_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive callback arg error");
        return;
    }

    if (!IS_BROADCAST_ADDR(des_addr)) {
        ESP_LOGD(TAG, "Receive unicast ESP-NOW data. Ignoring");
        return;
    }

    if (data[0] == DB_ESPNOW_ORIGIN_GND && DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_GND) {
        ESP_LOGD(TAG, "Receive ESP-NOW data. Ignoring - Came from another GND station.");
        return;
    }
    if (data[0] == DB_ESPNOW_ORIGIN_AIR && DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_AIR) {
        ESP_LOGD(TAG, "Receive ESP-NOW data. Ignoring - Came from another AIR station.");
        return;
    }

    evt.id = DB_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, src_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(db_espnow_send_recv_callback_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send to receive queue fail");
        free(recv_cb->data);
    }
}

/**
 *  Task that handles all ESP-NOW releated data processing. Reads ESP-NOW Callback-Queue, Reads ESP-NOW send queue and
 *  writes to UART-WRITE Queue.
 *
 */
_Noreturn void process_espnow_data() {
    db_espnow_event_t evt;
    uint16_t recv_seq = 0;
    bool ready_to_send = false; // indicator that the last ESP-NOW send callback returned -> we can send the next packet
    int ret;
    // ToDo: Send something initially via ESP-NOW so we end up in the loop

    while(1) {
        while (xQueueReceive(db_espnow_send_recv_callback_queue, &evt, 0) == pdTRUE) {
            switch (evt.id) {
                case DB_ESPNOW_SEND_CB: {
                    db_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                    ESP_LOGD(TAG, "Send data to "MACSTR", status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
                    // indicate that we can send next packet - the last sending callback has returned, so we keep the order
                    // of packets. ESP-NOW by default does not guarantee the order when many packets are sent in quick succession
                    ready_to_send = true;
                    // try to immediately send the next packet if available and set ready_to_send accordingly
                    ready_to_send = !db_read_uart_queue_and_send();
                    break;
                }
                case DB_ESPNOW_RECV_CB: {
                    db_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                    ret = db_espnow_process_rcv_data(recv_cb->data, recv_cb->data_len);
                    free(recv_cb->data);
                    if (ret == DB_ESPNOW_DATA_BROADCAST) {
                        ESP_LOGD(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq,
                                 MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    } else if (ret == DB_ESPNOW_DATA_UNICAST) {
                        ESP_LOGD(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq,
                                 MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    } else {
                        ESP_LOGW(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                    }
                    break;
                }
                default:
                    ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                    break;
            }
        }
        // process UART -> ESP-NOW
        if (ready_to_send) {
            // send the next packet if available and set ready_to_send accordingly
            ready_to_send = !db_read_uart_queue_and_send();
        } else {
            // do nothing - we are not ready for sending another packet
        }
    }
}

int init_gcm_encryption_module() {
    mbedtls_gcm_init(&aes);
    uint8_t aes_key[AES_256_KEY_BYTES];
    generate_pkcs5_key((const char*) DB_WIFI_PWD, aes_key, AES_256_KEY_BYTES);
    int ret = mbedtls_gcm_setkey(&aes, MBEDTLS_CIPHER_ID_AES, aes_key, DB_ESPNOW_AES_KEY_LEN);
    return ret;
}

void deinit_espnow_all(){
    mbedtls_gcm_free(&aes);
    vSemaphoreDelete(db_espnow_send_recv_callback_queue);
    vSemaphoreDelete(db_espnow_send_queue); // ToDo: Check if that is a good idea
    esp_now_deinit();
}

esp_err_t db_espnow_init() {
    ESP_LOGI(TAG, "Initializing up ESP-NOW parameters");
    db_espnow_send_recv_callback_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(db_espnow_event_t));
    if (db_espnow_send_recv_callback_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(db_esp_now_send_callback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(db_esp_now_receive_callback));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        deinit_espnow_all();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = DB_WIFI_CHANNEL;
    peer->ifidx = WIFI_IF_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    if (DB_TRANS_BUF_SIZE > DB_ESPNOW_PAYLOAD_MAXSIZE) {
        DB_TRANS_BUF_SIZE = DB_ESPNOW_PAYLOAD_MAXSIZE;  // Limit payload size to the max we can do
    } else if (DB_TRANS_BUF_SIZE % 16 == 0) {
        // All good do nothing, we can only accept multiples of 16.
    } else {
        DB_TRANS_BUF_SIZE = 64;   // Default value in case someone fucked up the config
    }

    int ret = init_gcm_encryption_module();
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_gcm_setkey returned an error: %i", ret);
        deinit_espnow_all();
        return ESP_FAIL;
    }

    return ESP_OK;
}