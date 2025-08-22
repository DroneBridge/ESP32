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

#include <sys/cdefs.h>
#include <esp_err.h>
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

#include <db_parameters.h>

#include "globals.h"
#include "main.h"
#include "espnow.h"

#define TAG "DB_ESPNOW"

const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static QueueHandle_t db_espnow_send_recv_callback_queue;    // Queue that contains ESP-NOW callback results
QueueHandle_t db_espnow_send_queue;    // Queue that contains data to be sent via ESP-NOW (filled by control task)
QueueHandle_t db_uart_write_queue;    // Queue that contains data to be written to UART (filled by ESP-NOW task)
db_esp_now_clients_list_t *db_esp_now_clients_list; // Local list of known ESP-NOW peers with the last RSSI value and noise floor and sequence number

// packet that is filled with payload and sent. Updates the sequ. number with every send.
static db_esp_now_packet_t db_esp_now_packet_global = {
        .db_esp_now_packet_header.seq_num = 0,
        .db_esp_now_packet_header.packet_type = DB_ESP_NOW_PACKET_TYPE_DATA
};   // make static so it gets instanced only once

mbedtls_gcm_context aes;
uint8_t const db_esp_now_packet_header_len = sizeof(db_esp_now_packet_header_t);

/**
 * Generates a AES key from a password using pkcs5 - pbkdf2 and mbedTLS
 *
 * @param password The password that gets transformed to PKCS5 key used for encryption
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

/**
 * Encrypts and authenticates a DroneBridge for ESP32 ESP-NOW packet with its payload using AES-GCM 256
 * Beware we are using the same key for both communication directions.
 * Calls mbedtls_gcm_crypt_and_tag()
 *
 * @param db_esp_now_packet Packet containing payload data and header info. AES IV & TAG will be filled
 * @param encrypt_payload_len Length of the to be encrypted data (db_esp_now_packet_protected_data)
 * @return 0 on success, -1 in case of unknown error of mbedtls_gcm_crypt_and_tag or return value of mbedtls_gcm_crypt_and_tag
 * payload is encrypted and part of the db_esp_now_packet
 */
int db_encrypt_payload(db_esp_now_packet_t* db_esp_now_packet, uint8_t encrypt_payload_len) {
    // Generate random IV - This is risky since (password+IV) shall never be reused!
    // Counter would secure for a single session but super unsecure with multiple sessions where the password is not changed
    esp_fill_random(db_esp_now_packet->db_esp_now_packet_header.aes_iv, DB_ESPNOW_AES_IV_LEN);
    int ret = mbedtls_gcm_crypt_and_tag(
            &aes,
            MBEDTLS_GCM_ENCRYPT,
            encrypt_payload_len,
            db_esp_now_packet->db_esp_now_packet_header.aes_iv, DB_ESPNOW_AES_IV_LEN,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_header, db_esp_now_packet_header_len,
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
 *
 * @param db_esp_now_packet db_esp_now_packet_protected_data_t structure wth the data de decrypt inside
 * @param decrypt_out_buffer Pointer to the out buffer for the decrypted data
 * @param len_encrypted_data length of the buffer (db_esp_now_packet_protected_data)
 * @return returns result of mbedtls_gcm_auth_decrypt e.g. 0 on success and valid data
 */
int db_decrypt_payload(db_esp_now_packet_t* db_esp_now_packet, uint8_t* decrypt_out_buffer,
                       uint8_t len_encrypted_data) {
    int ret_decrypt = mbedtls_gcm_auth_decrypt(
            &aes,
            len_encrypted_data,
            db_esp_now_packet->db_esp_now_packet_header.aes_iv, DB_ESPNOW_AES_IV_LEN,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_header, db_esp_now_packet_header_len,
            db_esp_now_packet->tag, DB_ESPNOW_AES_TAG_LEN,
            (uint8_t*) &db_esp_now_packet->db_esp_now_packet_protected_data,
            decrypt_out_buffer
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
 * Encrypts, authenticates and sends packet via ESP-NOW
 * @param db_esp_now_packet Packet to send
 * @param payload_length Value of payload_length_decrypted of db_esp_now_packet - saves us from creating a var inside the function
 * @return false if no packet was sent, true if packet was sent (actual sending will be confirmed by the send-callback)
 */
bool db_espnow_encrypt_auth_send(db_esp_now_packet_t *db_esp_now_packet, const uint8_t payload_length) {
    static int ret;
    static esp_err_t err;
    // after encryption the payload_length_decrypted will be unreadable -> keep a copy (payload_length) for sending the packet
    // const uint8_t payload_length = db_esp_now_packet->db_esp_now_packet_protected_data.payload_length_decrypted;
    ret = db_encrypt_payload(db_esp_now_packet, db_esp_now_packet->db_esp_now_packet_protected_data.payload_length_decrypted + 1);
    if (ret == 0) {
        err = esp_now_send(BROADCAST_MAC, (const uint8_t *) db_esp_now_packet, (db_esp_now_packet_header_len + DB_ESPNOW_AES_TAG_LEN + payload_length + 1));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error (%s) sending ESP-NOW data - size %i !", esp_err_to_name(err), (db_esp_now_packet_header_len + DB_ESPNOW_AES_TAG_LEN + payload_length + 1));
            return false;
        } else {
            return true;
        }
    } else {
        ESP_LOGE(TAG, "db_encrypt_payload returned error code: %i, not sending packet", ret);
        return false;
    }
}

/**
 * Tries to read one entry from the ESP-NOW send queue (mainly filled by the UART task) and sends the data via broadcast.
 * Only call this function when the last ESP-NOW send callback has returned! Otherwise order of packets is not guaranteed
 *
 * @return false if no packet was sent, true if packet was sent (actual sending will be confirmed by the send-callback)
 */
bool db_read_uart_queue_and_send() {
    static db_espnow_queue_event_t evt;
    // Receive data from Queue that was put there by other tasks to be sent via ESP-NOW
    if (db_espnow_send_queue != NULL && xQueueReceive(db_espnow_send_queue, &evt, 0) == pdTRUE) {
        db_esp_now_packet_global.db_esp_now_packet_header.packet_type = evt.packet_type;
        if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND) {
            db_esp_now_packet_global.db_esp_now_packet_header.origin = DB_ESPNOW_ORIGIN_GND;
        } else {
            db_esp_now_packet_global.db_esp_now_packet_header.origin = DB_ESPNOW_ORIGIN_AIR;
        }
        // Create an DroneBridge for ESP32 ESP-NOW packet
        db_esp_now_packet_global.db_esp_now_packet_protected_data.payload_length_decrypted = evt.data_len;
        // ToDo: Potential to optimize: esp_now_send() does not need an instance of buffer after sending, however using evt.data as pointer resulted in errors
        memcpy(db_esp_now_packet_global.db_esp_now_packet_protected_data.payload, evt.data, evt.data_len);
        free(evt.data);
        // Encrypt, authenticate and send packet
        if(db_espnow_encrypt_auth_send(&db_esp_now_packet_global, evt.data_len)) {
            // update sequence number for next packet
            if (db_esp_now_packet_global.db_esp_now_packet_header.seq_num < UINT32_MAX) {
                db_esp_now_packet_global.db_esp_now_packet_header.seq_num++;    // packet is static so just increase number once we sent it
            } else {
                db_esp_now_packet_global.db_esp_now_packet_header.seq_num = 0;  // catch overflow in case someone got crazy
            }
            return true;
        } else {
            return false;
        }
    } else {
        if (db_espnow_send_queue == NULL) {
            ESP_LOGE(TAG, "db_espnow_send_queue is NULL!");
        } else {
            // nothing to do - Queue is empty
        }

    }
    return false;
}

/**
 * Add a new ESP-NOW broadcast peer to the list of known ESP-NOW peers. Checks if peer is already known based on MAC.
 * List is then used to keep track of the RSSI, packet loss and signal quality on GND side.
 *
 * @param esp_now_clients List of broadcast peers (ESP-NOW) with their RSSI
 * @param broadcast_peer_mac MAC address of a potentially new broadcast peer
 * @return index of the peer inside the list or -1 in case of error e.g. if list is full or not initialized
 */
int16_t update_peer_list(db_esp_now_clients_list_t *esp_now_clients, uint8_t broadcast_peer_mac[6]) {
    if (esp_now_clients == NULL) { // Check if the list is NULL
        return -1; // Do nothing
    }
    for (uint8_t i = 0; i < esp_now_clients->size; i++) {
        if (memcmp(esp_now_clients->db_esp_now_bpeer_info[i].broadcast_peer_mac, broadcast_peer_mac, ESP_NOW_ETH_ALEN) == 0) {
            // found the client - he is already part of the known clients list
            return i;
        }
    }
    if (esp_now_clients->size == DB_ESPNOW_MAX_BROADCAST_PEERS) { // Check if the list is full
        return -1; // Do nothing - list is full already
    } else {
        // Add new peer to list - Copy the element data to the end of the array
        db_esp_now_bpeer_info_t db_esp_now_bpeer_info = {.gnd_rssi = -127, .last_seq_num = 0, .gnd_rx_lost_packets = 0};
        memcpy(db_esp_now_bpeer_info.broadcast_peer_mac, broadcast_peer_mac, ESP_NOW_ETH_ALEN);
        esp_now_clients->db_esp_now_bpeer_info[esp_now_clients->size] = db_esp_now_bpeer_info;
        esp_now_clients->size++; // Increment the size of the list
        return (esp_now_clients->size-1) ;
    }
}

/**
 * Steps to process received ESPNOW data. ESP firmware ensures that only correct packets are forwarded to us
 * 1. Decrypt & authenticate
 * 2. Check if we know the client based on MAC - update rssi in case we are GND - AIR side only expects to only ever have on GND peer anyways
 * 3. Check sequence number and if we are GND station then update lost packet count based on seq. numbers
 * 4. Write payload to uart-queue so it can be processed by the control_espnow task
 *
 * @param data Received raw data via ESP-NOW
 * @param data_len  Length of received data
 * @param src_addr Source MAC address of the data
 * @param rssi RSSI in dBm of this packet
 */
void db_espnow_process_rcv_data(uint8_t *data, uint16_t data_len, uint8_t *src_addr, int8_t rssi) {
    db_esp_now_packet_t *db_esp_now_packet = (db_esp_now_packet_t *) data;
    uint8_t len_payload = data_len - DB_ESPNOW_AES_TAG_LEN - db_esp_now_packet_header_len;
    uint8_t db_decrypted_data[len_payload];

    /* Decrypt and authenticate packet - only then process its contents */
    if (db_decrypt_payload(db_esp_now_packet, db_decrypted_data, len_payload) == 0) {
        /* Check if we know that peer already */
        int16_t peer_index = update_peer_list(db_esp_now_clients_list, src_addr);
        if (peer_index != -1) {
            if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND) {
                // update the list with the rssi only if we are GND
                db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].gnd_rssi = rssi;
            } else {
                // AIR unit keeps track of RSSI using db_esp_signal_quality variable
                // no need to do here anything
                // TODO: Clean up RSSI variables - a bit confusing that AIR and GND use different structures
            }
            // check packet sequence number
            if (db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].last_seq_num < db_esp_now_packet->db_esp_now_packet_header.seq_num) {
                // Count the lost packets per peer since the last received packet
                db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].gnd_rx_lost_packets +=
                        (db_esp_now_packet->db_esp_now_packet_header.seq_num -
                        db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].last_seq_num - 1);
            } else {
                ESP_LOGW(TAG, "Sequence number lower than expected. Sender did reset or packet may be part of a replay attack.");
                // accept packet anyway for now to make for a more robust link
                db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].gnd_rx_lost_packets = 0;
            }

            db_esp_now_clients_list->db_esp_now_bpeer_info[peer_index].last_seq_num = db_esp_now_packet->db_esp_now_packet_header.seq_num;
        } else {
            /* Do nothing since we only count packet loss and rssi on the GND side not on the air side
             * and only if there is still a free spot in the list */
        }

        /* Process packet depending on packet type */
        if (db_esp_now_packet->db_esp_now_packet_header.packet_type == DB_ESP_NOW_PACKET_TYPE_DATA) {
            // Pass data to UART queue
            db_espnow_queue_event_t db_uart_evt;
            db_uart_evt.data_len = db_decrypted_data[0];    // should be equal to len_payload-1 if everything worked out
            db_uart_evt.data = malloc(db_uart_evt.data_len);
            // For some reason it seems we cannot directly decrypt to db_espnow_uart_event_t -> Queues get set to NULL
            memcpy(db_uart_evt.data, &db_decrypted_data[1], db_uart_evt.data_len);
            if (xQueueSend(db_uart_write_queue, &db_uart_evt, ESPNOW_MAXDELAY) != pdTRUE) {
                ESP_LOGW(TAG, "Send to db_uart_write_queue failed");
                free(db_uart_evt.data);
            } else {
                // all good
            }
        } else if (db_esp_now_packet->db_esp_now_packet_header.packet_type == DB_ESP_NOW_PACKET_TYPE_INTERNAL_TELEMETRY) {
            /* This is only called on the AIR side since GND sends telemetry to AIR only */
            db_esp_now_clients_list_t *clients = (db_esp_now_clients_list_t*) &db_decrypted_data[1];
            ESP_LOGD(TAG, "Received internal telemetry frame containing %i entries", clients->size);
            for (int i = 0; i < clients->size; i++) {
                if (memcmp(LOCAL_MAC_ADDRESS, clients->db_esp_now_bpeer_info[i].broadcast_peer_mac, ESP_NOW_ETH_ALEN) == 0) {
                    // found us (this local ESP32 AIR unit) -> update internal telemetry buffer,
                    // so it gets sent with next Mavlink RADIO STATUS in case MAVLink radio status is enabled
                    db_esp_signal_quality.gnd_noise_floor = clients->gnd_noise_floor;
                    db_esp_signal_quality.gnd_rssi = clients->db_esp_now_bpeer_info[i].gnd_rssi;
                    db_esp_signal_quality.gnd_rx_packets_lost = clients->db_esp_now_bpeer_info[i].gnd_rx_lost_packets;
                    break;
                } else {
                    // keep on looking for our MAC
                }
            }
        } else {
            ESP_LOGW(TAG, "Received unknown DroneBridge for ESP32 ESP-NOW packet type");
        }
    } else {
        ESP_LOGE(TAG, "Failed to Decrypt & Authenticate. Ignoring");
        return;
    }
}

/**
 * ESP-NOW sending callback function is called in WiFi task.
 * Do not do lengthy operations from this task. Instead, post necessary data to a queue and handle it from the control
 * module task.
 * Note that too short interval between sending two ESP-NOW data may lead to disorder of sending callback function.
 * So, it is recommended that sending the next ESP-NOW data after the sending callback function of the previous sending
 * has returned.
 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void db_espnow_send_callback(const esp_now_send_info_t *esp_now_send_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_FAIL) {
        ESP_LOGE(TAG, "Failed to send ESP-NOW packet - MAC: %02xh:%02xh:%02xh:%02xh:%02xh:%02xh",
                 esp_now_send_info->des_addr[0], esp_now_send_info->des_addr[1], esp_now_send_info->des_addr[2], esp_now_send_info->des_addr[3], esp_now_send_info->des_addr[4], esp_now_send_info->des_addr[5]);
    }
#else
static void db_espnow_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send callback arg error");
        return;
    }

    if (status == ESP_NOW_SEND_FAIL) {
        ESP_LOGE(TAG, "Failed to send ESP-NOW packet - MAC: %02xh:%02xh:%02xh:%02xh:%02xh:%02xh",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    }
#endif

    db_espnow_event_t evt;
    db_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    evt.id = DB_ESPNOW_SEND_CB;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
    memcpy(send_cb->mac_addr, esp_now_send_info->des_addr, ESP_NOW_ETH_ALEN);
#else
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
#endif
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
static void db_espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    db_espnow_event_t evt;
    db_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *src_addr = recv_info->src_addr;
    uint8_t *des_addr = recv_info->des_addr;

    if (src_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive callback arg error");
        return;
    }

    if (!IS_BROADCAST_ADDR(des_addr)) {
        ESP_LOGD(TAG, "Receive uni-cast ESP-NOW data. Ignoring");
        return;
    }

    if (data[0] == DB_ESPNOW_ORIGIN_GND && DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND) {
        // Ignoring packet - Came from another GND station
        return;
    } else {
        // we are GND and packet is for us from AIR
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C2) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C6)
        db_esp_now_clients_list->gnd_noise_floor = (int8_t ) recv_info->rx_ctrl->noise_floor;
        // rest RSSI will be processed in process_espnow_data()
#endif
    }

    if (data[0] == DB_ESPNOW_ORIGIN_AIR && DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_AIR) {
        // Ignoring packet - Came from another AIR station
        return;
    } else {
        // we are AIR a packet is for us from GND - we only expect one GND station to be talking to us
        db_esp_signal_quality.air_rssi = recv_info->rx_ctrl->rssi;
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C2) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C6)
        db_esp_signal_quality.air_noise_floor = recv_info->rx_ctrl->noise_floor;
#endif
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
    recv_cb->rssi = recv_info->rx_ctrl->rssi;
    if (db_espnow_send_recv_callback_queue != NULL && xQueueSend(db_espnow_send_recv_callback_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send to receive queue fail");
        free(recv_cb->data);
    }
}

/**
 *  Init/Create structure containing all ESP-NOW broadcast connection quality information
 *  List is used to keep track of the RSSI and signal quality on GND side.
 * @return Structure containing all ESP-NOW broadcast connection quality information
 */
db_esp_now_clients_list_t *db_espnow_broadcast_peers_list_create() {
    db_esp_now_clients_list_t *client_list = malloc(sizeof(db_esp_now_clients_list_t)); // Allocate memory for the list
    if (client_list == NULL) { // Check if the allocation failed
        return NULL; // Return NULL to indicate an error
    }
    client_list->size = 0; // Initialize the size to 0
    return client_list; // Return the pointer to the list
}

/**
 * Destroy structure containing all ESP-NOW broadcast connection quality information
 * @param db_esp_now_clients Structure containing all ESP-NOW broadcast connection quality information
 */
void db_espnow_broadcast_peers_list_destroy(db_esp_now_clients_list_t *esp_now_clients) {
    if (esp_now_clients == NULL) { // Check if the list is NULL
        return; // Do nothing
    }
    free(esp_now_clients); // Free the list
}

/**
 * Init mbedtls aes gcm mode and set encryption key based on the WiFi password specified by the user
 * @param aes_key buffer for saving the generated aes key of len AES_256_KEY_BYTES
 * @return result of mbedtls_gcm_setkey
 */
int init_gcm_encryption_module(uint8_t *aes_key) {
    mbedtls_gcm_init(&aes);
    generate_pkcs5_key(DB_PARAM_PASS, aes_key, AES_256_KEY_BYTES);
    ESP_LOGI(TAG, "Derived AES Key:");
    for (int i = 0; i < AES_256_KEY_BYTES; ++i) {
        printf("%02x", aes_key[i]);
    }
    printf("\n");
    int ret = mbedtls_gcm_setkey(&aes, MBEDTLS_CIPHER_ID_AES, aes_key, DB_ESPNOW_AES_KEY_LEN);
    return ret;
}

void deinit_espnow_all(){
    ESP_LOGW(TAG, "De init ESPNOW incl. Queues & AES");
    mbedtls_gcm_free(&aes);
    vSemaphoreDelete(db_espnow_send_recv_callback_queue);
    vSemaphoreDelete(db_espnow_send_queue); // ToDo: Check if that is a good idea since control task might be using it
    vSemaphoreDelete(db_uart_write_queue); // ToDo: Check if that is a good idea since control task might be using it
    db_espnow_broadcast_peers_list_destroy(db_esp_now_clients_list);
    esp_now_deinit();
}

/**
 * Init all relevant structures and Queues for ESP-NOW communication
 * @return ESP_FAIL on failure or ESP_OK on success
 */
esp_err_t db_espnow_init() {
    ESP_LOGI(TAG, "Initializing ESP-NOW parameters");
    db_esp_now_clients_list = db_espnow_broadcast_peers_list_create();
    /* Init Queue for ESP-NOW internal callbacks when packet is finally sent or received */
    db_espnow_send_recv_callback_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(db_espnow_event_t));
    if (db_espnow_send_recv_callback_queue == NULL) {
        ESP_LOGE(TAG, "Create db_espnow_send_recv_callback_queue mutex fail");
        return ESP_FAIL;
    }

    /* Init Queues for communication with control task */
    db_espnow_send_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(db_espnow_queue_event_t));
    db_uart_write_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(db_espnow_queue_event_t));
    if (db_espnow_send_queue == NULL) {
        ESP_LOGE(TAG, "Create db_espnow_send_queue mutex fail");
        return ESP_FAIL;
    }
    if (db_uart_write_queue == NULL) {
        ESP_LOGE(TAG, "Create db_uart_write_queue mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESP-NOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(db_espnow_send_callback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(db_espnow_receive_callback));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    if (!esp_now_is_peer_exist(BROADCAST_MAC)) ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    /* Limit payload size to the max we can do */
    if (DB_PARAM_SERIAL_PACK_SIZE > DB_ESPNOW_PAYLOAD_MAXSIZE || DB_PARAM_SERIAL_PACK_SIZE < 1) {
        DB_PARAM_SERIAL_PACK_SIZE = DB_ESPNOW_PAYLOAD_MAXSIZE;
    } else {
        // all good
    }

    /* Init AES GCM encryption module */
    uint8_t aes_key[AES_256_KEY_BYTES];
    int ret = init_gcm_encryption_module(aes_key);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_gcm_setkey returned an error: %i", ret);
        deinit_espnow_all();
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_now_set_pmk(aes_key));  // only first 16 bytes will be used

    if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND) {
        ESP_LOGI(TAG, "ESP-NOW for DroneBridge init done - acting as ESP-NOW GND station");
    } else {
        ESP_LOGI(TAG, "ESP-NOW for DroneBridge init done - acting as ESP-NOW AIR station");
    }
    return ESP_OK;
}

/**
 * Send ESP-NOW DroneBridge for ESP32 internal telemetry packet containing RSSI and noise floor info from GND to AIR.
 * Used by AIR side to create a RADIO_STATUS MAVLink message sent to GCS.
 * @return true when packet was scheduled for sending, false if it will not be sent
 */
bool db_espnow_schedule_internal_telemetry_packet() {
    if (DB_PARAM_RADIO_MODE != DB_WIFI_MODE_ESPNOW_GND) {
        // Only GND sends internal telemetry. This function was called wrongly
        return false;
    } else {
        // continue
    }
    // ToDo: Split list if longer than max payload size
    if (sizeof(db_esp_now_clients_list_t) > DB_ESPNOW_PAYLOAD_MAXSIZE) {
        ESP_LOGE(TAG, "Size of db_esp_now_clients_list_t is > %i", DB_ESPNOW_PAYLOAD_MAXSIZE);
        return false;
    } else {
        /* continue */
    }
    ESP_LOGD(TAG, "Scheduling int. telem. packet");
    uint8_t payload_size = sizeof(db_esp_now_clients_list_t);

    db_espnow_queue_event_t evt;
    // Set payload of DroneBridge for ESP32 ESP-NOW packet
    evt.data = malloc(payload_size);
    memcpy(evt.data, db_esp_now_clients_list, payload_size);
    evt.data_len = payload_size;
    evt.packet_type = DB_ESP_NOW_PACKET_TYPE_INTERNAL_TELEMETRY;
    if (xQueueSend(db_espnow_send_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send to db_espnow_send_queue queue fail");
        free(evt.data);
        return false;
    } else {
        // all good
        return true;
    }
}

/**
 *  Task that handles all ESP-NOW related data processing. Reads ESP-NOW Callback-Queue, Reads ESP-NOW send queue and
 *  writes to UART-WRITE Queue.
 */
_Noreturn void process_espnow_data() {
    esp_err_t err = db_espnow_init();
    if (err == ESP_OK) {
        // all good. continue
    } else {
        ESP_LOGE(TAG, "Failed to init espnow (db_espnow_init()) aborting start of db_espnow task");
        vTaskDelete(NULL);
    }
    db_espnow_event_t evt;
    // indicator that the last ESP-NOW send callback returned -> we can send the next packet
    bool ready_to_send = true; // initially true
    bool send_internal_telemetry_frame = false;    // flag that indicates a new internal telemetry frame shall be sent
    int delay_timer_cnt = 0;
    while(1) {
        if (db_espnow_send_recv_callback_queue == NULL) ESP_LOGE(TAG, "db_espnow_send_recv_callback_queue is NULL!");
        if(db_espnow_send_recv_callback_queue != NULL && xQueueReceive(db_espnow_send_recv_callback_queue, &evt, 0) == pdTRUE) {
            switch (evt.id) {
                case DB_ESPNOW_SEND_CB: {
                    // db_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;  // no needed for now
                    // indicate that we can send next packet - the last sending callback has returned, so we keep the order
                    // of packets. ESP-NOW by default does not guarantee the order when many packets are sent in quick succession
                    ready_to_send = true;
                    if (DB_PARAM_RADIO_MODE == DB_WIFI_MODE_ESPNOW_GND && send_internal_telemetry_frame) {
                        send_internal_telemetry_frame = !db_espnow_schedule_internal_telemetry_packet();
                    } else {
                        // try to immediately send the next packet if available and set ready_to_send accordingly
                        ready_to_send = !db_read_uart_queue_and_send();
                    }
                    break;
                }
                case DB_ESPNOW_RECV_CB: {
                    db_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                    db_espnow_process_rcv_data(recv_cb->data, recv_cb->data_len, recv_cb->mac_addr, recv_cb->rssi);
                    free(recv_cb->data);
                    break;
                }
                default:
                    ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                    break;
            }
        }
        /* process UART -> ESP-NOW */
        if (ready_to_send) {
            // send the next packet if available and set ready_to_send accordingly
            ready_to_send = !db_read_uart_queue_and_send();
        } else {
            // do nothing - we are not ready for sending another packet
        }

        if (delay_timer_cnt == 5000) {
            /* all actions are non-blocking so allow some delay so that the IDLE task of FreeRTOS and the watchdog can run
            read: https://esp32developer.com/programming-in-c-c/tasks/tasks-vs-co-routines for reference */
            vTaskDelay(10 / portTICK_PERIOD_MS);
            send_internal_telemetry_frame = true;   // use this timer trigger to schedule a new internal telemetry frame
            delay_timer_cnt = 0;
        } else {
            delay_timer_cnt++;
        }
    }
    vTaskDelete(NULL);
}

/**
 * Start task that handles ESP-NOW data
 */
void db_start_espnow_module() {
    xTaskCreate(&process_espnow_data, "db_espnow", 40960, NULL, 5, NULL);
}