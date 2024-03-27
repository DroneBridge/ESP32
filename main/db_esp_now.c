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
#include "db_esp_now.h"
#include "globals.h"
#include "main.h"

#define TAG "DB_ESPNOW"

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static QueueHandle_t db_espnow_send_recv_callback_queue;    // Queue that contains ESP-NOW callback results

mbedtls_gcm_context aes;

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

static void db_espnow_deinit(db_espnow_send_param_t *send_param) {
    free(send_param);
    vSemaphoreDelete(db_espnow_send_recv_callback_queue);
    vSemaphoreDelete(db_espnow_send_queue); // ToDo: Check if that is a good idea - used by other task
    esp_now_deinit();
}

/**
 * Beware we are using the same key for both communication directions.
 *
 * @param input_buff
 * @param input_length
 * @param output_buff Can be the same as input buffer according to mbedtls_gcm_crypt_and_tag
 * @param padded_output_length
 * @param iv
 * @param tag
 * @return
 */
int db_encrypt_payload(uint8_t *input_buff, uint8_t input_length, uint8_t *output_buff, uint8_t *padded_output_length,
                    uint8_t iv[DB_ESPNOW_AES_IV_LEN], uint8_t tag[DB_ESPNOW_AES_TAG_LEN], uint8_t *add_auth_data, uint8_t add_data_auth_len) {
//    // create padding to make input data a multiple of 16
//    int padded_input_len = 0;
//    int modulo16 = input_length % 16;
//    if (input_length < 16) {
//        padded_input_len = 16;
//    } else {
//        padded_input_len = (strlen(input_buff) / 16 + 1) * 16;   // ToDo: Check - We are not using strings!
//    }
//    // ToDo: Check implementation - we might not want to malloc
//    char *padded_input = (char *) malloc(padded_input_len);
//    if (!padded_input) {
//        ESP_LOGE(TAG, "Failed to allocate memory for encryption");
//        return -1;
//    }
//    memcpy(padded_input, input_buff, strlen(input_buff));
//    uint8_t pkc5_value = (17 - modulo16);
//    for (int i = input_length; i < padded_input_len; i++) {
//        padded_input[i] = pkc5_value;
//    }
//
//    esp_fill_random(iv, 16); // fill/generate random IV of 16 bytes
//    mbedtls_aes_crypt_cbc(&aes,
//                          MBEDTLS_AES_ENCRYPT,
//                          padded_input_len,
//                          iv,
//                          (unsigned char *) padded_input,
//                          output_buff);
    int ret = mbedtls_gcm_crypt_and_tag(&aes, MBEDTLS_GCM_ENCRYPT, input_length, iv, DB_ESPNOW_AES_IV_LEN, add_auth_data, add_data_auth_len, input_buff, output_buff, DB_ESPNOW_AES_TAG_LEN, tag);
    if (ret == 0) {
        return 1;
    } else {
        if (ret == MBEDTLS_ERR_GCM_BAD_INPUT) ESP_LOGE(TAG, "mbedtls_gcm_crypt_and_tag returned MBEDTLS_ERR_GCM_BAD_INPUT");
        return ESP_FAIL;
    }
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
        static db_esp_now_packet_t db_esp_now_packet = {.db_esp_now_packet_header.seq_num = 0,
                                                        .db_esp_now_packet_header.packet_type = 0};   // make static so it is gets instanced only once
        if (DB_WIFI_MODE == DB_WIFI_MODE_ESPNOW_GND) {
            db_esp_now_packet.db_esp_now_packet_header.origin = DB_ESPNOW_ORIGIN_GND;
        } else {
            db_esp_now_packet.db_esp_now_packet_header.origin = DB_ESPNOW_ORIGIN_AIR;
        }
        static uint8_t padded_output_length = 0;
        // that call can only be made because db_esp_now_send_event_t is byte-equal to db_esp_now_packet_payload_t
        db_encrypt_payload((uint8_t *) &evt,
                           sizeof(db_esp_now_send_event_t),
                        (uint8_t *) &db_esp_now_packet.db_esp_now_packet_payload,
                        &padded_output_length,
                        db_esp_now_packet.aes_iv,
                        db_esp_now_packet.tag,
                        (uint8_t *) &db_esp_now_packet.db_esp_now_packet_header,
                        DB_ESPNOW_PACKET_HEADER_LENGTH);
        // peer addr set to NULL so all peers in the peer-list get the packet
        if (esp_now_send(NULL, (const uint8_t *) &db_esp_now_packet, padded_output_length + DB_ESPNOW_PACKET_HEADER_LENGTH) != ESP_OK) {
            ESP_LOGE(TAG, "Error sending ESP-NOW data!");
            return false;
        } else {
            if (db_esp_now_packet.db_esp_now_packet_header.seq_num < UINT32_MAX)
                db_esp_now_packet.db_esp_now_packet_header.seq_num++;    // packet is static so just increase number once we sent it
            else
                db_esp_now_packet.db_esp_now_packet_header.seq_num = 0;  // catch overflow in case someone got crazy
            return true;
        }
    }
    return false;
}

void db_esp_now_receive() {
    mbedtls_gcm_auth_decrypt();
    // TODO: Decrypt AES payload
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
        }
    }
}

void derive_aes_key(unsigned char aes_key[DB_ESPNOW_AES_KEY_LEN/8]) {
    // ToDo: Create an AES key based of the given WiFi password
}

int init_gcm_encryption_module() {
    unsigned char aes_key[DB_ESPNOW_AES_KEY_LEN/8];
    derive_aes_key(aes_key);
    mbedtls_gcm_init(&aes);
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
    db_espnow_send_param_t *send_param;

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

    if (DB_TRANS_BUF_SIZE > 224) {
        send_param->len = 224;  // ESP-NOW max payload size is 250 bytes
    } else if (DB_TRANS_BUF_SIZE % 16 == 0) {
        send_param->len = DB_TRANS_BUF_SIZE;   // Because of AES encryption we can only accept multiples of 16.
    } else {
        send_param->len = 64;   // Default value in case someone fucked up the config
    }

    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);

    int ret = init_gcm_encryption_module();
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_gcm_setkey returned an error: %i", ret);
        deinit_espnow_all();
        return ESP_FAIL;
    }

    return ESP_OK;
}