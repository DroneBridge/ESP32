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
#include "db_esp_now.h"
#include "globals.h"
#include "main.h"

#define TAG "DB_ESPNOW"

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint16_t s_example_espnow_seq[DB_ESPNOW_DATA_MAX] = {0, 0};
static QueueHandle_t s_db_espnow_queue;

/**
 * Parse received ESPNOW data.
 * @param data
 * @param data_len
 * @param state
 * @param seq
 * @param magic
 * @return
 */
int db_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic) {
    db_espnow_data_t *buf = (db_espnow_data_t *) data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(db_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *) buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/**
 * Prepare ESPNOW data to be sent.
 *
 * @param send_param
 */
void db_espnow_data_prepare(db_espnow_send_param_t *send_param) {
    db_espnow_data_t *buf = (db_espnow_data_t *) send_param->buffer;

    assert(send_param->len >= sizeof(db_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? DB_ESPNOW_DATA_BROADCAST : DB_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    esp_fill_random(buf->payload, send_param->len - sizeof(db_espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *) buf, send_param->len);
}

static void db_espnow_deinit(db_espnow_send_param_t *send_param) {
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_db_espnow_queue);
    esp_now_deinit();
}

void db_esp_now_send(uint8_t data[], uint32_t data_length) {
    // TODO: Encrypt with AES
    // TODO: Set peer addr based on mode
    esp_err_t esp_err = esp_now_send(peer_addr, data, data_length);
}

void db_esp_now_receive() {
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
    if (xQueueSend(s_db_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
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
    if (xQueueSend(s_db_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send to receive queue fail");
        free(recv_cb->data);
    }
}

/**
 * Called in the control loop task. Handles the more work intensive stuff. Takes the packets from the queue and processes
 * it accordingly.
 *
 * @param send_param
 */
void process_espnow_data(db_espnow_send_param_t *send_param) {
    db_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        db_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_db_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case DB_ESPNOW_SEND_CB: {
                // send next packet - the last sending callback has returned so we keep the order
                db_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                db_espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    db_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case DB_ESPNOW_RECV_CB:
            {
                db_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = db_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                if (ret == DB_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                }
                else if (ret == DB_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

esp_err_t db_espnow_init() {
    ESP_LOGI(TAG, "Initializing up ESP-NOW parameters");
    db_espnow_send_param_t *send_param;

    s_db_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(db_espnow_event_t));
    if (s_db_espnow_queue == NULL) {
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
        vSemaphoreDelete(s_db_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = DB_WIFI_CHANNEL;
    peer->ifidx = WIFI_IF_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(db_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_db_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(db_espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = DB_ESPNOW_SEND_DELAY;
    //TODO: MSP/LTM with their variable packet size are not supported - only transparent mode
    if (DB_TRANS_BUF_SIZE > 240) {
        send_param->len = 240;  // ESP-NOW max payload size is 250 bytes
    } else if(DB_TRANS_BUF_SIZE % 16 == 0) {
        send_param->len = DB_TRANS_BUF_SIZE;   // Because of AES encryption we can only accept multiples of 16.
    } else {
        send_param->len = 64;   // Default value in case someone fucked up the config
    }
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_db_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    db_espnow_data_prepare(send_param);

    return ESP_OK;
}