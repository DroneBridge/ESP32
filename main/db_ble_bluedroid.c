/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2025 Wolfgang Christl
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

/**
 * Implements Bluetooth Low-Energy using Bluedroid
 */

#include "db_ble.h"
#include <freertos/queue.h>
#include <stdint.h>
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include <esp_bt.h>
#include "esp_log.h"
#include "globals.h"

#define USE_ESPRESSIF_EXAMPLE_HEADER 1  // send fragmented data with an additional header
#define TAG "DB_BLUEDROID"
#define GATT_MTU_SIZE (512)

QueueHandle_t db_uart_write_queue_ble;
QueueHandle_t db_uart_read_queue_ble;

static char *DEVICE_NAME = "DroneBridge";
#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SPP_SVC_INST_ID                0

static const uint16_t spp_service_uuid = BLE_SVC_SERIAL_UUID16;

static const uint8_t spp_adv_data[23] = {
        /* Flags */
        0x02, 0x01, 0x06,
        /* Complete List of 16-bit Service Class UUIDs */
        0x03, 0x03, 0xF0, 0xAB,
        /* Complete Local Name in advertising */
        0x0F, 0x09, 'D', 'r', 'o', 'n', 'e', 'B', 'r', 'i', 'd', 'g', 'e', 'E', 'S', 'P'
};

static uint16_t ble_mtu_size = GATT_MTU_SIZE;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
// QueueHandle_t spp_uart_queue = NULL;
// static QueueHandle_t cmd_cmd_queue = NULL;

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
        .adv_int_min        = 0x20,
        .adv_int_max        = 0x40,
        .adv_type           = ADV_TYPE_IND,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .channel_map        = ADV_CHNL_ALL,
        .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node {
    int32_t len;
    uint8_t *node_buff;
    struct spp_receive_data_node *next_node;
} spp_receive_data_node_t;

static spp_receive_data_node_t *temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t *temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff {
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t *first_node;
} spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
        .node_num   = 0,
        .buff_size  = 0,
        .first_node = NULL
};

static void
gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile, one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
        [SPP_PROFILE_APP_IDX] = {
                .gatts_cb = gatts_profile_event_handler,
                .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t spp_data_notity_char_prop = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_INDICATE;


///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = BLE_SVC_SERIAL_CHR_WRITE_UUID16;
static const uint8_t spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = BLE_SVC_SERIAL_CHR_NOTIFY_UUID16;
static const uint8_t spp_data_notify_val[20] = {0x00};
static const uint8_t spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = BLE_SVC_SERIAL_CMD_WRITE_UUID16;
static const uint8_t spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = BLE_SVC_SERIAL_CMD_NOTIFY_UUID16;
static const uint8_t spp_status_val[10] = {0x00};
static const uint8_t spp_status_ccc[2] = {0x00, 0x00};


///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] = {
        //SPP -  Service Declaration
        [SPP_IDX_SVC]                        =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &primary_service_uuid, ESP_GATT_PERM_READ,
                         sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *) &spp_service_uuid}},

        //SPP -  data receive characteristic Declaration
        [SPP_IDX_SPP_DATA_RECV_CHAR]            =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) &char_prop_read_write}},

        //SPP -  data receive characteristic Value
        [SPP_IDX_SPP_DATA_RECV_VAL]                =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &spp_data_receive_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                         SPP_DATA_MAX_LEN, sizeof(spp_data_receive_val), (uint8_t *) spp_data_receive_val}},

        //SPP -  data notify characteristic Declaration
        [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) &spp_data_notity_char_prop}},

        //SPP -  data notify characteristic Value
        [SPP_IDX_SPP_DATA_NTY_VAL]   =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &spp_data_notify_uuid, ESP_GATT_PERM_READ,
                         SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *) spp_data_notify_val}},

        //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_DATA_NTF_CFG]         =
                {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
                        sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

        //SPP -  command characteristic Declaration
        [SPP_IDX_SPP_COMMAND_CHAR]            =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) &char_prop_read_write}},

        //SPP -  command characteristic Value
        [SPP_IDX_SPP_COMMAND_VAL]                 =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &spp_command_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                         SPP_CMD_MAX_LEN, sizeof(spp_command_val), (uint8_t *) spp_command_val}},

        //SPP -  status characteristic Declaration
        [SPP_IDX_SPP_STATUS_CHAR]            =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) &char_prop_read_notify}},

        //SPP -  status characteristic Value
        [SPP_IDX_SPP_STATUS_VAL]                 =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &spp_status_uuid, ESP_GATT_PERM_READ,
                         SPP_STATUS_MAX_LEN, sizeof(spp_status_val), (uint8_t *) spp_status_val}},

        //SPP -  status characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_STATUS_CFG]         =
                {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                         sizeof(uint16_t), sizeof(spp_status_ccc), (uint8_t *) spp_status_ccc}},
};

/**
 * Sends provided data to the UART Queue that will be processed by the control module
 *
 * @param data Buffer containing the data. Data will be copied from the buffer
 * @param data_len Length of the data
 */
void db_ble_send_to_uart(uint8_t *data, uint16_t data_len) {
    db_ble_queue_event_t bleData;            // Create a struct instance
    bleData.data = malloc(data_len);          // Allocate memory for the data buffer
    bleData.data_len = data_len;                  // Ensure len does not exceed buffer size

    if (bleData.data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for BLE data of length %d", data_len);
        return;
    }

    if (memcpy(bleData.data, data, data_len) != 0) {
        ESP_LOGE(TAG, "Failed to copy data to db_ble_queue_event_t");
        free(bleData.data);
        return;
    }
    ESP_LOGI(TAG, "Sending to UART");
    // Send the received data to the FreeRTOS queue
    if (xQueueSend(db_uart_write_queue_ble, &bleData, pdMS_TO_TICKS(50)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send BLE data to queue");
        free(bleData.data);
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "Advertising start successfully");
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "Advertising stop successfully");
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                     param->update_conn_params.status,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

static uint8_t find_char_and_desr_index(uint16_t handle) {
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB; i++) {
        if (handle == spp_handle_table[i]) {
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data) {
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *) malloc(sizeof(spp_receive_data_node_t));

    if (temp_spp_recv_data_node_p1 == NULL) {
        ESP_LOGI(TAG, "malloc error %s %d", __func__, __LINE__);
        return false;
    }
    if (temp_spp_recv_data_node_p2 != NULL) {
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *) malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    if (temp_spp_recv_data_node_p1->node_buff == NULL) {
        ESP_LOGI(TAG, "malloc error %s %d\n", __func__, __LINE__);
        temp_spp_recv_data_node_p1->len = 0;
    } else {
        memcpy(temp_spp_recv_data_node_p1->node_buff, p_data->write.value, p_data->write.len);
    }

    if (SppRecvDataBuff.node_num == 0) {
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    } else {
        SppRecvDataBuff.node_num++;
    }

    return true;
}


static void free_write_buffer(void) {
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL) {
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        if (temp_spp_recv_data_node_p1->node_buff) {
            free(temp_spp_recv_data_node_p1->node_buff);
        }
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer(void) {
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL) {
        db_ble_send_to_uart(temp_spp_recv_data_node_p1->node_buff, temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status,
                     param->reg.app_id, gatts_if);
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data_raw((uint8_t *) spp_adv_data, sizeof(spp_adv_data));
            esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "Characteristic read");
            break;
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(TAG, "Characteristic write, conn_id %d, handle %d", param->write.conn_id, param->write.handle);
            res = find_char_and_desr_index(p_data->write.handle);
            if (p_data->write.is_prep == false) {
                if (res == SPP_IDX_SPP_COMMAND_VAL) {
                    ESP_LOGW(TAG, "Got command, will ignore");
//                    uint8_t *spp_cmd_buff = NULL;
//                    spp_cmd_buff = (uint8_t *) malloc((ble_mtu_size - 3) * sizeof(uint8_t));
//                    if (spp_cmd_buff == NULL) {
//                        ESP_LOGE(TAG, "%s malloc failed", __func__);
//                        break;
//                    }
//                    memset(spp_cmd_buff, 0x0, (ble_mtu_size - 3));
//                    memcpy(spp_cmd_buff, p_data->write.value, p_data->write.len);
//                    xQueueSend(cmd_cmd_queue, &spp_cmd_buff, 10 / portTICK_PERIOD_MS);
                } else if (res == SPP_IDX_SPP_DATA_NTF_CFG) {
                    if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) &&
                        (p_data->write.value[1] == 0x00)) {
                        ESP_LOGI(TAG, "SPP data notification enable");
                        enable_data_ntf = true;
                    } else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x02) &&
                               (p_data->write.value[1] == 0x00)) {
                        ESP_LOGI(TAG, "SPP data indication enable");
                        enable_data_ntf = true;
                    } else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) &&
                               (p_data->write.value[1] == 0x00)) {
                        ESP_LOGI(TAG, "SPP data notification/indication disable");
                        enable_data_ntf = false;
                    }
                } else if (res == SPP_IDX_SPP_STATUS_CFG) {
                    if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) &&
                        (p_data->write.value[1] == 0x00)) {
                        ESP_LOGI(TAG, "SPP status notification enable");
                    } else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) &&
                               (p_data->write.value[1] == 0x00)) {
                        ESP_LOGI(TAG, "SPP status notification disable");
                    }
                } else if (res == SPP_IDX_SPP_DATA_RECV_VAL) {
                    db_ble_send_to_uart(p_data->write.value, p_data->write.len);
                } else {
                    //TODO:
                }
            } else if ((p_data->write.is_prep == true) && (res == SPP_IDX_SPP_DATA_RECV_VAL)) {
                ESP_LOGI(TAG, "Storing data");
                store_wr_buffer(p_data);
            }
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT: {
            ESP_LOGI(TAG, "Execute write");
            if (p_data->exec_write.exec_write_flag) {
                print_write_buffer();
                free_write_buffer();
            }
            break;
        }
        case ESP_GATTS_RESPONSE_EVT:
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "MTU exchange, MTU %d", param->mtu.mtu);
            ble_mtu_size = p_data->mtu.mtu;
            break;
        case ESP_GATTS_CONF_EVT:
            if (param->conf.status) {
                ESP_LOGI(TAG, "Confirm received, status %d, handle %d", param->conf.status, param->conf.handle);
            }
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "Service start, status %d, service_handle %d",
                     param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                     param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
            spp_conn_id = p_data->connect.conn_id;
            spp_gatts_if = gatts_if;
            is_connected = true;
            memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                     ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
            ble_mtu_size = 23;
            is_connected = false;
            enable_data_ntf = false;
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            ESP_LOGI(TAG, "The number handle %x", param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Create attribute table failed, error code 0x%x", param->add_attr_tab.status);
            } else if (param->add_attr_tab.num_handle != SPP_IDX_NB) {
                ESP_LOGE(TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, SPP_IDX_NB);
            } else {
                memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
                esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
            }
            break;
        }
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE ||
                /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

/**
 * Sends supplied buffer via BLE indication/notification to all connected clients
 * @param data Pointer to send buffer
 * @param data_len Length of send buffer
 * @param need_confirm True if a confirmation is required, which is a GATT indication; false if the confirmation is not required, which is a GATT notification.
 * @return 0 on success for all clients. -1 on failure. Gives esp_ble_gatts_send_indicate return value
 */
int db_ble_send_data(uint8_t *data, uint16_t data_len, uint8_t need_confirm) {
    int suc = esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], data_len, data, need_confirm);
    ESP_LOGI(TAG, "Sending BLE Data");
    if (suc != ESP_OK) {
        ESP_LOGE(TAG, "Error sending indicate esp_ble_gatts_send_indicate: %i", suc);
    }
    return suc;
}

/**
 * Used packet header format as of
 * https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/ble_spp_server/main/ble_spp_server_demo.c
 * Understood by Speedybee App and others
 *
 * @param data Pointer to send buffer
 * @param data_len Length of send buffer
 * @return 0 on success for all clients. -1 on failure. Gives esp_ble_gatts_send_indicate return value
 */
int send_ble_data_chunked_espressif(uint8_t *data, uint16_t data_len) {
    if (data_len <= (ble_mtu_size - 3)) {
        db_ble_send_data(data, data_len, true);
    } else {
        uint8_t total_num = 0;
        uint8_t current_num = 0;
        uint8_t *ntf_value_p = NULL;

        if ((data_len % (ble_mtu_size - 7)) == 0) {
            total_num = data_len / (ble_mtu_size - 7);
        } else {
            total_num = data_len / (ble_mtu_size - 7) + 1;
        }
        current_num = 1;
        ntf_value_p = (uint8_t *)malloc((ble_mtu_size-3)*sizeof(uint8_t));
        if (ntf_value_p == NULL) {
            ESP_LOGE(TAG, "%s malloc.2 failed", __func__);
            return -1;
        }

        while (current_num <= total_num) {
            if (current_num < total_num) {
                ntf_value_p[0] = '#';
                ntf_value_p[1] = '#';
                ntf_value_p[2] = total_num;
                ntf_value_p[3] = current_num;
                memcpy(ntf_value_p + 4, data + (current_num - 1)*(ble_mtu_size-7), (ble_mtu_size-7));
                db_ble_send_data(ntf_value_p, (ble_mtu_size-3), false);
            } else if(current_num == total_num) {
                ntf_value_p[0] = '#';
                ntf_value_p[1] = '#';
                ntf_value_p[2] = total_num;
                ntf_value_p[3] = current_num;
                memcpy(ntf_value_p + 4, data + (current_num - 1)*(ble_mtu_size-7), (data_len - (current_num - 1)*(ble_mtu_size - 7)));
                db_ble_send_data(ntf_value_p, (data_len - (current_num - 1)*(ble_mtu_size - 7) + 4), false);
            }
            vTaskDelay(20 / portTICK_PERIOD_MS);
            current_num++;
        }
        free(ntf_value_p);
    }
    return 0;
}

/**
 * @brief Sends BLE data, automatically splitting it into chunks if it exceeds the MTU limit.
 *
 * @param data Pointer to the data buffer to send.
 * @param data_len Length of the data in the buffer.
 * @return 0 on success, negative error code on failure during sending. Returns 1 if data_len is 0.
 */
int send_ble_data_chunked(uint8_t *data, uint16_t data_len) {
    // BLE requires 3 bytes overhead for notifications (ATT header)
    const uint16_t max_payload_size = ble_mtu_size > 3 ? ble_mtu_size - 3 : 0;
    int ret = 0;

    if (max_payload_size == 0) {
        ESP_LOGE(TAG, "Cannot send data, effective MTU is zero (ble_mtu_size: %d)", ble_mtu_size);
        return -1; // Indicate error: MTU too small
    }

    if (data == NULL || data_len == 0) {
        ESP_LOGW(TAG, "No data provided to send_ble_data_chunked.");
        return 1; // Indicate nothing was sent, but not necessarily an error state.
    }

    if (data_len <= max_payload_size) {
        // Data fits in a single packet
        ESP_LOGD(TAG, "Sending single packet, len: %d, max_payload: %d", data_len, max_payload_size);
        ret = db_ble_send_data(data, data_len, false);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to send single packet, error: %d", ret);
        }
    } else {
        // Data needs to be split into multiple packets
        ESP_LOGD(TAG, "Splitting data (len: %d) into chunks (max_payload: %d)", data_len, max_payload_size);
        uint16_t offset = 0;
        while (offset < data_len) {
            // Calculate the size of the current chunk
            uint16_t chunk_size = (data_len - offset > max_payload_size) ?
                                  max_payload_size :
                                  (data_len - offset);

            ESP_LOGD(TAG, "Sending chunk, offset: %d, chunk_size: %d", offset, chunk_size);

            // Send the current chunk (pointer arithmetic to get the chunk start)
            ret = db_ble_send_data(data + offset, chunk_size, false);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to send chunk at offset %d, error: %d", offset, ret);
                break; // Stop sending further chunks on error
            }

            // Move to the next chunk
            offset += chunk_size;
        }
    }
    return ret; // Return the status of the last send operation
}

/**
 * Main sending task for BLE data
 */
_Noreturn static void db_ble_server_uart_task() {
    DB_RADIO_IS_OFF = false; // we are about to send stuff - update this parameter to indicate to others
    ESP_LOGI(TAG, "BLE server UART_task started");
    int delay_timer_cnt = 0;
    db_ble_queue_event_t bleData;
    while (true) {
        // Waiting for UART event.
        if (xQueueReceive(db_uart_read_queue_ble, &bleData, 0)) {
            // Split into multiple packets if MTU is bigger than data to send
            // Instead, we could also change DB_PARAM_SERIAL_PACK_SIZE dynamically based on the BLE MTU, but that can
            // result in very small buffers for the UART task to read which may cause performance issues.
            // The serial packets arrive here with the length of DB_PARAM_SERIAL_PACK_SIZE. Fragment again if necessary.
            if(!enable_data_ntf){
                ESP_LOGE(TAG, "%s do not enable data Notify", __func__);
                continue;
            }
            if (USE_ESPRESSIF_EXAMPLE_HEADER) {
                send_ble_data_chunked_espressif(bleData.data, bleData.data_len);
            } else {
                send_ble_data_chunked(bleData.data, bleData.data_len);
            }
            free(bleData.data);
        }
        if (delay_timer_cnt == 5000) {
            /* all actions are non-blocking so allow some delay so that the IDLE task of FreeRTOS and the watchdog can run
            read: https://esp32developer.com/programming-in-c-c/tasks/tasks-vs-co-routines for reference */
            db_ble_request_rssi(&db_esp_signal_quality.air_rssi); // use this timer to get the RSSI updated
            vTaskDelay(10 / portTICK_PERIOD_MS);
            delay_timer_cnt = 0;
        } else {
            delay_timer_cnt++;
        }
    }
    vTaskDelete(NULL);
}

/**
 * Gets the RSSI of the connection
 * @param rssi Pointer to the RSSI variable to be filled. The value is in dBm (I suspect)
 */
void db_ble_request_rssi(int8_t *rssi) {
//    if (active_conn_handle != BLE_HS_CONN_HANDLE_NONE && !DB_RADIO_IS_OFF) {
//        int rc = ble_gap_conn_rssi(active_conn_handle, rssi);
//        if (rc != 0) {
//            ESP_LOGE(TAG, "Failed to read BLE RSSI; rc=%d", rc);
//        }
//    }
}

void db_ble_init() {
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    /* Start BLE Notify Task*/
    xTaskCreate(db_ble_server_uart_task, /**< Task Function */
                "BLETask",                 /**< Task Name */
                4096,                    /**< Stack size */
                NULL,                    /**< Parameters, NULL as no parameters required */
                5,                       /**< Task Priority */
                NULL                     /**< Task reference, used to change behaviour of task from another task */
    );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "%s init bluetooth", __func__);

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(GATT_MTU_SIZE);
    if (local_mtu_ret) {
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }


}

void db_ble_deinit() {

}

void db_ble_queue_init() {
    db_uart_write_queue_ble = xQueueCreate(5, sizeof(db_ble_queue_event_t));
    db_uart_read_queue_ble = xQueueCreate(5, sizeof(db_ble_queue_event_t));

    if (db_uart_write_queue_ble == NULL) {
        ESP_LOGI(TAG, "Failed to create queue, you are on your own, KABOOM!");
        return;
    }
    if (db_uart_read_queue_ble == NULL) {
        ESP_LOGI(TAG, "Failed to create another queue, you are definitely on your own, KABOOM! KABOOM!");
        return;
    }
}