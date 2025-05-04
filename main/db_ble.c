#include <sys/cdefs.h>
/***************************************************************************************************************************
 * @file db_ble.c
 * @brief DroneBridge ESP32 BLE Source File
 *
 * This file is part of DroneBridge and CosmicBridge
 *
 * This file contains the NimBLE Initialisation functions to start the BLE Host
 * stack and start advertising the BLE Service
 * @authors Witty-Wizard <agarwalshashank429@gmail.com> further modified by Wolfgang Christl
 * @license Apache License, Version 2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 * https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************************************************************/

/***************************************************************************************************************************
 * Header Inclusion
 **************************************************************************************************************************/
#include "db_ble.h"

/***************************************************************************************************************************
 * ESP-IDF APIs
 **************************************************************************************************************************/
#include "esp_check.h"

/***************************************************************************************************************************
 * NimBLE Host APIs
 **************************************************************************************************************************/
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "globals.h"

/***************************************************************************************************************************
 * MACROS
 **************************************************************************************************************************/
#define TAG "DB_BLE"

/***************************************************************************************************************************
 * Public Variables
 **************************************************************************************************************************/
QueueHandle_t db_uart_write_queue_ble;
QueueHandle_t db_uart_read_queue_ble;

/***************************************************************************************************************************
 * Private Variables
 **************************************************************************************************************************/
static uint16_t active_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
static char *DEVICE_NAME = "DroneBridge";
static uint16_t ble_spp_svc_gatt_notify_val_handle;
static uint16_t ble_spp_svc_gatt_write_val_handle;
static uint16_t ble_spp_svc_gatt_notify_cmd_handle;
static uint16_t ble_spp_svc_gatt_write_cmd_handle;
static uint16_t ble_mtu_size = 23;

/***************************************************************************************************************************
 * Library Function Declaration
 **************************************************************************************************************************/

/***************************************************************************************************************************
 * @brief Library function, the function is defined in source file and not the
 * header file for the linker to work this function needs to be defined.
 */
void ble_store_config_init();

/***************************************************************************************************************************
 * Private Function Declaration
 **************************************************************************************************************************/

static int gap_event_handler(struct ble_gap_event *event, void *arg);

/***************************************************************************************************************************
 * @brief Starts the BLE advertising process.
 **************************************************************************************************************************/
static void start_advertising(void);

/***************************************************************************************************************************
 * @brief Format Address to string
 *
 * @warning Does not check for pointer overflow, make sure to allocate enough
 * memory to the character array.
 *
 * Takes address stored in integer array and
 * convert to character array of the form xx:xx:xx:xx:xx:xx.
 *
 * @param [out] addr_str pointer to the output character array
 **************************************************************************************************************************/
inline static void format_addr(char *addr_str, uint8_t addr[]);

/***************************************************************************************************************************
 * @brief FreeRTOS task to run NimBLE port
 *
 * @param [in] params Input parameters to the FreeRTOS Task
 **************************************************************************************************************************/
static void nimble_host_task(void *params);

/***************************************************************************************************************************
 * @brief Function to initialize NimBLE host config
 **************************************************************************************************************************/
static void nimble_host_config_init();

/***************************************************************************************************************************
 * @brief Function to initialize gatt service
 **************************************************************************************************************************/
static int gatt_svr_init();

/***************************************************************************************************************************
 * @brief Function to initialize advertising
 **************************************************************************************************************************/
static void adv_init(void);

/***************************************************************************************************************************
 * Callback Function Declaration
 **************************************************************************************************************************/

/***************************************************************************************************************************
 * @brief Callback Function called when the BLE host stack is reset
 **************************************************************************************************************************/
static void on_stack_reset(int reason);

/***************************************************************************************************************************
 * @brief Callback Function called when the BLE host stack synchronises
 **************************************************************************************************************************/

static void on_stack_sync(void);

/***************************************************************************************************************************
 * @brief GATT service register callback function
 **************************************************************************************************************************/
static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

/***************************************************************************************************************************
 * @brief Callback function to handle gap events
 **************************************************************************************************************************/
static int gap_event_handler(struct ble_gap_event *event, void *arg);

/***************************************************************************************************************************
 * @brief Handler for BLE GATT service.
 *
 * This Callback function handles BLE GATT service events.
 *
 * @param conn_handle The connection handle.
 * @param attr_handle The attribute handle.
 * @param ctxt The GATT access context.
 * @param arg Additional arguments.
 *
 * @return Status code indicating the result of the handler.
 **************************************************************************************************************************/
static int
ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

/***************************************************************************************************************************
 * BLE GATT service table
 **************************************************************************************************************************/
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
        {
                /*** Service: Serial Interface */
                .type = BLE_GATT_SVC_TYPE_PRIMARY,
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_SERIAL_UUID16),
                .characteristics =
                (struct ble_gatt_chr_def[]) {
                        {
                                /* Write-only characteristic */
                                .uuid       = BLE_UUID16_DECLARE(BLE_SVC_SERIAL_CHR_WRITE_UUID16),
                                .access_cb  = ble_svc_gatt_handler,
                                .val_handle = &ble_spp_svc_gatt_write_val_handle,
                                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                        },
                        {
                                /* Notify-only characteristic */
                                .uuid       = BLE_UUID16_DECLARE(BLE_SVC_SERIAL_CHR_NOTIFY_UUID16),
                                .access_cb  = ble_svc_gatt_handler, // No direct access required, only notify
                                .val_handle = &ble_spp_svc_gatt_notify_val_handle,
                                .flags      = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
                        },
                        {
                                /* Write-only characteristic */
                                .uuid       = BLE_UUID16_DECLARE(BLE_SVC_SPP_CMD_WRITE_UUID16),
                                .access_cb  = ble_svc_gatt_handler, // No direct access required, only notify
                                .val_handle = &ble_spp_svc_gatt_write_cmd_handle,
                                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                        },
                        {
                                /* Notify-only characteristic */
                                .uuid       = BLE_UUID16_DECLARE(BLE_SVC_SPP_CMD_NOTIFY_UUID16),
                                .access_cb  = ble_svc_gatt_handler, // No direct access required, only notify
                                .val_handle = &ble_spp_svc_gatt_notify_cmd_handle,
                                .flags      = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
                        },
                        {0}, /* No more characteristics */
                },
        },
        {0}, /* No more services. */
};

/***************************************************************************************************************************
 * Private Function Definition
 **************************************************************************************************************************/

/**
 * Sends supplied buffer via BLE notification to all connected clients
 * @param data Pointer to send buffer
 * @param data_len Length of send buffer
 * @return 0 on success for all clients. -1 on failure to allocate buffer or ble_gatts_notify_custom return value
 */
 int db_ble_send_data(const uint8_t *data, uint16_t data_len) {
    for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        /* Check if the client has subscribed to notifications */
        if (conn_handle_subs[i]) {

            /* Write to the characteristics */
            int rc = ble_gattc_write_flat(i,ble_spp_svc_gatt_notify_val_handle,
                data,     /* Data pointer */
                data_len, /* Data length */
                NULL,     /* Callback function */
                NULL      /* Callback function arguments */
            );
            if (rc != 0) {
                ESP_LOGE(TAG, "Failed to write characteristic value (rc = %d)", rc);
                return rc;
            }

            /* Now send a notification with the updated characteristic value */
            rc = ble_gatts_notify(i, ble_spp_svc_gatt_notify_val_handle);
            if (rc != 0) {
                ESP_LOGE(TAG, "Error sending BLE notification rc = %d", rc);
                return rc;
            }
        }
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
int send_ble_data_chunked(const uint8_t *data, uint16_t data_len) {
    // NimBLE requires 3 bytes overhead for notifications (ATT header)
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
        ret = db_ble_send_data(data, data_len);
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
            ret = db_ble_send_data(data + offset, chunk_size);
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
            send_ble_data_chunked(bleData.data, bleData.data_len);
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

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    struct ble_gap_conn_desc desc;
    int rc = 0;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            /* A new connection was established or a connection attempt failed. */
            ESP_LOGI(TAG, "connection %s; status=%d", event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);
            if (event->connect.status == 0) {
                active_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connected, conn_handle=%d", active_conn_handle);
                rc = ble_gap_conn_find(active_conn_handle, &desc);
                assert(rc == 0);
                // print_conn_desc(&desc);
            }
            ESP_LOGI(TAG, "\n");
            if (event->connect.status != 0 || CONFIG_BT_NIMBLE_MAX_CONNECTIONS > 1) {
                /* Connection failed or if multiple connection allowed; resume
                 * advertising. */
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);
            conn_handle_subs[event->disconnect.conn.conn_handle] = false;
            // Mark connection handle as invalid
            active_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            /* Connection terminated; resume advertising. */
            start_advertising();
            break;

        case BLE_GAP_EVENT_CONN_UPDATE:
            /* The central has updated the connection parameters. */
            ESP_LOGI(TAG, "connection updated; status=%d ", event->conn_update.status);
            rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
            assert(rc == 0);
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "advertise complete; reason=%d", event->adv_complete.reason);
            start_advertising();
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d", event->mtu.conn_handle,
                     event->mtu.channel_id,
                     event->mtu.value);
            ble_mtu_size = event->mtu.value;    // ToDo: Support more than one MTU for all connections
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d reason=%d prevn=%d curn=%d previ=%d curi=%d",
                     event->subscribe.conn_handle, event->subscribe.attr_handle, event->subscribe.reason,
                     event->subscribe.prev_notify, event->subscribe.cur_notify, event->subscribe.prev_indicate,
                     event->subscribe.cur_indicate);
            conn_handle_subs[event->subscribe.conn_handle] = true;
            break;

        default:
            break;
    }
    return rc;
}

static void start_advertising(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *) name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]) {BLE_UUID16_INIT(BLE_SVC_SERIAL_UUID16)};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, // Type of address that stack should use
                           NULL, // Peer address for direct advertising, null for in-direct advertising
                           BLE_HS_FOREVER,    // Duration of advertisement, BLE_HS_FOREVER for no expiration
                           &adv_params,       // Additional arguments for advertismenet
                           gap_event_handler, // Gap event handler callback
                           NULL);             // Callback arguments
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

static int
ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGI(TAG, "Callback for read");
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            ESP_LOGD(TAG, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
            struct os_mbuf *om = ctxt->om;           // Get the received data buffer
            int len = OS_MBUF_PKTLEN(om); // Get total length of received data
            db_ble_queue_event_t bleData;            // Create a struct instance
            bleData.data = malloc(len);          // Allocate memory for the data buffer
            bleData.data_len = len;                  // Ensure len does not exceed buffer size

            if (bleData.data == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for BLE data of length %d", len);
                return BLE_ATT_ERR_UNLIKELY;
            }

            if (os_mbuf_copydata(om, 0, bleData.data_len, bleData.data) != 0) {
                ESP_LOGE(TAG, "Failed to copy data from os_mbuf");
                free(bleData.data);
                return BLE_ATT_ERR_UNLIKELY;
            }

            // Send the received data to the FreeRTOS queue
            if (xQueueSend(db_uart_write_queue_ble, &bleData, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send BLE data to queue");
                free(bleData.data);
            }
            break;

        default:
            ESP_LOGI(TAG, "Default Callback");
            break;
    }
    return 0;
}

static void adv_init(void) {
    /* Local variables */
    int rc = 0;
    char addr_str[18] = {0};

    /* Make sure we have proper BT identity address set */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }

    /* Figure out BT address to use while advertising */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    /* Copy device address to addr_val */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "device address: %s", addr_str);

    /* Start advertising. */
    start_advertising();
}

int gap_init() {
    /* Local Variable to store the error code */
    int rc = 0;

    /* Initialize GAP Server*/
    ble_svc_gap_init();

    /* Set GAP Device name*/
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);

    /* check for if any error occured */
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device name to %s, error code: %d", DEVICE_NAME, rc);
        return rc;
    }

    /* set the ble gap appearance tag */
    rc = ble_svc_gap_device_appearance_set(BLE_GAP_APPEARANCE_GENERIC_TAG);

    /* check for error and return */
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device appearance, error code: %d", rc);
        return rc;
    }

    // no errors return
    return rc;
}

static int gatt_svr_init() {
    int rc = 0;
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            ESP_LOGD(TAG, "registered service %s with handle=%d\n", ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                        ctxt->svc.handle);
            break;

        case BLE_GATT_REGISTER_OP_CHR:
            ESP_LOGD(TAG, "registering characteristic %s with def_handle=%d val_handle=%d\n",
                        ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf), ctxt->chr.def_handle, ctxt->chr.val_handle);
            break;

        case BLE_GATT_REGISTER_OP_DSC:
            ESP_LOGD(TAG, "registering descriptor %s with handle=%d\n",
                        ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                        ctxt->dsc.handle);
            break;

        default:
            assert(0);
            break;
    }
}

static void nimble_host_config_init() {
    // Set Host callbacks
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_sc = 1;   // enable secure connection

    /** WittyWizard:
     * TO DO: Figure out what they do later, code works without them, but what they do I have no idea
     * */

    //   ble_hs_cfg.sm_io_cap = CONFIG_EXAMPLE_IO_TYPE;
    // #ifdef CONFIG_EXAMPLE_BONDING
    //   ble_hs_cfg.sm_bonding = 1;
    // #endif
    // #ifdef CONFIG_EXAMPLE_MITM
    //   ble_hs_cfg.sm_mitm = 1;
    // #endif
    // #ifdef CONFIG_EXAMPLE_USE_SC
    //   ble_hs_cfg.sm_sc = 1;
    // #else
    //   ble_hs_cfg.sm_sc = 0;
    // #endif
    // #ifdef CONFIG_EXAMPLE_BONDING
    //   ble_hs_cfg.sm_our_key_dist = 1;
    //   ble_hs_cfg.sm_their_key_dist = 1;
    // #endif

    // Store host configuration
    ble_store_config_init();
}

static void on_stack_reset(int reason) {
    // On reset, print reset reason to console
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    // Initialize advertising
    adv_init();
}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

/***************************************************************************************************************************
 * Public Function Definition
 **************************************************************************************************************************/

/**
 * Gets the RSSI of the connection
 * @param rssi Pointer to the RSSI variable to be filled. The value is in dBm (I suspect)
 */
void db_ble_request_rssi(int8_t *rssi) {
    if (active_conn_handle != BLE_HS_CONN_HANDLE_NONE && !DB_RADIO_IS_OFF) {
        int rc = ble_gap_conn_rssi(active_conn_handle, rssi);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to read BLE RSSI; rc=%d", rc);
        }
    }
}

void db_ble_init() {
    ESP_LOGI(TAG, "Initializing BLE stack");

    /* NimBLE host stack initialization */
    ESP_ERROR_CHECK(nimble_port_init());

    /* GAP service initialization */
    ESP_ERROR_CHECK(gap_init());

    /* GAP service initialization */
    ESP_ERROR_CHECK(gatt_svr_init());

    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    /* Start NimBLE Notify Task*/
    xTaskCreate(db_ble_server_uart_task, /**< Task Function */
                "BLETask",                 /**< Task Name */
                4096,                    /**< Stack size */
                NULL,                    /**< Parameters, NULL as no parameters required */
                5,                       /**< Task Priority */
                NULL                     /**< Task reference, used to change behaviour of task from another task */
    );

    /* Start NimBLE host task thread and return */
    nimble_port_freertos_init(nimble_host_task);
}

void db_ble_deinit() {
    ESP_LOGI(TAG, "Deinitializing BLE stack");
    if (ble_gap_adv_stop() != 0) {
        ESP_LOGE(TAG, "Failed to stop advertising");
    }
    int rc = nimble_port_stop();
    if (rc == 0) {
        nimble_port_deinit();
    } else {
        ESP_LOGI(TAG, "Nimble port stop failed, rc = %d", rc);
    }
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
