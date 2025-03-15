/**************************************************************************
 * @file db_ble.c
 * @brief DroneBridge ESP32 BLE Source File
 *
 * This file is part of DroneBridge and CosmicBridge
 *
 * This file contains the NimBLE Initialisation functions to start the BLE Host
 * stack and start advertising the BLE Service
 * @author Witty-Wizard <agarwalshashank429@gmail.com>
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
 *************************************************************************/

/**************************************************************************
 * Header Inclusion
 *************************************************************************/
#include "db_ble.h"
#include "globals.h"

/**************************************************************************
 * ESP-IDF APIs
 *************************************************************************/
#include "esp_check.h"

/**************************************************************************
 * NimBLE Host APIs
 *************************************************************************/
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/**************************************************************************
 * MACROS
 *************************************************************************/
#define TAG "DB_BLE_MSGS"

/**************************************************************************
 * Private Variables
 *************************************************************************/
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
static char *DEVICE_NAME = "DroneBridge";
static uint16_t ble_spp_svc_gatt_read_val_handle;

/**************************************************************************
 * Library Function Declaration
 *************************************************************************/

/**
 * @brief Library function, the function is defined in source file and not the
 * header file for the linker to work this function needs to be defined.
 */
void ble_store_config_init();

/**************************************************************************
 * Private Function Declaration
 *************************************************************************/

static int gap_event_handler(struct ble_gap_event *event, void *arg);

/**
 * @brief Starts the BLE advertising process.
 */
static void start_advertising(void);

/**
 * @brief Format Address to string
 *
 * @warning Does not check for pointer overflow, make sure to allocate enough
 * memory to the character array.
 *
 * Takes address stored in integer array and
 * convert to character array of the form xx:xx:xx:xx:xx:xx.
 *
 * @param [out] addr_str pointer to the output character array
 */
inline static void format_addr(char *addr_str, uint8_t addr[]);

/**
 * @brief FreeRTOS task to run NimBLE port
 *
 * @param [in] params Input parameters to the FreeRTOS Task
 */
static void nimble_host_task(void *params);

/**
 * @brief Function to initialize NimBLE host config
 */
static void nimble_host_config_init();

/**
 * @brief Function to initialize gatt service
 */
static int gatt_svr_init();

/**
 * @brief Function to initialize advertising
 */
static void adv_init(void);

/**************************************************************************
 * Callback Function Declaration
 *************************************************************************/

/**
 * @brief Callback Function called when the BLE host stack is reset
 */
static void on_stack_reset(int reason);

/**
 * @brief Callback Function called when the BLE host stack synchronises
 */

static void on_stack_sync(void);

/**
 * @brief GATT service register callback function
 */
static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt,
                                 void *arg);

/**
 * @brief Callback function to handle gap events
 */
static int gap_event_handler(struct ble_gap_event *event, void *arg);

/**
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
 */
static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

/**************************************************************************
 * BLE GATT service table
 *************************************************************************/
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        /*** Service: SPP */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    /* Support SPP service */
                    .uuid       = BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16),
                    .access_cb  = ble_svc_gatt_handler,
                    .val_handle = &ble_spp_svc_gatt_read_val_handle,
                    .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                             BLE_GATT_CHR_F_NOTIFY,
                },
                {
                    0, /* No more characteristics */
                }},
    },
    {
        0, /* No more services. */
    },
};

/**************************************************************************
 * Private Function Definition
 *************************************************************************/

static void db_ble_server_uart_task() {
  MODLOG_DFLT(INFO, "BLE server UART_task started\n");
  int rc = 0;
  BleData_t bleData;
  while (true) {
    // Waiting for UART event.
    if (xQueueReceive(db_uart_read_queue_global, &bleData, pdMS_TO_TICKS(10))) {
      for (int i = 0; i < CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        /* Check if client has subscribed to notifications */
        if (conn_handle_subs[i]) {
          struct os_mbuf *txom;

          // Convert BleData_t struct data into an os_mbuf buffer
          txom = ble_hs_mbuf_from_flat(bleData.data, bleData.length);
          if (!txom) {
            MODLOG_DFLT(ERROR, "Failed to allocate os_mbuf");
            return;
          }

          // Send BLE notification
          rc = ble_gatts_notify_custom(i, ble_spp_svc_gatt_read_val_handle, txom);
          if (rc != 0) {
            MODLOG_DFLT(ERROR, "Error sending BLE notification rc = %d", rc);
          }
        }
      }
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
    MODLOG_DFLT(INFO, "connection %s; status=%d ",
                event->connect.status == 0 ? "established" : "failed",
                event->connect.status);
    if (event->connect.status == 0) {
      rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
      assert(rc == 0);
      // print_conn_desc(&desc);
    }
    MODLOG_DFLT(INFO, "\n");
    if (event->connect.status != 0 || CONFIG_BT_NIMBLE_MAX_CONNECTIONS > 1) {
      /* Connection failed or if multiple connection allowed; resume
       * advertising. */
      start_advertising();
    }
    break;

  case BLE_GAP_EVENT_DISCONNECT:
    MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
    // print_conn_desc(&event->disconnect.conn);
    MODLOG_DFLT(INFO, "\n");

    conn_handle_subs[event->disconnect.conn.conn_handle] = false;

    /* Connection terminated; resume advertising. */
    start_advertising();
    break;

  case BLE_GAP_EVENT_CONN_UPDATE:
    /* The central has updated the connection parameters. */
    MODLOG_DFLT(INFO, "connection updated; status=%d ",
                event->conn_update.status);
    rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
    assert(rc == 0);
    // print_conn_desc(&desc);
    MODLOG_DFLT(INFO, "\n");
    break;

  case BLE_GAP_EVENT_ADV_COMPLETE:
    MODLOG_DFLT(INFO, "advertise complete; reason=%d",
                event->adv_complete.reason);
    start_advertising();
    break;

  case BLE_GAP_EVENT_MTU:
    MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                event->mtu.conn_handle, event->mtu.channel_id,
                event->mtu.value);
    break;

  case BLE_GAP_EVENT_SUBSCRIBE:
    MODLOG_DFLT(INFO,
                "subscribe event; conn_handle=%d attr_handle=%d "
                "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                event->subscribe.conn_handle, event->subscribe.attr_handle,
                event->subscribe.reason, event->subscribe.prev_notify,
                event->subscribe.cur_notify, event->subscribe.prev_indicate,
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
  fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;

  name                    = ble_svc_gap_device_name();
  fields.name             = (uint8_t *)name;
  fields.name_len         = strlen(name);
  fields.name_is_complete = 1;

  fields.uuids16             = (ble_uuid16_t[]){BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
  fields.num_uuids16         = 1;
  fields.uuids16_is_complete = 1;

  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
    return;
  }

  /* Begin advertising. */
  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  rc                   = ble_gap_adv_start(own_addr_type,     // Type of address that stack should use
                                           NULL,              // Peer address for direct advertising, null for in-direct advertising
                                           BLE_HS_FOREVER,    // Duration of advertisement, BLE_HS_FOREVER for no expiration
                                           &adv_params,       // Additional arguments for advertismenet
                                           gap_event_handler, // Gap event handler callback
                                           NULL);             // Callback arguments
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
    return;
  }
}

inline static void format_addr(char *addr_str, uint8_t addr[]) {
  sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
  switch (ctxt->op) {
  case BLE_GATT_ACCESS_OP_READ_CHR:
    MODLOG_DFLT(INFO, "Callback for read");
    break;

  case BLE_GATT_ACCESS_OP_WRITE_CHR:
    MODLOG_DFLT(INFO, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
    struct os_mbuf *om = ctxt->om;               // Get the received data buffer
    int len            = OS_MBUF_PKTLEN(om);     // Get total length of received data
    BleData_t bleData;                           // Create a struct instance
    bleData.length = len % sizeof(bleData.data); // Ensure len does not exceed buffer size

    os_mbuf_copydata(om, 0, bleData.length, bleData.data); // Copy data to buffer
    bleData.data[bleData.length] = '\0';                   // Null-terminate for printing as a string

    // Send the received data to the FreeRTOS queue
    if (xQueueSend(db_uart_write_queue_global, &bleData, portMAX_DELAY) != pdPASS) {
      MODLOG_DFLT(ERROR, "Failed to send BLE data to queue");
    }
    break;

  default:
    MODLOG_DFLT(INFO, "\nDefault Callback");
    break;
  }
  return 0;
}

static void adv_init(void) {
  /* Local variables */
  int rc            = 0;
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
    ESP_LOGE(TAG, "failed to set device name to %s, error code: %d",
             DEVICE_NAME, rc);
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

static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt,
                                 void *arg) {
  char buf[BLE_UUID_STR_LEN];

  switch (ctxt->op) {
  case BLE_GATT_REGISTER_OP_SVC:
    MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                ctxt->svc.handle);
    break;

  case BLE_GATT_REGISTER_OP_CHR:
    MODLOG_DFLT(DEBUG,
                "registering characteristic %s with "
                "def_handle=%d val_handle=%d\n",
                ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                ctxt->chr.def_handle, ctxt->chr.val_handle);
    break;

  case BLE_GATT_REGISTER_OP_DSC:
    MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
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
  ble_hs_cfg.reset_cb          = on_stack_reset;
  ble_hs_cfg.sync_cb           = on_stack_sync;
  ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
  ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

  /* WittyWizard: Figure out what they do later*/
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

/**************************************************************************
 * Public Function Definition
 *************************************************************************/

void db_init_ble() {
  ESP_LOGI("BLE", "Initializing BLE stack");

  /* NimBLE host stack initialization */
  ESP_ERROR_CHECK(nimble_port_init());

  /* GAP service initialization */
  ESP_ERROR_CHECK(gap_init());

  /* GAP service initialization */
  ESP_ERROR_CHECK(gatt_svr_init());

  /* NimBLE host configuration initialization */
  nimble_host_config_init();

  /* Start NimBLE Notify Task*/
  xTaskCreate(
      db_ble_server_uart_task, // Task Function
      "uTask",                 // Task Name
      4096,                    // Stack size
      NULL,                    // Parameters, NULL as no parameters required
      8,                       // Task Priority
      NULL                     // Task reference, used to change behaviour of task from another task
  );

  /* Start NimBLE host task thread and return */
  nimble_port_freertos_init(nimble_host_task);
  return;
}
