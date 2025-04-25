/***************************************************************************************************************************
 * @file db_ble.h
 * @brief DroneBridge ESP32 BLE Header file
 *
 * This file is part of DroneBridge and CosmicBridge
 *
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
 **************************************************************************************************************************/

#ifndef DB_BLE_H
#define DB_BLE_H

/***************************************************************************************************************************
 * Standard & System Headers
 **************************************************************************************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <stdint.h>

/***************************************************************************************************************************
 * MACROS
 **************************************************************************************************************************/
#define BLE_SVC_SPP_UUID16             0xABF0 ///< 16-bit UUID for the SPP (Serial Port Profile) service.
#define BLE_SVC_SPP_CHR_WRITE_UUID16   0xABF1 ///< 16-bit UUID for the SPP Receive service characteristic.
#define BLE_SVC_SPP_CHR_NOTIFY_UUID16  0xABF2 ///< 16-bit UUID for the SPP Send service characteristic.
#define BLE_SVC_SPP_CMD_WRITE_UUID16   0xABF3 ///< 16-bit UUID for the SPP Receive command characteristic.
#define BLE_SVC_SPP_CMD_NOTIFY_UUID16  0xABF4 ///< 16-bit UUID for the SPP Send command characteristic.
#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0180 ///< Generic tag appearance value for BLE GAP.
#define BLE_GAP_LE_ROLE_PERIPHERAL     0x00   ///< LE role value indicating a peripheral device.

/***************************************************************************************************************************
 * @brief Structure to hold BLE data.
 **************************************************************************************************************************/
typedef struct {
  uint16_t data_len; /**< Length of the data. */
  uint8_t *data;    /**< Pointer to the data. */
} __attribute__((__packed__)) db_ble_queue_event_t;

/***************************************************************************************************************************
 * Queue Handles
 **************************************************************************************************************************/

extern QueueHandle_t db_uart_write_queue_ble; /** Queue for data to be written to UART, filled by wireless
                                                 communication interface task */
extern QueueHandle_t
    db_uart_read_queue_ble; /** Queue for data to be written to wireless communication interface - used by BLE, filled by UART */

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/

/***************************************************************************************************************************
 * @brief Drone Bridge BLE initialisation function
 **************************************************************************************************************************/
void db_ble_init();

/***************************************************************************************************************************
 * @brief Initializes the global UART read and write queues.
 *
 * This function creates two FreeRTOS queues using xQueueCreate().
 * - `db_uart_write_queue_ble`: Used to store data that needs to be written to UART.
 * - `db_uart_read_queue_ble`: Used to store data read from UART for further processing.
 *
 * The queue size is set to 5, with each element being the size of `BleData_t`.
 * If queue creation fails, appropriate error messages are logged using `ESP_LOGI()`.
 *
 * @return void
 **************************************************************************************************************************/
void db_ble_queue_init();

/**
 * Reads RSSI from BLE connected device
 * @param rssi the var to store the rssi
 */
void db_ble_request_rssi(int8_t *rssi);

/*
 * Shut down BLE stack
 */
void db_ble_deinit();

#endif // DB_BLE_H