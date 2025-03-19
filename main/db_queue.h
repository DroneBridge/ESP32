/***************************************************************************************************************************
 * @file db_queue.h
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

#ifndef DB_QUEUE_H
#define DB_QUEUE_H

#include <esp_wifi_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>

/***************************************************************************************************************************
 * @brief Structure to hold BLE data.
 **************************************************************************************************************************/
typedef struct {
  uint8_t data_len; /**< Length of the data. */
  uint8_t *data;    /**< Pointer to the data. */
} __attribute__((__packed__)) db_ble_queue_event_t;

extern QueueHandle_t db_uart_write_queue_global; // Global queue for data to be written to UART, filled by wireless
                                                 // communication interface task
extern QueueHandle_t
    db_uart_read_queue_global; // Global queue for data to be written to Wireless communication interface, filled by UART

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/
/***************************************************************************************************************************
 * @brief Initializes the global UART read and write queues.
 *
 * This function creates two FreeRTOS queues using xQueueCreate().
 * - `db_uart_write_queue_global`: Used to store data that needs to be written to UART.
 * - `db_uart_read_queue_global`: Used to store data read from UART for further processing.
 *
 * The queue size is set to 5, with each element being the size of `BleData_t`.
 * If queue creation fails, appropriate error messages are logged using `ESP_LOGI()`.
 *
 * @note Ensure FreeRTOS is initialized before calling this function.
 *
 * @return void
 **************************************************************************************************************************/
void db_queue_init();

#endif // DB_QUEUE_H