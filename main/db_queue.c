/***************************************************************************************************************************
 * @file db_queue.c
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

#include "db_queue.h"
#include "esp_log.h"

QueueHandle_t db_uart_write_queue_global;
QueueHandle_t db_uart_read_queue_global;

#define TAG "db_queue"
/***************************************************************************************************************************
 * Public Function Definition
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
 * @return void
 **************************************************************************************************************************/
void db_queue_init() {
  db_uart_write_queue_global = xQueueCreate(5, sizeof(BleData_t));
  db_uart_read_queue_global  = xQueueCreate(5, sizeof(BleData_t));

  if (db_uart_write_queue_global == NULL) {
    ESP_LOGI(TAG, "Failed to create queue, you are on your own, KABOOM!");
    return;
  }
  if (db_uart_read_queue_global == NULL) {
    ESP_LOGI(TAG, "Failed to create another queue, you are definitely on your own, KABOOM! KABOOM!");
    return;
  }
}