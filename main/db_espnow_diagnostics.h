#pragma once

#include <sdkconfig.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stddef.h>

#define CONFIG_DB_ESPNOW_DIAGNOSTICS 1


/** Sources of serial output, independent of the task performing the write. */
typedef enum { DB_DIAG_GND, DB_DIAG_PEER, DB_DIAG_LOCAL, DB_DIAG_SOURCE_COUNT } db_diag_source_t;
/** Queue submission classes; RX classes share one physical UART queue. */
typedef enum { DB_DIAG_RX_GND, DB_DIAG_RX_PEER, DB_DIAG_TX, DB_DIAG_CALLBACK,
               DB_DIAG_QUEUE_COUNT } db_diag_queue_t;

#ifdef CONFIG_DB_ESPNOW_DIAGNOSTICS
#include "esp_timer.h"
extern QueueHandle_t db_diag_uart_events;
/** Start measurement and the reporting/event-draining task after serial initialization. */
void db_diag_start(void);
/** Record a serial write, including unsuccessful or partial writes. */
void db_diag_write(db_diag_source_t source, size_t requested, int written, int64_t start);
/** Record the interval between consecutive serial read attempts. */
void db_diag_read(void);
/** Count a complete peer radio-status frame withheld from the local FC. */
void db_diag_peer_radio_status_filtered(void);
/** Submit with the original timeout while measuring latency and queue occupancy. */
BaseType_t db_diag_queue_send(db_diag_queue_t kind, QueueHandle_t queue,
                              const void *item, TickType_t timeout);
#define DB_DIAG_NOW() esp_timer_get_time()
#else
#define DB_DIAG_NOW() 0
#define db_diag_start() ((void)0)
#define db_diag_write(source, requested, written, start) ((void)0)
#define db_diag_read() ((void)0)
#define db_diag_peer_radio_status_filtered() ((void)0)
#define db_diag_queue_send(kind, queue, item, timeout) xQueueSend(queue, item, timeout)
#endif
