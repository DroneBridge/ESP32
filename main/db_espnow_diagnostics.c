#include "db_espnow_diagnostics.h"

#ifdef CONFIG_DB_ESPNOW_DIAGNOSTICS
#include <inttypes.h>
#include <string.h>
#include "driver/uart.h"
#include "soc/uart_channel.h"
#include "esp_log.h"
#include "db_serial.h"
#include "db_parameters.h"
#include "globals.h"

QueueHandle_t db_diag_uart_events;
static portMUX_TYPE stats_lock = portMUX_INITIALIZER_UNLOCKED;
static bool active;
static int64_t last_read;

/** Timing accumulator for one reporting interval. */
typedef struct {
    uint32_t count;
    uint64_t total_us;
    uint64_t max_us;
    uint64_t bytes;
    uint32_t failures;
    UBaseType_t peak;
} db_diag_metric_t;

/** Shared counters copied and reset atomically by the reporter. */
typedef struct {
    db_diag_metric_t writes[DB_DIAG_SOURCE_COUNT];
    db_diag_metric_t queues[DB_DIAG_QUEUE_COUNT];
    uint64_t read_gap_us;
    uint32_t peer_radio_status_filtered;
} db_diag_stats_t;
static db_diag_stats_t stats;

/** Count suppressed link-local reports without logging in the forwarding task. */
void db_diag_peer_radio_status_filtered(void) {
    portENTER_CRITICAL(&stats_lock);
    if (active) stats.peer_radio_status_filtered++;
    portEXIT_CRITICAL(&stats_lock);
}

/** Add elapsed time to an accumulator while its caller holds stats_lock. */
static void accumulate(db_diag_metric_t *metric, uint64_t elapsed) {
    metric->count++;
    metric->total_us += elapsed;
    if (elapsed > metric->max_us) metric->max_us = elapsed;
}

/** Record write latency and actual accepted bytes without logging in the data task. */
void db_diag_write(db_diag_source_t source, size_t requested, int written, int64_t start) {
    const int64_t end = esp_timer_get_time();
    portENTER_CRITICAL(&stats_lock);
    if (active) {
        db_diag_metric_t *metric = &stats.writes[source];
        accumulate(metric, end - start);
        metric->bytes += written > 0 ? (unsigned)written : 0;
        if (written < 0 || (size_t)written != requested) metric->failures++;
    }
    portEXIT_CRITICAL(&stats_lock);
}

/** Measure read service gaps, retaining the previous timestamp across reporting intervals. */
void db_diag_read(void) {
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&stats_lock);
    if (active) {
        if (last_read && (uint64_t)(now - last_read) > stats.read_gap_us)
            stats.read_gap_us = now - last_read;
        last_read = now;
    }
    portEXIT_CRITICAL(&stats_lock);
}

/** Preserve queue behavior and measure the entire call, including scheduler delays. */
BaseType_t db_diag_queue_send(db_diag_queue_t kind, QueueHandle_t queue,
                              const void *item, TickType_t timeout) {
    const UBaseType_t before = uxQueueMessagesWaiting(queue);
    const int64_t start = esp_timer_get_time();
    const BaseType_t result = xQueueSend(queue, item, timeout);
    const uint64_t elapsed = esp_timer_get_time() - start;
    const UBaseType_t after = uxQueueMessagesWaiting(queue);
    const UBaseType_t peak = before > after ? before : after;
    portENTER_CRITICAL(&stats_lock);
    if (active) {
        db_diag_metric_t *metric = &stats.queues[kind];
        accumulate(metric, elapsed);
        if (result != pdTRUE) metric->failures++;
        if (peak > metric->peak) metric->peak = peak;
    }
    portEXIT_CRITICAL(&stats_lock);
    return result;
}

/** Drain UART notifications without reading, flushing, or resetting UART data. */
static void drain_events(uint32_t *buffer_full, uint32_t *fifo_overflow) {
    uart_event_t event;
    if (db_diag_uart_events == NULL) return;
    for (unsigned i = 0; i < 10 && xQueueReceive(db_diag_uart_events, &event, 0) == pdTRUE; i++) {
        if (event.type == UART_BUFFER_FULL) (*buffer_full)++;
        if (event.type == UART_FIFO_OVF) (*fifo_overflow)++;
    }
}

/** Report snapshots outside the lock; poll UART events between one-second reports. */
static void diagnostic_task(void *arg) {
    (void)arg;
    uint64_t queue_failures[DB_DIAG_QUEUE_COUNT] = {0};
    uint64_t full_total = 0, overflow_total = 0;
    uint64_t filtered_total = 0;
    uint32_t full = 0, overflow = 0;
    int64_t previous = esp_timer_get_time();
    const char *sources[] = {"gnd", "peer", "local"};
    const char *queues[] = {"rx_gnd", "rx_peer", "tx", "callback"};
    for (;;) {
        drain_events(&full, &overflow);
        const int64_t now = esp_timer_get_time();
        if (now - previous >= 1000000) {
            db_diag_stats_t snapshot;
            portENTER_CRITICAL(&stats_lock);
            /* Include a read stall still in progress, not just completed gaps. */
            if (last_read && now > last_read && (uint64_t)(now - last_read) > stats.read_gap_us)
                stats.read_gap_us = now - last_read;
            snapshot = stats;
            memset(&stats, 0, sizeof(stats));
            portEXIT_CRITICAL(&stats_lock);
            full_total += full;
            overflow_total += overflow;
            filtered_total += snapshot.peer_radio_status_filtered;
            ESP_LOGI("DB_DIAG", "window_ms=%" PRIi64 " read_gap_us=%" PRIu64
                     " rx_full=%" PRIu32 "/%" PRIu64 " fifo_ovf=%" PRIu32 "/%" PRIu64
                     " peer_radio_filtered=%" PRIu32 "/%" PRIu64 " events=%s",
                     (now - previous) / 1000, snapshot.read_gap_us,
                     full, full_total, overflow, overflow_total,
                     snapshot.peer_radio_status_filtered, filtered_total,
                     db_diag_uart_events ? "uart" : "unavailable");
            for (unsigned i = 0; i < DB_DIAG_SOURCE_COUNT; i++) {
                const db_diag_metric_t *m = &snapshot.writes[i];
                ESP_LOGI("DB_DIAG", "write_%s n=%" PRIu32 " bytes=%" PRIu64
                         " total_us=%" PRIu64 " avg_us=%" PRIu64 " max_us=%" PRIu64 " short=%" PRIu32,
                         sources[i], m->count, m->bytes, m->total_us,
                         m->count ? m->total_us / m->count : 0, m->max_us, m->failures);
            }
            for (unsigned i = 0; i < DB_DIAG_QUEUE_COUNT; i++) {
                const db_diag_metric_t *m = &snapshot.queues[i];
                queue_failures[i] += m->failures;
                ESP_LOGI("DB_DIAG", "q_%s n=%" PRIu32 " peak_sample=%u total_us=%" PRIu64
                         " max_us=%" PRIu64 " fail=%" PRIu32 "/%" PRIu64,
                         queues[i], m->count, (unsigned)m->peak, m->total_us, m->max_us,
                         m->failures, queue_failures[i]);
            }
            previous = now;
            full = overflow = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10) ? pdMS_TO_TICKS(10) : 1);
    }
}

/** Enable only for ESP-NOW with a physically separate UART debug console. */
void db_diag_start(void) {
    if (DB_PARAM_RADIO_MODE != DB_WIFI_MODE_ESPNOW_AIR &&
        DB_PARAM_RADIO_MODE != DB_WIFI_MODE_ESPNOW_GND) return;
#ifndef CONFIG_DB_SERIAL_OPTION_JTAG
#ifdef CONFIG_ESP_CONSOLE_UART_CUSTOM
    const int console_tx = CONFIG_ESP_CONSOLE_UART_TX_GPIO >= 0 ?
                           CONFIG_ESP_CONSOLE_UART_TX_GPIO : UART_NUM_0_TXD_DIRECT_GPIO_NUM;
    const int console_rx = CONFIG_ESP_CONSOLE_UART_RX_GPIO >= 0 ?
                           CONFIG_ESP_CONSOLE_UART_RX_GPIO : UART_NUM_0_RXD_DIRECT_GPIO_NUM;
#else
    const int console_tx = UART_NUM_0_TXD_DIRECT_GPIO_NUM;
    const int console_rx = UART_NUM_0_RXD_DIRECT_GPIO_NUM;
#endif
    /* Suppress diagnostics entirely if the console could corrupt MAVLink. */
    if (CONFIG_ESP_CONSOLE_UART_NUM == UART_NUM ||
        DB_PARAM_GPIO_TX == console_tx || DB_PARAM_GPIO_RX == console_tx ||
        DB_PARAM_GPIO_TX == console_rx || DB_PARAM_GPIO_RX == console_rx) return;
#endif
    portENTER_CRITICAL(&stats_lock);
    active = true;
    portEXIT_CRITICAL(&stats_lock);
    if (xTaskCreate(diagnostic_task, "espnow_diag", 4096, NULL, 2, NULL) != pdPASS) {
        portENTER_CRITICAL(&stats_lock);
        active = false;
        portEXIT_CRITICAL(&stats_lock);
        ESP_LOGE("DB_DIAG", "Cannot start diagnostic task");
    }
}
#endif
