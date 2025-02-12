#include "ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"

static const char *TAG = "OTA";

void ota_update_task(void *pvParameter) {
    char *ota_write_data = (char *)pvParameter;
    esp_err_t err;
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Starting OTA update");

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    err = esp_ota_write(update_handle, (const void *)ota_write_data, strlen(ota_write_data));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
        esp_ota_end(update_handle);
        vTaskDelete(NULL);
        return;
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (%s)", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "OTA update successful, restarting...");
    esp_restart();
}
