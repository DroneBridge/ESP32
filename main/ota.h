#ifndef OTA_H
#define OTA_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_log.h"

void ota_update_task(void *pvParameter);

#endif // OTA_H
