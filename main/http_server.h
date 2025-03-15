/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2021 Wolfgang Christl
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

#include <esp_err.h>

#ifndef DB_ESP32_HTTP_SERVER_NEW_H
#define DB_ESP32_HTTP_SERVER_NEW_H

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/

/***************************************************************************************************************************
 * @brief Starts the RESTful web server.
 *
 * This function initializes and starts an HTTP server with predefined URI handlers to serve system information, settings,
 * and client management APIs.
 *
 * @param base_path Base path for serving static files.
 * @return ESP_OK on successful server start, ESP_FAIL otherwise.
 **************************************************************************************************************************/
esp_err_t start_rest_server(const char *base_path);

#endif // DB_ESP32_HTTP_SERVER_NEW_H