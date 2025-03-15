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

/***************************************************************************************************************************
 * Standard C Library Headers
 **************************************************************************************************************************/
#include <fcntl.h>
#include <string.h>

/***************************************************************************************************************************
 * ESP-IDF System Headers
 **************************************************************************************************************************/
#include <esp_chip_info.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_vfs.h>

/***************************************************************************************************************************
 * LWIP (Lightweight IP) Headers
 **************************************************************************************************************************/
#include <lwip/sockets.h>

/***************************************************************************************************************************
 * HTTP Server & JSON Handling Headers
 **************************************************************************************************************************/
#include "cJSON.h"
#include "esp_http_server.h"
#include "http_server.h"

/***************************************************************************************************************************
 * Other Project-Specific Headers
 **************************************************************************************************************************/
#include "db_serial.h"
#include "globals.h"
#include "main.h"

static const char *REST_TAG = "DB_HTTP_REST";
#define REST_CHECK(a, str, goto_tag, ...)                                                                                   \
  do {                                                                                                                      \
    if (!(a)) {                                                                                                             \
      ESP_LOGE(REST_TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__);                                            \
      goto goto_tag;                                                                                                        \
    }                                                                                                                       \
  } while (0)

#define FILE_PATH_MAX   (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct rest_server_context {
  char base_path[ESP_VFS_PATH_MAX + 1];
  char scratch[SCRATCH_BUFSIZE];
} rest_server_context_t;

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

bool is_valid_ip4(char *ipAddress) {
  struct sockaddr_in sa;
  int result = inet_pton(AF_INET, ipAddress, &(sa.sin_addr));
  return result != 0;
}

/***************************************************************************************************************************
 * Private Function Declaration
 **************************************************************************************************************************/
/***************************************************************************************************************************
 * @brief Set HTTP response content type according to file extension
 **************************************************************************************************************************/
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath);

/***************************************************************************************************************************
 * @brief Send HTTP response with the contents of the requested file
 **************************************************************************************************************************/
static esp_err_t rest_common_get_handler(httpd_req_t *req);

/***************************************************************************************************************************
 * Process incoming settings change and reply with HTTP success message
 * @param req
 * @return ESP error code
 **************************************************************************************************************************/
static esp_err_t settings_post_handler(httpd_req_t *req);

/***************************************************************************************************************************
 * Process incoming UDP connection add request. Only one IPv4 connection can be added at a time
 * Expecting JSON in the form of:
 * {
 *   "ip": "XXX.XXX.XXX.XXX",
 *   "port": 452
 * }
 * @param req
 * @return
 **************************************************************************************************************************/
static esp_err_t settings_clients_udp_post(httpd_req_t *req);

/***************************************************************************************************************************
 * Process a request that shall clear all active UDP connections.
 * ESP32 will remove all maintained UDP connections from its internal list and UDP clients will have to register again.
 * This also clears the one UDP client connection that gets saved to NVM.
 *
 * @param req
 * @return ESP_OK if all was good. Else ESP_FAIL
 **************************************************************************************************************************/
static esp_err_t settings_clients_clear_udp_get(httpd_req_t *req);

/***************************************************************************************************************************
 * Sets the static IP of the ESP32 when in Wi-Fi client mode
 * Expecting JSON in the form of:
 * {
 *   "client_ip": "XXX.XXX.XXX.XXX",
 *   "netmask": "XXX.XXX.XXX.XXX",
 *   "gw_ip": "XXX.XXX.XXX.XXX"
 * }
 *
 * To reset static IP (dyn. IP) send JSON with empty strings:
 * {
 *   "client_ip": "",
 *   "netmask": "",
 *   "gw_ip": ""
 * }
 * @param req
 * @return
 **************************************************************************************************************************/
static esp_err_t settings_static_ip_post_handler(httpd_req_t *req);

/***************************************************************************************************************************
 * Returns build information esp-idf version and build version
 * @param req
 * @return
 **************************************************************************************************************************/
static esp_err_t system_info_get_handler(httpd_req_t *req);

/***************************************************************************************************************************
 * Returns a JSON containing read bytes from UART, number of connected TCP connections and UDP broadcasts as well as the
 * current IP address of the ESP32
 * @param req
 * @return ESP_OK on successfully sending the http request
 **************************************************************************************************************************/
static esp_err_t system_stats_get_handler(httpd_req_t *req);

/***************************************************************************************************************************
 * Returns a JSON containing all active UDP connections that the ESP32 sends to
 * @param req
 * @return ESP_OK on successfully sending the http request
 **************************************************************************************************************************/
static esp_err_t system_clients_get_handler(httpd_req_t *req);

/***************************************************************************************************************************
 * Respond with all internally known settings
 * @param req
 * @return ESP_OK on successfully sending the http request
 **************************************************************************************************************************/
static esp_err_t settings_get_handler(httpd_req_t *req);

/***************************************************************************************************************************
 * Private Function Definitions
 **************************************************************************************************************************/

static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath) {
  const char *type = "text/plain";
  if (CHECK_FILE_EXTENSION(filepath, ".html")) {
    type = "text/html";
  } else if (CHECK_FILE_EXTENSION(filepath, ".js")) {
    type = "application/javascript";
  } else if (CHECK_FILE_EXTENSION(filepath, ".css")) {
    type = "text/css";
  } else if (CHECK_FILE_EXTENSION(filepath, ".png")) {
    type = "image/png";
  } else if (CHECK_FILE_EXTENSION(filepath, ".ico")) {
    type = "image/x-icon";
  } else if (CHECK_FILE_EXTENSION(filepath, ".svg")) {
    type = "text/xml";
  }
  return httpd_resp_set_type(req, type);
}

static esp_err_t rest_common_get_handler(httpd_req_t *req) {
  char filepath[FILE_PATH_MAX];

  rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
  strlcpy(filepath, rest_context->base_path, sizeof(filepath));
  if (req->uri[strlen(req->uri) - 1] == '/') {
    strlcat(filepath, "/index.html", sizeof(filepath));
  } else {
    strlcat(filepath, req->uri, sizeof(filepath));
  }
  int fd = open(filepath, O_RDONLY, 0);
  if (fd == -1) {
    ESP_LOGE(REST_TAG, "Failed to open file : %s", filepath);
    /* Respond with 500 Internal Server Error */
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
    return ESP_FAIL;
  }

  set_content_type_from_file(req, filepath);

  char *chunk = rest_context->scratch;
  ssize_t read_bytes;
  do {
    /* Read file in chunks into the scratch buffer */
    read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
    if (read_bytes == -1) {
      ESP_LOGE(REST_TAG, "Failed to read file : %s", filepath);
    } else if (read_bytes > 0) {
      /* Send the buffer contents as HTTP response chunk */
      if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK) {
        close(fd);
        ESP_LOGE(REST_TAG, "File sending failed!");
        /* Abort sending file */
        httpd_resp_sendstr_chunk(req, NULL);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
        return ESP_FAIL;
      }
    }
  } while (read_bytes > 0);
  /* Close file after sending complete */
  close(fd);
  ESP_LOGI(REST_TAG, "File sending complete");
  /* Respond with an empty chunk to signal HTTP response completion */
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t settings_post_handler(httpd_req_t *req) {
  int total_len = req->content_len;
  int cur_len   = 0;
  char *buf     = ((rest_server_context_t *)(req->user_ctx))->scratch;
  int received  = 0;
  if (total_len >= SCRATCH_BUFSIZE) {
    // This should be HTTPD_414_PAYLOAD_TOO_LARGE, but that's not
    // implemented yet, so use 400 Bad Request instead.
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "content too long");
    return ESP_FAIL;
  }
  while (cur_len < total_len) {
    received = httpd_req_recv(req, buf + cur_len, total_len);
    if (received <= 0) {
      /* Respond with 500 Internal Server Error */
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
      return ESP_FAIL;
    }
    cur_len += received;
  }
  buf[total_len] = '\0';

  cJSON *root = cJSON_Parse(buf);

  cJSON *json = cJSON_GetObjectItem(root, "esp32_mode");
  if (json)
    DB_RADIO_MODE_DESIGNATED =
        json->valueint; // do not directly change DB_WIFI_MODE since it is not safe and constantly processed by other tasks.
                        // Save settings and reboot will assign DB_RADIO_MODE_DESIGNATED to DB_WIFI_MODE

  json = cJSON_GetObjectItem(root, "wifi_ssid");
  if (json && strlen(json->valuestring) < 32 && strlen(json->valuestring) > 0)
    strncpy((char *)DB_WIFI_SSID, json->valuestring, sizeof(DB_WIFI_SSID) - 1);
  else if (json)
    ESP_LOGE(REST_TAG, "Invalid SSID length (1-31)");

  json = cJSON_GetObjectItem(root, "wifi_pass");
  if (json && strlen(json->valuestring) < 64 && strlen(json->valuestring) > 7)
    strncpy((char *)DB_WIFI_PWD, json->valuestring, sizeof(DB_WIFI_PWD) - 1);
  else if (json)
    ESP_LOGE(REST_TAG, "Invalid password length (8-63)");

  json = cJSON_GetObjectItem(root, "ap_channel");
  if (json && json->valueint > 0 && json->valueint < 14) {
    DB_WIFI_CHANNEL = json->valueint;
  } else if (json) {
    ESP_LOGE(REST_TAG, "No a valid wifi channel (1-13). Not changing!");
  }

  json = cJSON_GetObjectItem(root, "wifi_en_gn");
  if (json && (json->valueint == 1 || json->valueint == 0)) {
    DB_WIFI_EN_GN = json->valueint;
  } else if (json) {
    ESP_LOGE(REST_TAG, "wifi_en_gn is not 1 (802.11bgn) nor 0 (802.11b) for WiFi client mode");
  }

  json = cJSON_GetObjectItem(root, "ant_use_ext");
  if (json && (json->valueint == 1 || json->valueint == 0)) {
    DB_EN_EXT_ANT = json->valueint;
  } else if (json) {
    ESP_LOGW(REST_TAG, "ant_use_ext is not 1 (external antenna) nor 0 (on-board antenna)");
    DB_EN_EXT_ANT = false;
  }

  json = cJSON_GetObjectItem(root, "trans_pack_size");
  if (json && json->valueint > 0 && json->valueint < 1024) {
    DB_TRANS_BUF_SIZE = json->valueint;
  } else {
    ESP_LOGE(REST_TAG, "Settings change: trans_pack_size is out of range (1-1024): %i", json->valueint);
  }

  json = cJSON_GetObjectItem(root, "serial_timeout");
  if (json && json->valueint > 0) {
    DB_SERIAL_READ_TIMEOUT_MS = json->valueint;
  } else {
    ESP_LOGE(REST_TAG, "Serial read timeout must be >0 - ignoring");
  }

  json = cJSON_GetObjectItem(root, "tx_pin");
  if (json && json->valueint <= SOC_GPIO_IN_RANGE_MAX) {
    DB_UART_PIN_TX = json->valueint;
  } else {
    ESP_LOGW(REST_TAG, "GPIO pin must be in range of %i", SOC_GPIO_IN_RANGE_MAX);
  }
  json = cJSON_GetObjectItem(root, "rx_pin");
  if (json && json->valueint <= SOC_GPIO_IN_RANGE_MAX) {
    DB_UART_PIN_RX = json->valueint;
  } else {
    ESP_LOGW(REST_TAG, "GPIO pin must be in range of %i", SOC_GPIO_IN_RANGE_MAX);
  }
  json = cJSON_GetObjectItem(root, "rts_pin");
  if (json && json->valueint <= SOC_GPIO_IN_RANGE_MAX) {
    DB_UART_PIN_RTS = json->valueint;
  } else {
    ESP_LOGW(REST_TAG, "GPIO pin must be in range of %i", SOC_GPIO_IN_RANGE_MAX);
  }
  json = cJSON_GetObjectItem(root, "cts_pin");
  if (json && json->valueint <= SOC_GPIO_IN_RANGE_MAX) {
    DB_UART_PIN_CTS = json->valueint;
  } else {
    ESP_LOGW(REST_TAG, "GPIO pin must be in range of %i", SOC_GPIO_IN_RANGE_MAX);
  }

  json = cJSON_GetObjectItem(root, "rts_thresh");
  if (json)
    DB_UART_RTS_THRESH = json->valueint;

  json = cJSON_GetObjectItem(root, "baud");
  if (json)
    DB_UART_BAUD_RATE = json->valueint;

  json = cJSON_GetObjectItem(root, "telem_proto");
  if (json && (json->valueint == 1 || json->valueint == 4 || json->valueint == 5)) {
    DB_SERIAL_PROTOCOL = json->valueint;
  } else if (json) {
    ESP_LOGW(REST_TAG, "telem_proto is not 1 (LTM/MSP) or 4 (MAVLink) or 5 (Transparent). Changing to transparent");
    DB_SERIAL_PROTOCOL = DB_SERIAL_PROTOCOL_TRANSPARENT;
  }

  json = cJSON_GetObjectItem(root, "radio_dis_onarm");
  if (json && (json->valueint == 1 || json->valueint == 0)) {
    DB_DISABLE_RADIO_ARMED = json->valueint;
  } else if (json) {
    ESP_LOGW(REST_TAG, "radio_dis_onarm is not 1 (turn WiFi off on arm) or 0 (keep WiFi on)");
    DB_DISABLE_RADIO_ARMED = false;
  }

  json = cJSON_GetObjectItem(root, "ltm_pp");
  if (json)
    DB_LTM_FRAME_NUM_BUFFER = json->valueint;

  json = cJSON_GetObjectItem(root, "ap_ip");
  if (json && is_valid_ip4(json->valuestring)) {
    strncpy(DEFAULT_AP_IP, json->valuestring, sizeof(DEFAULT_AP_IP) - 1);
  } else if (json) {
    ESP_LOGE(REST_TAG, "New IP \"%s\" is not a valid IP address! Not changing!", json->valuestring);
  }
  db_write_settings_to_nvs();
  ESP_LOGI(REST_TAG, "Settings changed!");
  cJSON_Delete(root);
  httpd_resp_sendstr(req, "{\n"
                          "    \"status\": \"success\",\n"
                          "    \"msg\": \"Settings changed! Rebooting ...\"\n"
                          "  }");
  vTaskDelay(1000 / portTICK_PERIOD_MS); // wait to allow the website displaying the success message
  // Send reboot message
  httpd_resp_set_type(req, "application/json");
  cJSON *reboot_info = cJSON_CreateObject();
  cJSON_AddStringToObject(reboot_info, "msg", "Rebooting!");
  const char *sys_info = cJSON_Print(reboot_info);
  httpd_resp_sendstr(req, sys_info);
  free((void *)sys_info);
  cJSON_Delete(reboot_info);
  vTaskDelay(500 / portTICK_PERIOD_MS); // wait 1s to allow the website displaying the success message
  esp_restart();
  return ESP_OK;
}

static esp_err_t settings_clients_udp_post(httpd_req_t *req) {
  int total_len = req->content_len;
  int cur_len   = 0;
  char *buf     = ((rest_server_context_t *)(req->user_ctx))->scratch;
  int received  = 0;
  if (total_len >= SCRATCH_BUFSIZE) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "content too long");
    return ESP_FAIL;
  }
  while (cur_len < total_len) {
    received = httpd_req_recv(req, buf + cur_len, total_len);
    if (received <= 0) {
      /* Respond with 500 Internal Server Error */
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
      return ESP_FAIL;
    }
    cur_len += received;
  }
  buf[total_len] = '\0';

  // Obtain & process JSON from request
  cJSON *root = cJSON_Parse(buf);

  int new_udp_port = 0;
  char new_ip[IP4ADDR_STRLEN_MAX];
  uint8_t save_to_nvm = false;
  cJSON *json         = cJSON_GetObjectItem(root, "ip");
  if (json)
    strncpy(new_ip, json->valuestring, sizeof(new_ip));
  new_ip[IP4ADDR_STRLEN_MAX - 1] = '\0'; // to remove warning and to be sure
  json                           = cJSON_GetObjectItem(root, "port");
  if (json)
    new_udp_port = json->valueint;
  json = cJSON_GetObjectItem(root, "save");
  if (json && cJSON_IsBool(json)) {
    if (cJSON_IsTrue(json)) {
      save_to_nvm = true;
    } else {
      save_to_nvm = false;
    }
  } else {
  }

  // populate the UDP connections list with a new connection
  struct sockaddr_in new_sockaddr;
  memset(&new_sockaddr, 0, sizeof(new_sockaddr));
  new_sockaddr.sin_family = AF_INET;
  inet_pton(AF_INET, new_ip, &new_sockaddr.sin_addr);
  new_sockaddr.sin_port                 = htons(new_udp_port);
  struct db_udp_client_t new_udp_client = {
      .udp_client = new_sockaddr, .mac = {0, 0, 0, 0, 0, 0} // dummy MAC
  };
  // udp_conn_list is initialized as the very first thing during startup - we expect it to be there
  bool success = add_to_known_udp_clients(udp_conn_list, new_udp_client, save_to_nvm);

  // Clean up
  cJSON_Delete(root);
  if (success) {
    httpd_resp_sendstr(req, "{\n"
                            "    \"status\": \"success\",\n"
                            "    \"msg\": \"Added UDP connection!\"\n"
                            "  }");
  } else {
    httpd_resp_sendstr(req, "{\n"
                            "    \"status\": \"failed\",\n"
                            "    \"msg\": \"Failed to add UDP connection!\"\n"
                            "  }");
  }

  return ESP_OK;
}

static esp_err_t settings_clients_clear_udp_get(httpd_req_t *req) {
  for (int i = 0; i < udp_conn_list->size; ++i) {
    memset(&udp_conn_list->db_udp_clients[i], 0, sizeof(struct db_udp_client_t));
  }
  udp_conn_list->size = 0;
  ESP_LOGI(REST_TAG, "Removed all UDP clients from list!");
  // Clear saved client as well. Pass any client since it will be ignored as long as clear_client is set to true.
  save_udp_client_to_nvm(&udp_conn_list->db_udp_clients[0], true);
  return ESP_OK;
}

static esp_err_t settings_static_ip_post_handler(httpd_req_t *req) {
  int total_len = req->content_len;
  int cur_len   = 0;
  char *buf     = ((rest_server_context_t *)(req->user_ctx))->scratch;
  int received  = 0;
  if (total_len >= SCRATCH_BUFSIZE) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "content too long");
    return ESP_FAIL;
  }
  while (cur_len < total_len) {
    received = httpd_req_recv(req, buf + cur_len, total_len);
    if (received <= 0) {
      /* Respond with 500 Internal Server Error */
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
      return ESP_FAIL;
    }
    cur_len += received;
  }
  buf[total_len] = '\0';
  // Obtain & process JSON from request
  cJSON *root = cJSON_Parse(buf);

  esp_err_t err = ESP_OK;
  cJSON *json   = cJSON_GetObjectItem(root, "client_ip");
  if (json) {
    strncpy(DB_STATIC_STA_IP, json->valuestring, sizeof(DB_STATIC_STA_IP));
    DB_STATIC_STA_IP[IP4ADDR_STRLEN_MAX - 1] = '\0'; // to remove warning and to be sure
  } else {
    err = ESP_FAIL;
  }

  json = cJSON_GetObjectItem(root, "netmask");
  if (json) {
    strncpy(DB_STATIC_STA_IP_NETMASK, json->valuestring, sizeof(DB_STATIC_STA_IP_NETMASK));
    DB_STATIC_STA_IP_NETMASK[IP4ADDR_STRLEN_MAX - 1] = '\0'; // to remove warning and to be sure
  } else {
    err = ESP_FAIL;
  }

  json = cJSON_GetObjectItem(root, "gw_ip");
  if (json) {
    strncpy(DB_STATIC_STA_IP_GW, json->valuestring, sizeof(DB_STATIC_STA_IP_GW));
    DB_STATIC_STA_IP_GW[IP4ADDR_STRLEN_MAX - 1] = '\0'; // to remove warning and to be sure
  } else {
    err = ESP_FAIL;
  }

  // Clean up
  cJSON_Delete(root);
  db_write_settings_to_nvs();

  if (err == ESP_OK) {
    httpd_resp_sendstr(req, "{\n"
                            "    \"status\": \"success\",\n"
                            "    \"msg\": \"Updated static IP when in Wi-Fi client mode!\"\n"
                            "  }");
  } else {
    httpd_resp_sendstr(req, "{\n"
                            "    \"status\": \"failed\",\n"
                            "    \"msg\": \"Failed to update static IP when in Wi-Fi client mode\"\n"
                            "  }");
  }
  return ESP_OK;
}

static esp_err_t system_info_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  cJSON *root = cJSON_CreateObject();
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  cJSON_AddStringToObject(root, "idf_version", IDF_VER);
  cJSON_AddNumberToObject(root, "db_build_version", DB_BUILD_VERSION);
  cJSON_AddNumberToObject(root, "major_version", DB_MAJOR_VERSION);
  cJSON_AddNumberToObject(root, "minor_version", DB_MINOR_VERSION);
  cJSON_AddNumberToObject(root, "patch_version", DB_PATCH_VERSION);
  cJSON_AddStringToObject(root, "maturity_version", DB_MATURITY_VERSION);
  cJSON_AddNumberToObject(root, "esp_chip_model", chip_info.model);
  cJSON_AddNumberToObject(root, "has_rf_switch", DB_HAS_RF_SWITCH);
  char mac_str[18];
  sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", LOCAL_MAC_ADDRESS[0], LOCAL_MAC_ADDRESS[1], LOCAL_MAC_ADDRESS[2],
          LOCAL_MAC_ADDRESS[3], LOCAL_MAC_ADDRESS[4], LOCAL_MAC_ADDRESS[5]);
  cJSON_AddStringToObject(root, "esp_mac", mac_str);
#ifdef CONFIG_DB_SERIAL_OPTION_JTAG
  cJSON_AddNumberToObject(root, "serial_via_JTAG", 1);
#else
  cJSON_AddNumberToObject(root, "serial_via_JTAG", 0);
#endif
  const char *sys_info = cJSON_Print(root);
  httpd_resp_sendstr(req, sys_info);
  free((void *)sys_info);
  cJSON_Delete(root);
  return ESP_OK;
}

static esp_err_t system_stats_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  cJSON *root = cJSON_CreateObject();
  cJSON_AddNumberToObject(root, "read_bytes", serial_total_byte_count);
  cJSON_AddNumberToObject(root, "tcp_connected", num_connected_tcp_clients);
  cJSON_AddNumberToObject(root, "udp_connected", udp_conn_list->size);
  // add IP:PORT info on connected UDP clients
  cJSON *udp_clients = cJSON_CreateArray();
  for (int i = 0; i < udp_conn_list->size; i++) {
    char ip_string[INET_ADDRSTRLEN];
    char ip_port_string[INET_ADDRSTRLEN + 10];
    inet_ntop(AF_INET, &(udp_conn_list->db_udp_clients[i].udp_client.sin_addr), ip_string, INET_ADDRSTRLEN);
    sprintf(ip_port_string, "%s:%d", ip_string, htons(udp_conn_list->db_udp_clients[i].udp_client.sin_port));
    cJSON_AddItemToArray(udp_clients, cJSON_CreateString(ip_port_string));
  }
  cJSON_AddItemToObject(root, "udp_clients", udp_clients);
  // add RSSI and IP info
  if (DB_RADIO_MODE == DB_WIFI_MODE_STA) {
    cJSON_AddStringToObject(root, "current_client_ip", CURRENT_CLIENT_IP);
    cJSON_AddNumberToObject(root, "esp_rssi", db_esp_signal_quality.air_rssi);
  } else if (DB_RADIO_MODE == DB_WIFI_MODE_AP || DB_RADIO_MODE == DB_WIFI_MODE_AP_LR) {
    cJSON *sta_array = cJSON_AddArrayToObject(root, "connected_sta");
    for (int i = 0; i < wifi_sta_list.num; i++) {
      cJSON *connected_stations_status = cJSON_CreateObject();
      char mac_str[18];
      sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", wifi_sta_list.sta[i].mac[0], wifi_sta_list.sta[i].mac[1],
              wifi_sta_list.sta[i].mac[2], wifi_sta_list.sta[i].mac[3], wifi_sta_list.sta[i].mac[4],
              wifi_sta_list.sta[i].mac[5]);
      cJSON_AddStringToObject(connected_stations_status, "sta_mac", mac_str);
      cJSON_AddNumberToObject(connected_stations_status, "sta_rssi", wifi_sta_list.sta[i].rssi);
      cJSON_AddItemToArray(sta_array, connected_stations_status);
    }
  } else {
    // other modes like ESP-NOW do not activate HTTP server so do nothing
  }
  const char *sys_info = cJSON_Print(root);
  httpd_resp_sendstr(req, sys_info);
  free((void *)sys_info);
  cJSON_Delete(root);
  return ESP_OK;
}

static esp_err_t system_clients_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  cJSON *root = cJSON_CreateObject();
  //    cJSON *tcp_clients = cJSON_CreateArray();
  //    for (int i = 0; i < udp_conn_list->size; i++) {
  //        //TODO: Save the TCP IPs and port during accept and put them here into a JSON
  //    }
  //    cJSON_AddItemToObject(root, "tcp_clients", tcp_clients);

  cJSON *udp_clients = cJSON_CreateArray();
  for (int i = 0; i < udp_conn_list->size; i++) {
    char ip_string[INET_ADDRSTRLEN];
    char ip_port_string[INET_ADDRSTRLEN + 10];
    inet_ntop(AF_INET, &(udp_conn_list->db_udp_clients[i].udp_client.sin_addr), ip_string, INET_ADDRSTRLEN);
    sprintf(ip_port_string, "%s:%d", ip_string, htons(udp_conn_list->db_udp_clients[i].udp_client.sin_port));
    cJSON_AddItemToArray(udp_clients, cJSON_CreateString(ip_port_string));
  }
  cJSON_AddItemToObject(root, "udp_clients", udp_clients);

  const char *sys_info = cJSON_Print(root);
  httpd_resp_sendstr(req, sys_info);
  free((void *)sys_info);
  cJSON_Delete(root);
  return ESP_OK;
}

static esp_err_t settings_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  cJSON *root = cJSON_CreateObject();
  cJSON_AddNumberToObject(root, "esp32_mode", DB_RADIO_MODE);
  cJSON_AddStringToObject(root, "wifi_ssid", (char *)DB_WIFI_SSID);
  cJSON_AddStringToObject(root, "wifi_pass", (char *)DB_WIFI_PWD);
  cJSON_AddNumberToObject(root, "ap_channel", DB_WIFI_CHANNEL);
  cJSON_AddNumberToObject(root, "wifi_en_gn", DB_WIFI_EN_GN);
  cJSON_AddNumberToObject(root, "ant_use_ext", DB_EN_EXT_ANT);
  cJSON_AddNumberToObject(root, "trans_pack_size", DB_TRANS_BUF_SIZE);
  cJSON_AddNumberToObject(root, "serial_timeout", DB_SERIAL_READ_TIMEOUT_MS);
  cJSON_AddNumberToObject(root, "tx_pin", DB_UART_PIN_TX);
  cJSON_AddNumberToObject(root, "rx_pin", DB_UART_PIN_RX);
  cJSON_AddNumberToObject(root, "cts_pin", DB_UART_PIN_CTS);
  cJSON_AddNumberToObject(root, "rts_pin", DB_UART_PIN_RTS);
  cJSON_AddNumberToObject(root, "rts_thresh", DB_UART_RTS_THRESH);
  cJSON_AddNumberToObject(root, "baud", DB_UART_BAUD_RATE);
  cJSON_AddNumberToObject(root, "telem_proto", DB_SERIAL_PROTOCOL);
  cJSON_AddNumberToObject(root, "radio_dis_onarm", DB_DISABLE_RADIO_ARMED);
  cJSON_AddNumberToObject(root, "ltm_pp", DB_LTM_FRAME_NUM_BUFFER);
  cJSON_AddStringToObject(root, "ap_ip", DEFAULT_AP_IP);
  cJSON_AddStringToObject(root, "static_client_ip", DB_STATIC_STA_IP);
  cJSON_AddStringToObject(root, "static_netmask", DB_STATIC_STA_IP_NETMASK);
  cJSON_AddStringToObject(root, "static_gw_ip", DB_STATIC_STA_IP_GW);
  const char *sys_info = cJSON_Print(root);
  httpd_resp_sendstr(req, sys_info);
  free((void *)sys_info);
  cJSON_Delete(root);
  return ESP_OK;
}

/***************************************************************************************************************************
 * Public Function Definition
 **************************************************************************************************************************/

esp_err_t start_rest_server(const char *base_path) {
  REST_CHECK(base_path, "wrong base path", err);
  rest_server_context_t *rest_context = calloc(1, sizeof(rest_server_context_t));
  REST_CHECK(rest_context, "No memory for rest context", err);
  strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));

  httpd_handle_t server   = NULL;
  httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
  config.uri_match_fn     = httpd_uri_match_wildcard;
  config.max_uri_handlers = 9;

  ESP_LOGI(REST_TAG, "Starting HTTP Server");
  REST_CHECK(httpd_start(&server, &config) == ESP_OK, "Start server failed", err_start);

  /* URI handler for fetching system info */
  httpd_uri_t system_info_get_uri = {
      .uri = "/api/system/info", .method = HTTP_GET, .handler = system_info_get_handler, .user_ctx = rest_context};
  httpd_register_uri_handler(server, &system_info_get_uri);

  /* URI handler for fetching client connection info */
  httpd_uri_t system_clients_get_uri = {
      .uri = "/api/system/clients", .method = HTTP_GET, .handler = system_clients_get_handler, .user_ctx = rest_context};
  httpd_register_uri_handler(server, &system_clients_get_uri);

  /* URI handler for fetching system info */
  httpd_uri_t system_stats_get_uri = {
      .uri = "/api/system/stats", .method = HTTP_GET, .handler = system_stats_get_handler, .user_ctx = rest_context};
  httpd_register_uri_handler(server, &system_stats_get_uri);

  /* URI handler for fetching settings data */
  httpd_uri_t settings_get_uri = {
      .uri = "/api/settings", .method = HTTP_GET, .handler = settings_get_handler, .user_ctx = rest_context};
  httpd_register_uri_handler(server, &settings_get_uri);

  httpd_uri_t settings_post_uri = {
      .uri = "/api/settings", .method = HTTP_POST, .handler = settings_post_handler, .user_ctx = rest_context};
  httpd_register_uri_handler(server, &settings_post_uri);

  /* URI handler for adding a new udp client connection */
  httpd_uri_t settings_clients_udp_post_uri = {.uri      = "/api/settings/clients/udp",
                                               .method   = HTTP_POST,
                                               .handler  = settings_clients_udp_post,
                                               .user_ctx = rest_context};
  httpd_register_uri_handler(server, &settings_clients_udp_post_uri);

  /* URI handler for removing all known UDP client connections */
  httpd_uri_t settings_clients_clear_udp_get_uri = {.uri      = "/api/settings/clients/clear_udp",
                                                    .method   = HTTP_DELETE,
                                                    .handler  = settings_clients_clear_udp_get,
                                                    .user_ctx = rest_context};
  httpd_register_uri_handler(server, &settings_clients_clear_udp_get_uri);

  /* URI handler for setting a static IP for the ESP32 in Wi-Fi client mode */
  httpd_uri_t settings_static_ip_port_uri = {.uri      = "/api/settings/static-ip",
                                             .method   = HTTP_POST,
                                             .handler  = settings_static_ip_post_handler,
                                             .user_ctx = rest_context};
  httpd_register_uri_handler(server, &settings_static_ip_port_uri);

  /* URI handler for getting web server files */
  httpd_uri_t common_get_uri = {
      .uri = "/*", .method = HTTP_GET, .handler = rest_common_get_handler, .user_ctx = rest_context};
  httpd_register_uri_handler(server, &common_get_uri);

  return ESP_OK;
err_start:
  free(rest_context);
err:
  return ESP_FAIL;
}
