#include <cJSON.h>
#include <stdint.h>
#include <string.h>
#include "db_comm_protocol.h"
#include "db_protocol.h"
#include "db_comm.h"
#include "crc.h"


/**
 * @brief Add crc32 at end and build final array
 * 
 * @param msg_buf Message buffer (uint8_t array pointer)
 * @param req_json JSON string
 * @return int Length of response
 */
int finalize_message(uint8_t *msg_buf, char *req_json) {
    size_t json_length = strlen(req_json);
    uint32_t new_crc = calc_crc32((uint32_t) 0, (unsigned char *) req_json, strlen(req_json));
    msg_buf[json_length] = new_crc;
    msg_buf[json_length + 1] = new_crc >> 8;
    msg_buf[json_length + 2] = new_crc >> 16;
    msg_buf[json_length + 3] = new_crc >> 24;
    memcpy(msg_buf, req_json, json_length);
    return (int) json_length + 4;
}


/**
 * @brief Generate a system response message
 *
 * @param message_buffer Buffer where generated message will be placed
 * @param id Communication message ID to respond to
 * @param new_fw_id Firmware ID to respond with
 * @return Length of response
 */
int gen_db_comm_sys_ident_json(uint8_t *message_buffer, int id, int new_fw_id) {
    cJSON *root;
    root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, DB_COMM_KEY_DEST, DB_COMM_DST_GCS);
    cJSON_AddStringToObject(root, DB_COMM_KEY_TYPE, DB_COMM_TYPE_SYS_IDENT_RESPONSE);
    cJSON_AddStringToObject(root, DB_COMM_KEY_ORIGIN, DB_COMM_ORIGIN_GND);
    cJSON_AddNumberToObject(root, DB_COMM_KEY_HARDWID, DB_SYS_HID_ESP32);
    cJSON_AddNumberToObject(root, DB_COMM_KEY_FIRMWID, new_fw_id);
    cJSON_AddNumberToObject(root, DB_COMM_KEY_ID, id);
    return finalize_message(message_buffer, cJSON_Print(root));
}


int gen_db_comm_ping_resp(uint8_t *message_buffer, int id) {
    cJSON *root;
    root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, DB_COMM_KEY_DEST, DB_COMM_DST_GCS);
    cJSON_AddStringToObject(root, DB_COMM_KEY_TYPE, DB_COMM_TYPE_PING_RESPONSE);
    cJSON_AddStringToObject(root, DB_COMM_KEY_ORIGIN, DB_COMM_ORIGIN_GND);
    cJSON_AddNumberToObject(root, DB_COMM_KEY_ID, id);
    return finalize_message(message_buffer, cJSON_Print(root));
}


/**
 * @brief Generate error response
 *
 * @param message_buffer Buffer where generated message will be placed
 * @param id Communication message ID to respond to
 * @param error_message The error message
 * @return Length of response
 */
int gen_db_comm_err_resp(uint8_t *message_buffer, int id, char *error_message) {
    cJSON *root;
    root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, DB_COMM_KEY_DEST, DB_COMM_DST_GCS);
    cJSON_AddStringToObject(root, DB_COMM_KEY_TYPE, DB_COMM_TYPE_ERROR);
    cJSON_AddStringToObject(root, DB_COMM_KEY_ORIGIN, DB_COMM_ORIGIN_GND);
    cJSON_AddStringToObject(root, DB_COMM_KEY_MSG, error_message);
    cJSON_AddNumberToObject(root, DB_COMM_KEY_ID, id);
    return finalize_message(message_buffer, cJSON_Print(root));
}