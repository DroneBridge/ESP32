#ifndef DB_ESP32_GLOBALS_H
#define DB_ESP32_GLOBALS_H

#include <freertos/event_groups.h>
#include "db_esp32_control.h"

#define MAX_LTM_FRAMES_IN_BUFFER 5
#define BUILDVERSION 9
#define MAJOR_VERSION 1
#define MINOR_VERSION 4

// can be set by user
extern uint8_t DB_NETIF_MODE;
extern uint8_t DEFAULT_SSID[32];
extern uint8_t DEFAULT_PWD[64];
extern char DEFAULT_AP_IP[32];
extern char CURRENT_CLIENT_IP[32];  // IP address of the ESP32 when we are in client mode connected
extern uint8_t DEFAULT_CHANNEL;
extern uint8_t SERIAL_PROTOCOL;  // 1=MSP, 3=MAVLink/transparent
extern uint8_t DB_UART_PIN_TX;      // set TX & RX pin to the same number to indicate vanilla system
extern uint8_t DB_UART_PIN_RX;
extern int DB_UART_BAUD_RATE;
extern uint16_t TRANSPARENT_BUF_SIZE;
extern uint8_t LTM_FRAME_NUM_BUFFER;    // Number of LTM frames per UDP packet (min = 1; max = 5)
extern uint8_t MSP_LTM_SAMEPORT;        // 0 = no (1607 MSP, 1604 LTM); >0 = yes (1604)
extern int station_rssi;               // updated when ESP32 is in station mode and connected to an access point

extern uint32_t uart_byte_count;
extern int8_t num_connected_tcp_clients;
extern struct udp_conn_list_t *udp_conn_list;

extern int WIFI_ESP_MAXIMUM_RETRY;

#endif //DB_ESP32_GLOBALS_H
