#ifndef DB_ESP32_DB_ESP32_CONTROL_H
#define DB_ESP32_DB_ESP32_CONTROL_H

#include <lwip/sockets.h>

#define MAX_UDP_CLIENTS 8
#define TRANS_RD_BYTES_NUM  8   // amount of bytes read form serial port at once when transparent is selected
#define UDP_BUF_SIZE    2048
#define UART_BUF_SIZE   (1024)

// per client structure of connected devices in softAP mode
struct db_udp_client_t {
    uint8_t mac[6];     // MAC address of connected client
    struct sockaddr_in udp_client;    // socket address (IP & PORT) of connected client
};

struct udp_conn_list_t {
    struct db_udp_client_t db_udp_clients[MAX_UDP_CLIENTS]; // The array of list items
    int size; // The number of items in the list
    int udp_socket;     // ID of UDP socket
};

void control_module();
struct udp_conn_list_t *udp_client_list_create();
void udp_client_list_destroy(struct udp_conn_list_t *n_udp_conn_list);
void add_to_known_udp_clients(struct udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client);
void remove_from_known_udp_clients(struct udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client);

#endif //DB_ESP32_DB_ESP32_CONTROL_H
