#include "udp_server.h"

#include <esp_log.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>

#define TAG "UDP_SERVER"

/**
 * @brief Open non-blocking UDP socket
 *
 * @return int socket file descriptor
 */
int open_udp_socket() {
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(APP_PORT_PROXY_UDP);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(server_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int udp_socket = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }
    int err = bind(udp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    fcntl(udp_socket, F_SETFL, O_NONBLOCK);
    ESP_LOGI(TAG, "Opened UDP socket on port %i", APP_PORT_PROXY_UDP);
    return udp_socket;
}

/**
 * @brief Sends unit8_t array to all connected UDP clients
 *
 * @param n_udp_conn_list list of connected UDP clients
 * @param data pointer to uint8_t array of data to send
 * @param data_length length of the data to send (in bytes)
 */
void send_to_all_udp_clients(struct udp_conn_list_t *n_udp_conn_list, const uint8_t *data, uint data_length) {
    for (int i = 0; i < n_udp_conn_list->size; i++) {  // send to all UDP clients
    resend:;
        int sent = sendto(n_udp_conn_list->udp_socket, data, data_length, 0,
                          (struct sockaddr *)&n_udp_conn_list->db_udp_clients[i].udp_client,
                          sizeof(n_udp_conn_list->db_udp_clients[i].udp_client));
        if (sent != data_length) {
            ESP_LOGE(TAG, "UDP - Error sending (%i/%i) because of %d - trying again!", sent, data_length, errno);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            goto resend;
        }
    }
}

/**
 * @brief Create structure containing all UDP connection information
 *
 * @return struct udp_conn_list_t* Structure containing all UDP connection information
 */
struct udp_conn_list_t *udp_client_list_create() {
    struct udp_conn_list_t *n_udp_conn_list = malloc(sizeof(struct udp_conn_list_t));  // Allocate memory for the list
    if (n_udp_conn_list == NULL) {                                                     // Check if the allocation failed
        return NULL;                                                                   // Return NULL to indicate an error
    }
    n_udp_conn_list->size = 0;  // Initialize the size to 0
    return n_udp_conn_list;     // Return the pointer to the list
}

/**
 * @brief Destroy structure containing all UDP connection information
 *
 * @param n_udp_conn_list Structure containing all UDP connection information
 */
void udp_client_list_destroy(struct udp_conn_list_t *n_udp_conn_list) {
    if (n_udp_conn_list == NULL) {  // Check if the list is NULL
        return;                     // Do nothing
    }
    free(n_udp_conn_list);  // Free the list
}

/**
 * @brief Add a new UDP client to the list of known UDP clients. Checks if client is already known based on IP and port.
 * Added client will receive UDP packets with serial info and will be able to send UDP packets to the serial interface
 * of the ESP32.
 * PORT, MAC & IP should be set inside new_db_udp_client. If MAC is not set then the device cannot be removed later on.
 *
 * @param n_udp_conn_list Structure containing all UDP connection information
 * @param new_db_udp_client New client to add to the UDP list. PORT, MAC & IP must be set. If MAC is not set then the
 *                          device cannot be removed later on.
 */
void add_to_known_udp_clients(struct udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client) {
    if (n_udp_conn_list == NULL) {  // Check if the list is NULL
        return;                     // Do nothing
    }
    if (n_udp_conn_list->size == MAX_UDP_CLIENTS) {  // Check if the list is full
        return;                                      // Do nothing
    }
    for (int i = 0; i < n_udp_conn_list->size; i++) {
        if ((n_udp_conn_list->db_udp_clients[i].udp_client.sin_port == new_db_udp_client.udp_client.sin_port) &&
            (n_udp_conn_list->db_udp_clients[i].udp_client.sin_addr.s_addr ==
             new_db_udp_client.udp_client.sin_addr.s_addr)) {
            return;  // client existing - do not add
        }
    }
    n_udp_conn_list->db_udp_clients[n_udp_conn_list->size] = new_db_udp_client;  // Copy the element data to the end of the array
    n_udp_conn_list->size++;                                                     // Increment the size of the list
}

/**
 * @brief Remove a client from the sending list. Client will no longer receive UDP packets. MAC address must be given.
 * Usually called in AP-Mode when a station disconnects. In any other case we will not know since UDP is a connection-less
 * protocol
 *
 * @param n_udp_conn_list Structure containing all UDP connection information
 * @param new_db_udp_client The UDP client to remove based on its MAC address
 */
void remove_from_known_udp_clients(struct udp_conn_list_t *n_udp_conn_list, struct db_udp_client_t new_db_udp_client) {
    if (n_udp_conn_list == NULL) {  // Check if the list is NULL
        return;                     // Do nothing
    }
    for (int i = 0; i < n_udp_conn_list->size; i++) {  // Loop through the array
        if (memcmp(n_udp_conn_list->db_udp_clients[i].mac, new_db_udp_client.mac,
                   sizeof(n_udp_conn_list->db_udp_clients[i].mac)) ==
            0) {  // Compare the current array element with the element
            // Found a match
            for (int j = i; j < n_udp_conn_list->size - 1; j++) {  // Loop from the current index to the end of the array
                n_udp_conn_list->db_udp_clients[j] = n_udp_conn_list->db_udp_clients[j +
                                                                                     1];  // Shift the array elements to the left
            }
            n_udp_conn_list->size--;  // Decrement the size of the list
            return;                   // Exit the function
        }
    }
    // No match found
}
