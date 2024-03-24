#ifndef DB_ESP32_TCP_SERVER_H
#define DB_ESP32_TCP_SERVER_H

#include <stdint.h>
#include <sys/types.h>

#define TCP_BUFF_SIZ 4096

int open_tcp_server(int port);
void send_to_all_tcp_clients(const int tcp_clients[], uint8_t data[], uint data_length);

#endif  // DB_ESP32_TCP_SERVER_H
