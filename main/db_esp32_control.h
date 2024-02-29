#ifndef DB_ESP32_DB_ESP32_CONTROL_H
#define DB_ESP32_DB_ESP32_CONTROL_H

#define TRANS_RD_BYTES_NUM 8  // amount of bytes read form serial port at once when transparent is selected
#define UART_BUF_SIZE (1024)

void control_module();

#endif  // DB_ESP32_DB_ESP32_CONTROL_H
