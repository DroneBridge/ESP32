idf_component_register(SRCS "tcp_server.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_event log lwip)