idf_component_register(SRCS "udp_server.c"
                       INCLUDE_DIRS "."
                       REQUIRES lwip
                       PRIV_REQUIRES esp_event log)