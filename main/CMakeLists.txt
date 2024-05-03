idf_component_register(SRCS main.c db_esp32_control.c globals.h sdkconfig.h msp_ltm_serial.c
        msp_ltm_serial.h db_protocol.h http_server.c http_server.h db_esp32_comm.c
        db_esp32_comm.h db_comm_protocol.h db_comm.c db_comm.h db_crc.c db_crc.h tcp_server.c tcp_server.h
        http_server.c http_server.h db_esp_now.c db_esp_now.h db_serial.c db_serial.h
        INCLUDE_DIRS ".")

if(CONFIG_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../frontend")
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT)
endif()