idf_component_register(SRCS main.c db_esp32_control.c globals.h sdkconfig.h msp_ltm_serial.c
        msp_ltm_serial.h db_protocol.h http_server.c http_server.h db_esp32_comm.c
        db_esp32_comm.h db_comm_protocol.h db_comm.c db_comm.h db_crc.c db_crc.h tcp_server.c tcp_server.h
        http_server.c http_server.h
        INCLUDE_DIRS ".")

if(CONFIG_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../frontend")
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT)
#    if(EXISTS ${WEB_SRC_DIR}/dist)
#        spiffs_create_partition_image(www ${WEB_SRC_DIR}/dist FLASH_IN_PROJECT)
#    else()
#        message(FATAL_ERROR "${WEB_SRC_DIR}/dist doesn't exit. Please run 'npm run build' in ${WEB_SRC_DIR}")
#    endif()
endif()