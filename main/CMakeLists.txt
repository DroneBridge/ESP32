idf_component_register(SRCS main.c db_esp32_control.c mavlink_parser.c
                            db_esp32_comm.c http_server.c db_comm.c
                       INCLUDE_DIRS ".")

if(CONFIG_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../frontend")
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT)
endif()