idf_component_register(SRCS
    main.c
    db_esp32_control.c
    msp_ltm_serial.c
    http_server.c
    db_crc.c
    tcp_server.c
    db_esp_now.c
    db_serial.c
    db_mavlink_msgs.c
    db_ble.c
    INCLUDE_DIRS ".")

add_compile_options(-Wno-address-of-packed-member) # Supress Mavlink Warnings
if(CONFIG_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_BINARY_DIR}/frontend")
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT DEPENDS frontend)
endif()
