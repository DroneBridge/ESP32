cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "../.." "../../../../common_components/linux_compat")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(COMPONENTS main esp_netif_linux)
project(mdns_host)

# Enable sanitizers for mdns implementation
idf_component_get_property(mdns mdns COMPONENT_LIB)
target_link_options(${mdns} INTERFACE -fsanitize=address -fsanitize=undefined)
