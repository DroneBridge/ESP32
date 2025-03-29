idf_component_register(SRCS fastmavlink_config.h common/. lib/.
        INCLUDE_DIRS "."
)
component_compile_options(-Wno-address-of-packed-member)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-address-of-packed-member)