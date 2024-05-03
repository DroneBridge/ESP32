idf_component_register(SRCS fastmavlink_config.h common/. lib/.
        INCLUDE_DIRS "."
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-address-of-packed-member)