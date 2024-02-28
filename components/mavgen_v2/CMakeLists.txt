idf_component_register(INCLUDE_DIRS "c_library_v2")
set_target_properties(${COMPONENT_LIB} PROPERTIES INTERFACE_COMPILE_OPTIONS -Wno-address-of-packed-member)