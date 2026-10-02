idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    # Host tests: "driver" is the CMock mock from $IDF_PATH/tools/mocks/driver
    set(requires i2c_bus driver freertos)
else()
    set(requires i2c_bus esp_driver_gpio freertos)
endif()

idf_component_register(
    SRCS
        "VL53L1X.cpp"
        "VL53L1X_config.cpp"
        "vl53l1x_platform.cpp"
        # Builds ST's VL53L1X_api.c; see the comment in this file
        "vl53l1x_uld.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "."
        "st_uld/core"
        "st_uld/platform"
    REQUIRES
        ${requires}
)

if(${target} STREQUAL "linux")
    # Host tests measure line coverage of this component (gcov); CI turns it into the coverage badge
    target_compile_options(${COMPONENT_LIB} PRIVATE --coverage)
    target_link_options(${COMPONENT_LIB} INTERFACE --coverage)
endif()
