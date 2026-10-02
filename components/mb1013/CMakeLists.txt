idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    # Host tests: "driver" is the CMock mock from $IDF_PATH/tools/mocks/driver
    set(requires driver)
else()
    set(requires esp_driver_gpio esp_driver_rmt)
endif()

idf_component_register(
    SRCS
        "MB1013.cpp"
        "MB1013_decode.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        ${requires}
)

if(${target} STREQUAL "linux")
    # Host tests measure line coverage of this component (gcov); CI turns it into the coverage badge
    target_compile_options(${COMPONENT_LIB} PRIVATE --coverage)
    target_link_options(${COMPONENT_LIB} INTERFACE --coverage)
endif()
