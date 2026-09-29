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
