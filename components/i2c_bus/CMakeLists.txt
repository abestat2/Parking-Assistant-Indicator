idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    # Host tests only use the I2cDevice interface; the ESP-IDF I2C master driver is not available there
    set(srcs)
    set(requires)
else()
    set(srcs "I2cBus.cpp")
    set(requires esp_driver_i2c)
endif()

idf_component_register(
    SRCS
        ${srcs}
    INCLUDE_DIRS
        "include"
    REQUIRES
        ${requires}
)
