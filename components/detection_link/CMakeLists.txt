idf_build_get_property(target IDF_TARGET)

set(srcs
    "Protocol.cpp"
    "ExchangeId.cpp"
    "Clock.cpp"
    "DetectionLink.cpp"
    "EspNowTransport.cpp")

if(${target} STREQUAL "linux")
    # Host tests: "esp_wifi" is the CMock mock from $IDF_PATH/tools/mocks/esp_wifi
    set(requires esp_wifi)
else()
    # RTC memory and the hardware random number generator only exist on the chip
    list(APPEND srcs "ExchangeId_retained.cpp")
    set(requires esp_wifi esp_hw_support)
endif()

idf_component_register(
    SRCS
        ${srcs}
    INCLUDE_DIRS
        "include"
    REQUIRES
        ${requires}
)
