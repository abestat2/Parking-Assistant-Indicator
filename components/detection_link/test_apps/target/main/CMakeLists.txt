set(srcs
    "test_app_main.cpp"
    "test_retained_id.cpp"
    "../../common/test_protocol.cpp"
    "../../common/test_exchange_id.cpp"
    "../../common/test_link_receiver.cpp"
    "../../common/test_link_detector.cpp"
    "../../common/test_link_pair.cpp")

# QEMU has no Wi-Fi radio, and an app that links the Wi-Fi driver stalls at startup there,
# so the [hw] radio tests are only built for the board
if(NOT CONFIG_DETECTION_LINK_TEST_QEMU)
    list(APPEND srcs "test_espnow_radio.cpp")
endif()

idf_component_register(
    SRCS
        ${srcs}
    PRIV_REQUIRES
        unity
        detection_link
        esp_wifi
        esp_event
        nvs_flash
    WHOLE_ARCHIVE
)
