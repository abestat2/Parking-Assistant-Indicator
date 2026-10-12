idf_component_register(
    SRCS "main.cpp" "RangeBench.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        i2c_bus
        vl53l1x
        mb1013
        detection_link
        esp_driver_gpio
        esp_driver_usb_serial_jtag
        esp_event
        esp_timer
        esp_wifi
        esp_http_server
        nvs_flash
        parking_indicator
)
