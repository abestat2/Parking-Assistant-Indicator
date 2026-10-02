idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        i2c_bus
        vl53l1x
        esp_wifi
        esp_http_server
        nvs_flash
)
