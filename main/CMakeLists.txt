idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        mb1013
        esp_wifi
        esp_http_server
        nvs_flash
)