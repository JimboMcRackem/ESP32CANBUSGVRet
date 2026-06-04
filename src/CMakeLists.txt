idf_component_register(
    SRCS
        "main.cpp"
        "config_store.cpp"
        "can_driver.cpp"
        "usb_serial.cpp"
        "indicators.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_timer freertos esp_system esp_common
    PRIV_REQUIRES gvret settings
)
