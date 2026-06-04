# Portable protocol/settings code lives in lib/ and is also unit-tested in the
# native env. PlatformIO's Library Dependency Finder compiles those lib/
# sources and adds their include paths automatically (for both the esp32dev and
# native builds) because this component includes their headers — so they must
# NOT be listed in SRCS here, or they'd be compiled twice (duplicate symbols).
idf_component_register(
    SRCS
        "main.cpp"
        "config_store.cpp"
        "can_driver.cpp"
        "usb_serial.cpp"
        "indicators.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_timer freertos esp_system esp_common
)
