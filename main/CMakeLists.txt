
idf_component_register(
    SRCS
        "main.c"
        "imu.c"
        "fusion.c"
        "radio.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_driver_spi
        esp_driver_i2c
        esp_driver_gpio
        esp_timer
)
