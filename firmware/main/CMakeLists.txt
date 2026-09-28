idf_component_register(
    SRCS "main.c" "flight.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2c esp_adc esp_timer bt nvs_flash
)
