idf_component_register(
    SRCS "dht22.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio
)