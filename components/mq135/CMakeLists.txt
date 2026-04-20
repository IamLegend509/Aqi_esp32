idf_component_register(
    SRCS "mq135.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_adc adc driver
)