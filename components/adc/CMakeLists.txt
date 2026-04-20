idf_component_register(
    SRCS "adc_shared.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_adc
)
