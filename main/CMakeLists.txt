idf_component_register(
    SRCS
        "main.c"
        "pulse.c"
        "metering.c"
        "power.c"
        "ota.c"
        "config_cluster.c"
    INCLUDE_DIRS "."
    REQUIRES esp-zigbee-lib nvs_flash driver esp_adc esp_timer app_update
)
