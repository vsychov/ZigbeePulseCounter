#pragma once

#include <stdint.h>
#include <stdbool.h>

#define APP_NVS_NAMESPACE "meter"
#define APP_NVS_KEY_CONFIG "cfg"
#define APP_NVS_KEY_PULSES "pulses"

#define APP_MFG_CODE CONFIG_ZB_MANUFACTURER_CODE
#define APP_MFG_CLUSTER_ID CONFIG_ZB_MFG_CLUSTER_ID
#define APP_MANUFACTURER_NAME "Custom"

#if CONFIG_ZB_VARIANT_ELECTRIC
#define APP_UNIT_OF_MEASURE 0
#define APP_METERING_DEVICE_TYPE 0
#define APP_MODEL_IDENTIFIER "ESP32-PulseMeter-Electric"
#elif CONFIG_ZB_VARIANT_WATER
#define APP_UNIT_OF_MEASURE 1
#define APP_METERING_DEVICE_TYPE 2
#define APP_MODEL_IDENTIFIER "ESP32-PulseMeter-Water"
#else
#define APP_UNIT_OF_MEASURE 1
#define APP_METERING_DEVICE_TYPE 1
#define APP_MODEL_IDENTIFIER "ESP32-PulseMeter-Gas"
#endif

#define APP_ZB_ENDPOINT CONFIG_ZB_ENDPOINT
#define APP_ZB_DEVICE_ID CONFIG_ZB_DEVICE_ID
#define APP_ZB_REPORT_DST_SHORT_ADDR CONFIG_ZB_REPORT_DST_SHORT_ADDR
#define APP_ZB_REPORT_DST_ENDPOINT CONFIG_ZB_REPORT_DST_ENDPOINT

typedef struct {
    uint32_t pulse_per_unit_numerator;
    uint8_t metering_device_type;
    uint8_t unit_of_measure;
    uint16_t debounce_ms;
} app_metering_cfg_t;

void app_config_load(app_metering_cfg_t *cfg);
void app_pulse_load_total(uint64_t *total);
void app_pulse_save_total(uint64_t total);
void app_config_reset_counter_request(void);
bool app_config_consume_reset_request(void);
