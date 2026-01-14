#include "config_cluster.h"

#include <string.h>
#include "sdkconfig.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "zcl/esp_zigbee_zcl_common.h"

static const char *TAG = "cfg_cluster";
static uint8_t s_reset_counter_attr;
static bool s_reset_pending;

void app_config_load(app_metering_cfg_t *cfg)
{
    app_metering_cfg_t defaults = {
        .pulse_per_unit_numerator = CONFIG_PULSE_PER_UNIT_NUMERATOR,
        .metering_device_type = APP_METERING_DEVICE_TYPE,
        .unit_of_measure = APP_UNIT_OF_MEASURE,
        .debounce_ms = CONFIG_PULSE_DEBOUNCE_MS,
    };

    *cfg = defaults;
    if (cfg->pulse_per_unit_numerator == 0) {
        cfg->pulse_per_unit_numerator = defaults.pulse_per_unit_numerator;
    }
    if (cfg->debounce_ms == 0) {
        cfg->debounce_ms = defaults.debounce_ms;
    }
    cfg->metering_device_type = APP_METERING_DEVICE_TYPE;
    cfg->unit_of_measure = APP_UNIT_OF_MEASURE;
}

void app_pulse_load_total(uint64_t *total)
{
    *total = 0;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(APP_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        err = nvs_get_u64(nvs, APP_NVS_KEY_PULSES, total);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "nvs_get_u64(%s) failed: %s", APP_NVS_KEY_PULSES, esp_err_to_name(err));
        }
        nvs_close(nvs);
    } else {
        ESP_LOGW(TAG, "nvs_open(read) failed: %s", esp_err_to_name(err));
    }
}

void app_pulse_save_total(uint64_t total)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_u64(nvs, APP_NVS_KEY_PULSES, total);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "nvs_save_total failed: %s", esp_err_to_name(err));
        }
        nvs_close(nvs);
    } else {
        ESP_LOGW(TAG, "nvs_open(write) failed: %s", esp_err_to_name(err));
    }
}

void app_config_reset_counter_request(void)
{
    ESP_LOGI(TAG, "Reset counter requested");
    s_reset_pending = true;
}

bool app_config_consume_reset_request(void)
{
    if (s_reset_pending) {
        s_reset_pending = false;
        ESP_LOGI(TAG, "Reset counter pending flag consumed");
        return true;
    }
    return false;
}

void config_cluster_add(esp_zb_cluster_list_t *cluster_list, app_metering_cfg_t *cfg)
{
    (void)cfg;
    s_reset_counter_attr = 0;

    esp_zb_attribute_list_t *attr_list = esp_zb_zcl_attr_list_create(APP_MFG_CLUSTER_ID);

    esp_zb_cluster_add_manufacturer_attr(attr_list, APP_MFG_CLUSTER_ID, 0x0008, APP_MFG_CODE,
                                         ESP_ZB_ZCL_ATTR_TYPE_BOOL,
                                         ESP_ZB_ZCL_ATTR_ACCESS_WRITE_ONLY,
                                         &s_reset_counter_attr);

    esp_zb_cluster_list_add_custom_cluster(cluster_list, attr_list,
                                           ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

void config_cluster_apply_pending(app_metering_cfg_t *cfg)
{
    (void)cfg;
    if (s_reset_counter_attr) {
        ESP_LOGI(TAG, "Reset attribute set via manufacturer cluster");
        s_reset_counter_attr = 0;
        app_config_reset_counter_request();
    }
}

void config_cluster_register_callbacks(void)
{
    (void)APP_MFG_CODE;
}
