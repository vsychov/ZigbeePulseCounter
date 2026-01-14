#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_pm.h"

#include "esp_zigbee_core.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_attribute.h"
#include "nwk/esp_zigbee_nwk.h"
#include "hal/ieee802154_ll.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "zcl/esp_zigbee_zcl_metering.h"
#include "zcl/esp_zigbee_zcl_ota.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "esp_zigbee_ota.h"
#include "esp_zigbee_trace.h"

#include "app_config.h"
#include "pulse.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "metering.h"
#include "power.h"
#include "ota.h"
#include "config_cluster.h"
#include "app_config.h"

#ifndef CONFIG_ZB_STEER_MAX_RETRIES
#define CONFIG_ZB_STEER_MAX_RETRIES 0
#endif
#ifndef CONFIG_ZB_STEER_COOLDOWN_S
#define CONFIG_ZB_STEER_COOLDOWN_S 0
#endif

#if CONFIG_FACTORY_RESET_BUTTON_GPIO >= 0 && CONFIG_FACTORY_RESET_BUTTON_GPIO == CONFIG_PULSE_GPIO
#error "CONFIG_FACTORY_RESET_BUTTON_GPIO must differ from CONFIG_PULSE_GPIO"
#endif

#if CONFIG_IDF_TARGET_ESP32H2
#if CONFIG_PULSE_GPIO < 7 || CONFIG_PULSE_GPIO > 14
#error "CONFIG_PULSE_GPIO must be an RTC-capable GPIO (GPIO7-14 on ESP32H2)"
#endif
#if CONFIG_FACTORY_RESET_BUTTON_GPIO >= 0 && (CONFIG_FACTORY_RESET_BUTTON_GPIO < 7 || CONFIG_FACTORY_RESET_BUTTON_GPIO > 14)
#error "CONFIG_FACTORY_RESET_BUTTON_GPIO must be RTC-capable (GPIO7-14)"
#endif
#elif CONFIG_IDF_TARGET_ESP32C6
#if CONFIG_PULSE_GPIO < 0 || CONFIG_PULSE_GPIO > 7
#error "CONFIG_PULSE_GPIO must be an RTC-capable GPIO (GPIO0-7 on ESP32C6)"
#endif
#if CONFIG_FACTORY_RESET_BUTTON_GPIO >= 0 && (CONFIG_FACTORY_RESET_BUTTON_GPIO < 0 || CONFIG_FACTORY_RESET_BUTTON_GPIO > 7)
#error "CONFIG_FACTORY_RESET_BUTTON_GPIO must be RTC-capable (GPIO0-7)"
#endif
#endif

#ifndef CONFIG_ZB_TX_POWER_DBM
#define CONFIG_ZB_TX_POWER_DBM IEEE802154_TXPOWER_VALUE_MAX
#endif
#define APP_ZB_TX_POWER_DBM CONFIG_ZB_TX_POWER_DBM
#define APP_ZB_TX_POWER_JOIN_DBM IEEE802154_TXPOWER_VALUE_MAX

#define APP_EVENT_QUEUE_LEN 32
#define APP_SAVE_INTERVAL_US (60ULL * 1000000ULL)
#define APP_SAVE_DEBOUNCE_US (5ULL * 1000000ULL)
/* On battery it is fine to sample less often to reduce wakeups. */
#define APP_BATTERY_TASK_PERIOD_BATT_MS (60 * 60 * 1000)
#define APP_ZB_SLEEP_THRESHOLD_MS 20
#define APP_STEER_RETRY_BASE_S 5
#define APP_STEER_RETRY_MAX_S 60
#define APP_STEER_MAX_RETRIES CONFIG_ZB_STEER_MAX_RETRIES
#define APP_STEER_COOLDOWN_S CONFIG_ZB_STEER_COOLDOWN_S
#define APP_DEMAND_IDLE_TIMEOUT_US (60ULL * 1000000ULL)
#define APP_DEMAND_IDLE_CHECK_US (1ULL * 1000000ULL)
#define APP_FACTORY_RESET_HOLD_MS 8000
#define APP_FACTORY_RESET_POLL_MS 50
#define APP_FACTORY_RESET_HOLD_US (APP_FACTORY_RESET_HOLD_MS * 1000ULL)
#define APP_FACTORY_RESET_POLL_US (APP_FACTORY_RESET_POLL_MS * 1000ULL)
#define APP_OTA_ELEMENT_HEADER_LEN 6
#define APP_SLEEP_JOIN_BLOCK_US (30LL * 1000000LL)

static const char *TAG = "zigbee_meter";

typedef enum {
    APP_EVENT_BATTERY,
} app_event_type_t;

typedef struct {
    app_event_type_t type;
    power_status_t power;
} app_event_t;

static QueueHandle_t s_app_event_queue;
static TaskHandle_t s_zigbee_task_handle;
static app_metering_cfg_t s_cfg;
static uint8_t s_last_battery_percent;
static uint8_t s_last_battery_voltage;
static int64_t s_last_save_us;
static esp_timer_handle_t s_steer_retry_timer;
static volatile bool s_request_steer;
static uint8_t s_steer_retry_count;
static bool s_steer_started;
static bool s_joined;
static int64_t s_last_demand_check_us;
static bool s_total_dirty;
static volatile bool s_factory_reset_requested;
static uint32_t s_steer_total_attempts;
static int64_t s_last_can_sleep_skip_log_us;
static int64_t s_no_sleep_until_us;
static button_handle_t s_reset_button;

static void app_zigbee_update_metering_attrs_static(void);
static void app_zigbee_update_metering_attrs_dynamic(void);
static void app_log_power_status(const power_status_t *status, const char *context);
static void app_log_wakeup_info(int64_t slept_ms);
static void app_factory_reset_press_cb(void *btn, void *data);
static void app_factory_reset_hold_cb(void *btn, void *data);

#define APP_ZCL_STR_LEN(str) ((uint8_t)(sizeof(str) - 1))
#define APP_ZCL_STR_SIZE(str) (1 + APP_ZCL_STR_LEN(str))

static uint8_t s_zb_manufacturer_name[APP_ZCL_STR_SIZE(APP_MANUFACTURER_NAME)];
static uint8_t s_zb_model_identifier[APP_ZCL_STR_SIZE(APP_MODEL_IDENTIFIER)];
static uint8_t s_zb_sw_build_id[17];
/* ZCL attribute storage must stay valid for the lifetime of the stack. */
static uint8_t s_attr_battery_voltage;
static uint8_t s_attr_battery_percent;
static esp_zb_uint48_t s_attr_summation;
static uint8_t s_attr_unit;
static uint8_t s_attr_summation_formatting;
static uint8_t s_attr_demand_formatting;
static uint8_t s_attr_device_type;
static esp_zb_uint24_t s_attr_multiplier;
static esp_zb_uint24_t s_attr_divisor;
static esp_zb_int24_t s_attr_demand;
static uint16_t s_attr_ota_stack_version;
static uint16_t s_attr_ota_downloaded_stack_version;
static uint32_t s_attr_ota_image_stamp;
static uint16_t s_attr_ota_server_addr;
static uint8_t s_attr_ota_server_ep;
static esp_zb_zcl_ota_upgrade_client_variable_t s_attr_ota_client_var;
static const esp_partition_t *s_ota_partition;
static esp_ota_handle_t s_ota_handle;
static uint32_t s_ota_total_size;
static uint32_t s_ota_expected_size;
static uint32_t s_ota_offset;
static bool s_ota_tag_received;
/* Flags to auto-detect OTA file format */
static bool s_ota_has_element_header;
static bool s_ota_header_checked;
static int64_t s_ota_start_time_us;

static void app_update_sw_build_id(void)
{
    int len = snprintf((char *)&s_zb_sw_build_id[1], 16, "PU=%" PRIu32 " DB=%u",
                       s_cfg.pulse_per_unit_numerator, s_cfg.debounce_ms);
    if (len < 0) {
        len = 0;
    } else if (len > 15) {
        len = 15;
    }
    s_zb_sw_build_id[0] = (uint8_t)len;
}

static esp_zb_uint48_t app_to_uint48(uint64_t value)
{
    esp_zb_uint48_t out = {
        .low = (uint32_t)(value & 0xFFFFFFFFULL),
        .high = (uint16_t)((value >> 32) & 0xFFFF),
    };
    return out;
}

static esp_zb_uint24_t app_to_uint24(uint32_t value)
{
    esp_zb_uint24_t out = {
        .low = (uint16_t)(value & 0xFFFF),
        .high = (uint8_t)((value >> 16) & 0xFF),
    };
    return out;
}

static esp_zb_int24_t app_to_int24(int32_t value)
{
    if (value > 0x7FFFFF) {
        value = 0x7FFFFF;
    } else if (value < -0x800000) {
        value = -0x800000;
    }

    uint32_t raw = (uint32_t)value & 0xFFFFFF;
    esp_zb_int24_t out = {
        .low = (uint16_t)(raw & 0xFFFF),
        .high = (int8_t)((raw >> 16) & 0xFF),
    };
    return out;
}

static void app_event_send(const app_event_t *evt)
{
    if (s_app_event_queue) {
        if (xQueueSend(s_app_event_queue, evt, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Dropping app event %d: queue full", evt->type);
        }
    }
}

static esp_err_t app_ota_element_slice(uint32_t total_size, const void *payload, uint16_t payload_size,
                                       const void **out_data, uint16_t *out_len)
{
    /* Support two formats:
     * 1) Plain Zigbee OTA file: payload is the raw file bytes.
     * 2) Wrapped in sub-element 0x0000: [tag(2) | len(4) | data...]; skip header once.
     */
    typedef struct __attribute__((packed)) {
        uint16_t tag_id;
        uint32_t length;
    } ota_element_header_t;

    if (!payload || payload_size == 0 || !out_data || !out_len) {
        return ESP_ERR_INVALID_ARG;
    }

    /* One-time detect sub-element 0x0000 header */
    if (!s_ota_header_checked) {
        s_ota_header_checked = true;
        s_ota_has_element_header = false;

        if (payload_size >= APP_OTA_ELEMENT_HEADER_LEN) {
            ota_element_header_t hdr;
            memcpy(&hdr, payload, sizeof(hdr));

            /* Wrapped file: tag_id 0x0000 and length + header == total size */
            if (hdr.tag_id == 0x0000 && (hdr.length + APP_OTA_ELEMENT_HEADER_LEN) == total_size) {
                s_ota_has_element_header = true;
                s_ota_expected_size = hdr.length;
            }
        }
    }

    if (s_ota_has_element_header) {
        if (!s_ota_tag_received) {
            if (payload_size <= APP_OTA_ELEMENT_HEADER_LEN) {
                return ESP_ERR_INVALID_ARG;
            }
            *out_data = (const uint8_t *)payload + APP_OTA_ELEMENT_HEADER_LEN;
            *out_len = payload_size - APP_OTA_ELEMENT_HEADER_LEN;
            s_ota_tag_received = true;
        } else {
            *out_data = payload;
            *out_len = payload_size;
        }
    } else {
        *out_data = payload;
        *out_len = payload_size;
    }

    return ESP_OK;
}

static esp_err_t app_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message)
{
    if (message.info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "OTA status cb: zcl status %u", message.info.status);
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;

    switch (message.upgrade_status) {
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
        ESP_LOGI(TAG, "-- OTA upgrade start (ver 0x%08lx, type 0x%x, mfg 0x%x)",
                 message.ota_header.file_version, message.ota_header.image_type, message.ota_header.manufacturer_code);
        s_ota_partition = esp_ota_get_next_update_partition(NULL);
        if (!s_ota_partition) {
            ESP_LOGE(TAG, "No OTA partition");
            return ESP_FAIL;
        }
        s_ota_total_size = message.ota_header.image_size;
        s_ota_expected_size = message.ota_header.image_size;
        s_ota_offset = 0;
        s_ota_tag_received = false;
        s_ota_has_element_header = false;
        s_ota_header_checked = false;
        s_ota_start_time_us = esp_timer_get_time();
        ret = esp_ota_begin(s_ota_partition, 0, &s_ota_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(ret));
        }
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
        if (message.payload_size && message.payload) {
            const void *data = NULL;
            uint16_t len = 0;
            ret = app_ota_element_slice(message.ota_header.image_size, message.payload, message.payload_size, &data, &len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "OTA slice parse failed: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = esp_ota_write(s_ota_handle, data, len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_write failed at %u: %s", s_ota_offset, esp_err_to_name(ret));
                return ret;
            }
            s_ota_offset += len;
            uint32_t target = s_ota_expected_size ? s_ota_expected_size : s_ota_total_size;
            if (target > 0) {
                ESP_LOGI(TAG, "-- OTA recv progress [%u/%u]", s_ota_offset, target);
            } else {
                ESP_LOGI(TAG, "-- OTA recv progress [%u/unknown]", s_ota_offset);
            }
        }
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
        ret = (s_ota_offset == (s_ota_expected_size ? s_ota_expected_size : s_ota_total_size)) ? ESP_OK : ESP_FAIL;
        ESP_LOGI(TAG, "-- OTA check status: %s", esp_err_to_name(ret));
        s_ota_offset = 0;
        s_ota_total_size = 0;
        s_ota_expected_size = 0;
        s_ota_tag_received = false;
        s_ota_has_element_header = false;
        s_ota_header_checked = false;
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
        ESP_LOGI(TAG, "-- OTA apply");
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
        ESP_LOGI(TAG, "-- OTA finish: image 0x%08lx size %ld bytes, time %lld ms",
                 message.ota_header.file_version, (long)message.ota_header.image_size,
                 (long long)((esp_timer_get_time() - s_ota_start_time_us) / 1000));
        ret = esp_ota_end(s_ota_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = esp_ota_set_boot_partition(s_ota_partition);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "set_boot_partition failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGW(TAG, "Rebooting into new image");
        esp_restart();
        break;

    default:
        ESP_LOGI(TAG, "OTA status: %d", message.upgrade_status);
        break;
    }
    return ret;
}

static esp_err_t app_ota_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message)
{
    if (message.info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "OTA query resp status %u", message.info.status);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OTA query resp: server 0x%04hx ep %u ver 0x%08lx mfg 0x%x type 0x%x size %ld",
             message.server_addr.u.short_addr, message.server_endpoint, message.file_version,
             message.manufacturer_code, message.image_type, (long)message.image_size);
    return ESP_OK;
}


static const char *app_signal_to_str(esp_zb_app_signal_type_t sig_type)
{
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_ERROR:
        return "ZDO_ERROR";
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        return "SKIP_STARTUP";
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        return "DEVICE_FIRST_START";
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        return "DEVICE_REBOOT";
    case ESP_ZB_BDB_SIGNAL_STEERING:
        return "BDB_STEERING";
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        return "BDB_FORMATION";
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        return "PRODUCTION_CONFIG_READY";
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        return "CAN_SLEEP";
    default:
        return "OTHER";
    }
}

static const char *app_bdb_status_to_str(esp_zb_bdb_commissioning_status_t status)
{
    switch (status) {
    case ESP_ZB_BDB_STATUS_SUCCESS:
        return "SUCCESS";
    case ESP_ZB_BDB_STATUS_IN_PROGRESS:
        return "IN_PROGRESS";
    case ESP_ZB_BDB_STATUS_NOT_AA_CAPABLE:
        return "NOT_AA_CAPABLE";
    case ESP_ZB_BDB_STATUS_NO_NETWORK:
        return "NO_NETWORK";
    case ESP_ZB_BDB_STATUS_TARGET_FAILURE:
        return "TARGET_FAILURE";
    case ESP_ZB_BDB_STATUS_FORMATION_FAILURE:
        return "FORMATION_FAILURE";
    case ESP_ZB_BDB_STATUS_NO_IDENTIFY_QUERY_RESPONSE:
        return "NO_IDENTIFY_QUERY_RESPONSE";
    case ESP_ZB_BDB_STATUS_BINDING_TABLE_FULL:
        return "BINDING_TABLE_FULL";
    case ESP_ZB_BDB_STATUS_NO_SCAN_RESPONSE:
        return "NO_SCAN_RESPONSE";
    case ESP_ZB_BDB_STATUS_NOT_PERMITTED:
        return "NOT_PERMITTED";
    case ESP_ZB_BDB_STATUS_TCLK_EX_FAILURE:
        return "TCLK_EX_FAILURE";
    case ESP_ZB_BDB_STATUS_NOT_ON_A_NETWORK:
        return "NOT_ON_A_NETWORK";
    case ESP_ZB_BDB_STATUS_ON_A_NETWORK:
        return "ON_A_NETWORK";
    case ESP_ZB_BDB_STATUS_CANCELLED:
        return "CANCELLED";
    case ESP_ZB_BDB_STATUS_DEV_ANNCE_SEND_FAILURE:
        return "DEV_ANNCE_SEND_FAILURE";
    default:
        return "UNKNOWN";
    }
}

static void app_log_commissioning_state(const char *context)
{
    esp_zb_bdb_commissioning_mode_mask_t mode = esp_zb_get_bdb_commissioning_mode();
    esp_zb_bdb_commissioning_status_t status = esp_zb_get_bdb_commissioning_status();
    uint32_t channel_mask = esp_zb_get_channel_mask();
    uint32_t primary_mask = esp_zb_get_primary_network_channel_set();
    uint32_t secondary_mask = esp_zb_get_secondary_network_channel_set();
    int8_t tx_power = 0;
    bool factory_new = esp_zb_bdb_is_factory_new();
    bool joined = esp_zb_bdb_dev_joined();

    esp_zb_get_tx_power(&tx_power);
    ESP_LOGI(TAG,
             "%s: factory_new=%d joined=%d mode 0x%02x status %s(%d) channel_mask 0x%08x primary 0x%08x secondary 0x%08x tx_power %d dBm",
             context, factory_new, joined, mode, app_bdb_status_to_str(status), status, channel_mask, primary_mask,
             secondary_mask,
             tx_power);
}

static void app_factory_reset_clear_app_state_full_nvs(const char *reason)
{
    ESP_LOGW(TAG, "Clearing application state via full NVS erase (%s)", reason ? reason : "n/a");

    /* Reset runtime counters so nothing persists before reboot. */
    metering_reset();
    metering_set_instantaneous_demand(0);
    pulse_set_total(0);

    /* Disable periodic save until reboot. */
    s_total_dirty = false;
    s_last_save_us = esp_timer_get_time();

    /* Wipe the entire NVS partition. */
    (void)nvs_flash_deinit(); /* best effort; ignore errors */
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_flash_erase failed: %s", esp_err_to_name(err));
    }
}

static void app_handle_factory_reset_request(const char *reason)
{
    ESP_LOGW(TAG, "Performing factory reset (%s)", reason ? reason : "n/a");

    app_factory_reset_clear_app_state_full_nvs(reason);

    /* Zigbee factory reset is harmless after NVS wipe, keep it for stack cleanup. */
    esp_zb_factory_reset();

    vTaskDelay(pdMS_TO_TICKS(200)); /* short pause for logs */
    esp_restart();
}

/* Instead of polling on a fixed timer, only check long-press while the line is actually held low.
 * Users press the button rarely, so this is almost free for power consumption.
 */
static void app_factory_reset_button_service(void)
{
#if CONFIG_FACTORY_RESET_BUTTON_GPIO >= 0
    (void)0; /* Event-driven via iot_button callbacks. */
#endif
}

static void app_factory_reset_press_cb(void *btn, void *data)
{
    (void)btn;
    (void)data;
    ESP_LOGW(TAG, "Factory reset button pressed, hold for %u ms to reset",
             (unsigned)APP_FACTORY_RESET_HOLD_MS);
}

static void app_factory_reset_hold_cb(void *btn, void *data)
{
    (void)btn;
    (void)data;
    s_factory_reset_requested = true;
    ESP_LOGW(TAG, "Factory reset requested (button held %u ms)", (unsigned)APP_FACTORY_RESET_HOLD_MS);
}

static void app_factory_reset_button_init(void)
{
#if CONFIG_FACTORY_RESET_BUTTON_GPIO >= 0
    button_config_t btn_cfg = {
        .long_press_time = APP_FACTORY_RESET_HOLD_MS,
        .short_press_time = 0,
    };
    button_gpio_config_t gpio_cfg = {
        .gpio_num = CONFIG_FACTORY_RESET_BUTTON_GPIO,
        .active_level = 0,
        /* Allow the driver to stop its timer when idle; wake via GPIO interrupt. */
        .enable_power_save = true,
        .disable_pull = false,
    };

    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &s_reset_button);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init factory reset button on GPIO%d: %s",
                 CONFIG_FACTORY_RESET_BUTTON_GPIO, esp_err_to_name(err));
        return;
    }

    esp_err_t reg_err = ESP_OK;
    reg_err |= iot_button_register_cb(s_reset_button, BUTTON_PRESS_DOWN, NULL,
                                      app_factory_reset_press_cb, NULL);

    button_event_args_t hold_args = {
        .long_press.press_time = APP_FACTORY_RESET_HOLD_MS,
    };
    reg_err |= iot_button_register_cb(s_reset_button, BUTTON_LONG_PRESS_START, &hold_args,
                                      app_factory_reset_hold_cb, NULL);

    if (reg_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register reset button callbacks: %s", esp_err_to_name(reg_err));
    } else {
        ESP_LOGI(TAG, "Factory reset button enabled on GPIO%d, hold for %u ms to reset Zigbee stack",
                 CONFIG_FACTORY_RESET_BUTTON_GPIO, (unsigned)APP_FACTORY_RESET_HOLD_MS);
    }
#else
    ESP_LOGI(TAG, "Factory reset button disabled (CONFIG_FACTORY_RESET_BUTTON_GPIO < 0)");
#endif
}

static void app_configure_light_sleep_wakeup_sources(void)
{
    esp_err_t err;

    uint64_t ext1_mask = 0;

    /* EXT1 only (RTC GPIOs). Non-RTC pins are rejected at compile-time. */
    ext1_mask |= (1ULL << CONFIG_PULSE_GPIO);

#if CONFIG_FACTORY_RESET_BUTTON_GPIO >= 0
    /* If the reset button is on its own pin, wake on low there too. */
    if (CONFIG_FACTORY_RESET_BUTTON_GPIO != CONFIG_PULSE_GPIO) {
        ext1_mask |= (1ULL << CONFIG_FACTORY_RESET_BUTTON_GPIO);
    }
#endif

    if (ext1_mask) {
        err = esp_sleep_enable_ext1_wakeup(ext1_mask, ESP_EXT1_WAKEUP_ANY_LOW);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_sleep_enable_ext1_wakeup(mask=0x%" PRIx64 ") failed: %s",
                     (uint64_t)ext1_mask, esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "EXT1 wake enabled mask=0x%" PRIx64, (uint64_t)ext1_mask);
        }
    }
}

static inline bool app_gpio_active_low(gpio_num_t gpio)
{
    return gpio_get_level(gpio) == 0;
}

/* GPIO wakeup is level-triggered; skip light sleep if any wake pin is already held low
 * to avoid immediate wake loops.
 */
static bool app_any_wakeup_pin_asserted(const char **out_reason)
{
    const char *reason = NULL;
    if (app_gpio_active_low((gpio_num_t)CONFIG_PULSE_GPIO)) {
        reason = "pulse_gpio";
#if CONFIG_FACTORY_RESET_BUTTON_GPIO >= 0
    } else if (CONFIG_FACTORY_RESET_BUTTON_GPIO != CONFIG_PULSE_GPIO &&
               app_gpio_active_low((gpio_num_t)CONFIG_FACTORY_RESET_BUTTON_GPIO)) {
        reason = "factory_reset_gpio";
#endif
    }

    if (out_reason) {
        *out_reason = reason;
    }
    return reason != NULL;
}

static void app_steer_retry_timer_cb(void *arg)
{
    s_request_steer = true;
}

static void app_schedule_steer_retry(const char *reason)
{
#if APP_STEER_MAX_RETRIES > 0
    if (s_steer_total_attempts >= (uint32_t)APP_STEER_MAX_RETRIES) {
        if (APP_STEER_COOLDOWN_S > 0) {
            ESP_LOGW(TAG,
                     "Steering attempts exhausted (%u/%u). Cooldown %u s then retry burst (%s)",
                     (unsigned)s_steer_total_attempts,
                     (unsigned)APP_STEER_MAX_RETRIES,
                     (unsigned)APP_STEER_COOLDOWN_S,
                     reason ? reason : "n/a");
            s_steer_retry_count = 0;
            s_steer_started = false;
            s_request_steer = false;
            s_joined = false;
            esp_timer_stop(s_steer_retry_timer);
            esp_timer_start_once(s_steer_retry_timer, (uint64_t)APP_STEER_COOLDOWN_S * 1000000ULL);
        } else {
            ESP_LOGW(TAG,
                     "Steering attempts exhausted (%u/%u). Stop auto-steering until reboot/reset (%s)",
                     (unsigned)s_steer_total_attempts,
                     (unsigned)APP_STEER_MAX_RETRIES,
                     reason ? reason : "n/a");
            s_request_steer = false;
            s_steer_started = false;
            esp_timer_stop(s_steer_retry_timer);
        }
        return;
    }
#endif

    uint32_t delay_s = APP_STEER_RETRY_BASE_S;
    if (s_steer_retry_count == 0) {
        delay_s = 5;
    } else if (s_steer_retry_count == 1) {
        delay_s = 10;
    } else if (s_steer_retry_count == 2) {
        delay_s = 20;
    } else {
        delay_s = 40;
    }
    if (delay_s > APP_STEER_RETRY_MAX_S) {
        delay_s = APP_STEER_RETRY_MAX_S;
    }
    s_steer_retry_count++;
    ESP_LOGW(TAG, "Scheduling network steering retry in %u s (%s)", delay_s, reason);
    s_steer_started = false;
    s_joined = false;
    esp_timer_stop(s_steer_retry_timer);
    esp_timer_start_once(s_steer_retry_timer, (uint64_t)delay_s * 1000000ULL);
}

static void app_zigbee_set_tx_power(int8_t target_dbm, const char *context)
{
    int8_t before = 0;
    int8_t after = 0;
    int8_t clamped = target_dbm;
    if (clamped > IEEE802154_TXPOWER_VALUE_MAX) {
        clamped = IEEE802154_TXPOWER_VALUE_MAX;
    } else if (clamped < IEEE802154_TXPOWER_VALUE_MIN) {
        clamped = IEEE802154_TXPOWER_VALUE_MIN;
    }
    esp_zb_get_tx_power(&before);
    esp_zb_set_tx_power(clamped);
    esp_zb_get_tx_power(&after);
    ESP_LOGI(TAG, "TX power set to %d dBm (before %d dBm after %d dBm, %s)",
             clamped, before, after, context);
}

static void app_start_network_steering(const char *context)
{
    if (s_steer_started) {
        return;
    }
#if APP_STEER_MAX_RETRIES > 0
    if (s_steer_total_attempts >= (uint32_t)APP_STEER_MAX_RETRIES) {
        ESP_LOGW(TAG, "Steering suppressed (attempt limit reached %u/%u) (%s)",
                 (unsigned)s_steer_total_attempts, (unsigned)APP_STEER_MAX_RETRIES,
                 context ? context : "n/a");
        return;
    }
#endif
    s_steer_started = true;
    s_joined = false;
    app_log_commissioning_state(context);
    ESP_LOGI(TAG, "Starting network steering (channel mask 0x%08x, tx_power %d dBm)",
             CONFIG_ZB_CHANNEL_MASK, APP_ZB_TX_POWER_DBM);
    app_zigbee_set_tx_power(APP_ZB_TX_POWER_JOIN_DBM, "steering");
    s_steer_total_attempts++;
    if (APP_STEER_MAX_RETRIES > 0) {
        ESP_LOGI(TAG, "Steering attempt %u/%u (%s)",
                 (unsigned)s_steer_total_attempts, (unsigned)APP_STEER_MAX_RETRIES,
                 context ? context : "n/a");
    } else {
        ESP_LOGI(TAG, "Steering attempt %u (unlimited) (%s)",
                 (unsigned)s_steer_total_attempts, context ? context : "n/a");
    }
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

static void app_log_network_info(const char *context)
{
    uint16_t short_addr = esp_zb_get_short_address();
    uint16_t pan_id = esp_zb_get_pan_id();
    uint8_t channel = esp_zb_get_current_channel();
    esp_zb_ieee_addr_t ieee = {0};
    esp_zb_ieee_addr_t extpan = {0};
    esp_zb_get_long_address(ieee);
    esp_zb_get_extended_pan_id(extpan);

    ESP_LOGI(TAG,
             "%s: short=0x%04x pan=0x%04x ch=%u "
             "ieee=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x "
             "extpan=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             context, short_addr, pan_id, channel,
             ieee[7], ieee[6], ieee[5], ieee[4], ieee[3], ieee[2], ieee[1], ieee[0],
             extpan[7], extpan[6], extpan[5], extpan[4], extpan[3], extpan[2], extpan[1], extpan[0]);
}

static void app_handle_pending_pulses(void)
{
    pulse_pending_info_t pending = {0};
    if (!pulse_take_pending(&pending)) {
        return;
    }

    if (pending.lost > 0) {
        ESP_LOGW(TAG, "Dropped %u pulses while pending queue was full", (unsigned)pending.lost);
    }

    if (pending.count == 0) {
        return;
    }

    metering_on_pulses(pending.count, pending.last_ts_us, pending.prev_ts_us);
    uint64_t total = pulse_get_total();
    ESP_LOGI(TAG, "Pulse counted: +%u total=%llu", (unsigned)pending.count, (unsigned long long)total);
    app_zigbee_update_metering_attrs_dynamic();
    s_total_dirty = true;
}

static void app_power_status_cb(const power_status_t *status, void *ctx)
{
    (void)ctx;

    app_event_t evt = {
        .type = APP_EVENT_BATTERY,
    };
    if (status) {
        evt.power = *status;
    }
    app_event_send(&evt);
}

static void app_zigbee_update_metering_attrs_static(void)
{
    /* These rarely/never change; set once on join/start to reduce ZCL work per pulse. */
    uint8_t unit = metering_get_unit_of_measure();
    uint8_t device_type = metering_get_device_type();
    uint8_t formatting = metering_get_summation_formatting();
    uint8_t demand_formatting = metering_get_demand_formatting();
    uint32_t multiplier = metering_get_multiplier();
    uint32_t divisor = metering_get_divisor();
    s_attr_unit = unit;
    s_attr_summation_formatting = formatting;
    s_attr_demand_formatting = demand_formatting;
    s_attr_device_type = device_type;
    s_attr_multiplier = app_to_uint24(multiplier);
    s_attr_divisor = app_to_uint24(divisor);
    ESP_LOGI(TAG, "Metering scale: unit=%u device_type=%u mult=%u div=%u fmt=0x%02x demand_fmt=0x%02x",
             (unsigned)unit, (unsigned)device_type, (unsigned)multiplier, (unsigned)divisor,
             (unsigned)formatting, (unsigned)demand_formatting);

    esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_METERING_UNIT_OF_MEASURE_ID,
                                 &s_attr_unit, false);

    esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_METERING_SUMMATION_FORMATTING_ID,
                                 &s_attr_summation_formatting, false);

    esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_METERING_DEMAND_FORMATTING_ID,
                                 &s_attr_demand_formatting, false);

    esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_METERING_METERING_DEVICE_TYPE_ID,
                                 &s_attr_device_type, false);

    esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_METERING_MULTIPLIER_ID,
                                 &s_attr_multiplier, false);

    esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_METERING_DIVISOR_ID,
                                 &s_attr_divisor, false);
}

static void app_zigbee_update_metering_attrs_dynamic(void)
{
    /* These change on pulses / demand decay. */
    uint64_t summation = metering_get_summation();
    int32_t demand = metering_get_instantaneous_demand();
    esp_zb_uint48_t summation_val = app_to_uint48(summation);
    esp_zb_int24_t demand_val = app_to_int24(demand);

    esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID,
                                 &summation_val, false);

    esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID,
                                 &demand_val, false);
}

static void app_zigbee_update_power_attrs(const power_status_t *status)
{
    uint8_t voltage = status->battery_voltage_attr;
    uint8_t percent = status->battery_percent_attr;
    uint8_t power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY;

    if (voltage != 0xFF) {
        if (s_last_battery_voltage == 0xFF || voltage != s_last_battery_voltage) {
            esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                         ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                         ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
                                         &voltage, false);
            s_last_battery_voltage = voltage;
        }
    }

    esp_zb_set_node_descriptor_power_source(false);
    esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID,
                                 &power_source, false);

    if (percent != 0xFF) {
        if (s_last_battery_percent == 0xFF ||
            (percent > s_last_battery_percent + (CONFIG_BATTERY_REPORT_HYST_PCT * 2)) ||
            (percent + (CONFIG_BATTERY_REPORT_HYST_PCT * 2) < s_last_battery_percent)) {
            esp_zb_zcl_set_attribute_val(APP_ZB_ENDPOINT,
                                         ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                         ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                         &percent, false);
            s_last_battery_percent = percent;
        }
    }
}

static void app_log_power_status(const power_status_t *status, const char *context)
{
    if (!status) {
        return;
    }

    ESP_LOGI(TAG, "%s power: battery_mv=%u voltage_attr=0x%02x percent_attr=0x%02x",
             context ? context : "Power",
             (unsigned)status->battery_mv,
             (unsigned)status->battery_voltage_attr,
             (unsigned)status->battery_percent_attr);
}

static void app_log_wakeup_info(int64_t slept_ms)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    esp_sleep_source_t bitmap = esp_sleep_get_wakeup_causes();

    ESP_LOGI(TAG, "Wakeup: slept~%lld ms cause=%d bitmap=0x%" PRIx64,
             (long long)slept_ms, (int)cause, (uint64_t)bitmap);
}

static void app_update_sleep_policy(void)
{
#if CONFIG_SLEEPY_END_DEVICE
    /* IMPORTANT:
     * RxOnWhenIdle is used during association/join. Joining with RxOnWhenIdle=1 makes the
     * device "non-sleepy" from Zigbee PoV, and CAN_SLEEP may never be produced later.
     *
     * For a sleepy end device build, keep RxOnWhenIdle disabled for the whole life of the node.
     */
    (void)esp_zb_set_rx_on_when_idle(false);

    (void)esp_zb_sleep_set_threshold(APP_ZB_SLEEP_THRESHOLD_MS);
    (void)esp_zb_sleep_enable(true);
#endif
}

static void app_zigbee_configure_reporting(void);

static void app_zcl_send_status_cb(esp_zb_zcl_command_send_status_message_t message)
{
    uint16_t short_addr = 0xFFFF;
    if (message.dst_addr.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT) {
        short_addr = message.dst_addr.u.short_addr;
    }

    ESP_LOGI(TAG, "ZCL send status: tsn %u dst 0x%04x status %s (0x%x)",
             message.tsn, short_addr, esp_err_to_name(message.status), message.status);
}

static esp_err_t app_core_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        if (message) {
            const esp_zb_zcl_set_attr_value_message_t *m = (const esp_zb_zcl_set_attr_value_message_t *)message;
            uint16_t cluster = m->info.cluster;
            uint16_t attr = m->attribute.id;
            if (cluster == APP_MFG_CLUSTER_ID && attr == 0x0008) {
                ESP_LOGI(TAG, "Reset requested via core action callback");
                app_config_reset_counter_request();
            }
        }
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        if (message) {
            return app_ota_upgrade_status_handler(*(const esp_zb_zcl_ota_upgrade_value_message_t *)message);
        }
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
        if (message) {
            return app_ota_query_image_resp_handler(*(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void app_configure_attr_reporting(const esp_zb_zcl_reporting_info_t *info, const char *label)
{
    esp_zb_zcl_attr_location_info_t attr_info = {
        .endpoint_id = info->ep,
        .cluster_id = info->cluster_id,
        .cluster_role = info->cluster_role,
        .manuf_code = info->manuf_code,
        .attr_id = info->attr_id,
    };
    esp_err_t start_err = esp_zb_zcl_start_attr_reporting(attr_info);
    if (start_err != ESP_OK && start_err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Reporting start failed: %s cluster 0x%04x attr 0x%04x err %s (0x%x)",
                 label, attr_info.cluster_id, attr_info.attr_id, esp_err_to_name(start_err), start_err);
    }

    esp_zb_zcl_reporting_info_t tmp = *info;
    esp_err_t update_err = esp_zb_zcl_update_reporting_info(&tmp);
    if (update_err != ESP_OK && update_err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Reporting update failed: %s cluster 0x%04x attr 0x%04x err %s (0x%x)",
                 label, info->cluster_id, info->attr_id, esp_err_to_name(update_err), update_err);
    }

    if ((start_err == ESP_OK || start_err == ESP_ERR_NOT_FOUND) &&
        (update_err == ESP_OK || update_err == ESP_ERR_NOT_FOUND)) {
        ESP_LOGI(TAG, "Reporting configured: %s cluster 0x%04x attr 0x%04x",
                 label, info->cluster_id, info->attr_id);
    }
}

typedef struct {
    esp_zb_zdo_bind_req_param_t req;
    uint16_t cluster_id;
    const char *label;
} app_bind_ctx_t;

static void app_bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    app_bind_ctx_t *ctx = (app_bind_ctx_t *)user_ctx;
    ESP_LOGI(TAG, "Bind %s (cluster 0x%04x) status %d", ctx->label, ctx->cluster_id, (int)zdo_status);
    free(ctx);
}

static void app_bind_cluster(uint16_t cluster_id, const char *label)
{
    const uint16_t dst_short = APP_ZB_REPORT_DST_SHORT_ADDR;
    if (dst_short == 0xFFFF) {
        ESP_LOGW(TAG, "Bind skipped for %s: dst short is 0xFFFF", label);
        return;
    }

    app_bind_ctx_t *ctx = (app_bind_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx) {
        ESP_LOGE(TAG, "No mem for bind ctx (%s)", label);
        return;
    }
    ctx->cluster_id = cluster_id;
    ctx->label = label;

    ctx->req.req_dst_addr = esp_zb_get_short_address();
    ctx->req.src_endp = APP_ZB_ENDPOINT;
    ctx->req.dst_endp = APP_ZB_REPORT_DST_ENDPOINT;
    ctx->req.cluster_id = cluster_id;
    ctx->req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;

    esp_zb_get_long_address(ctx->req.src_address);
    esp_zb_ieee_address_by_short(dst_short, ctx->req.dst_address_u.addr_long);

    esp_zb_zdo_device_bind_req(&ctx->req, app_bind_cb, ctx);
}

static void app_zigbee_configure_reporting(void)
{
    uint16_t min_interval = CONFIG_ZB_REPORT_MIN_S;
    uint16_t max_interval = CONFIG_ZB_REPORT_MAX_S;
    uint32_t change = CONFIG_ZB_REPORTABLE_CHANGE;

    ESP_LOGI(TAG, "Configure reporting: metering min %u max %u change %u",
             min_interval, max_interval, change);
    if (change == 0) {
        ESP_LOGW(TAG, "CONFIG_ZB_REPORTABLE_CHANGE=0 may cause frequent reports (min interval=%u s)",
                 (unsigned)min_interval);
    }

    esp_zb_zcl_reporting_info_t metering_info = {0};
    metering_info.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
    metering_info.ep = APP_ZB_ENDPOINT;
    metering_info.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_METERING;
    metering_info.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    metering_info.attr_id = ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID;
    metering_info.u.send_info.min_interval = min_interval;
    metering_info.u.send_info.max_interval = max_interval;
    metering_info.u.send_info.delta.u48 = app_to_uint48(change);
    metering_info.u.send_info.def_min_interval = min_interval;
    metering_info.u.send_info.def_max_interval = max_interval;

    esp_zb_zcl_reporting_info_t battery_info = {0};
    battery_info.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
    battery_info.ep = APP_ZB_ENDPOINT;
    battery_info.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
    battery_info.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    battery_info.attr_id = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
    battery_info.u.send_info.min_interval = CONFIG_ZB_BAT_REPORT_MIN_S;
    battery_info.u.send_info.max_interval = CONFIG_ZB_BAT_REPORT_MAX_S;
    battery_info.u.send_info.delta.u8 = CONFIG_ZB_BAT_REPORTABLE_CHANGE;
    battery_info.u.send_info.def_min_interval = CONFIG_ZB_BAT_REPORT_MIN_S;
    battery_info.u.send_info.def_max_interval = CONFIG_ZB_BAT_REPORT_MAX_S;

    esp_zb_zcl_reporting_info_t battery_voltage_info = {0};
    battery_voltage_info.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
    battery_voltage_info.ep = APP_ZB_ENDPOINT;
    battery_voltage_info.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
    battery_voltage_info.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    battery_voltage_info.attr_id = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID;
    battery_voltage_info.u.send_info.min_interval = CONFIG_ZB_BAT_REPORT_MIN_S;
    battery_voltage_info.u.send_info.max_interval = CONFIG_ZB_BAT_REPORT_MAX_S;
    battery_voltage_info.u.send_info.delta.u8 = 0; /* report on min/max */
    battery_voltage_info.u.send_info.def_min_interval = CONFIG_ZB_BAT_REPORT_MIN_S;
    battery_voltage_info.u.send_info.def_max_interval = CONFIG_ZB_BAT_REPORT_MAX_S;

    esp_zb_zcl_reporting_info_t demand_info = {0};
    demand_info.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
    demand_info.ep = APP_ZB_ENDPOINT;
    demand_info.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_METERING;
    demand_info.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    demand_info.attr_id = ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID;
    demand_info.u.send_info.min_interval = min_interval;
    demand_info.u.send_info.max_interval = max_interval;
    demand_info.u.send_info.delta.s24 = app_to_int24((int32_t)change);
    demand_info.u.send_info.def_min_interval = min_interval;
    demand_info.u.send_info.def_max_interval = max_interval;

    app_configure_attr_reporting(&metering_info, "metering");
    app_configure_attr_reporting(&battery_info, "battery");
    app_configure_attr_reporting(&battery_voltage_info, "battery_voltage");
    app_configure_attr_reporting(&demand_info, "demand");
}

static void app_on_joined(const char *reason)
{
    ESP_LOGI(TAG, "Joined network (%s)", reason ? reason : "n/a");
    s_joined = true;
    s_steer_started = false;
    s_steer_retry_count = 0;
    s_steer_total_attempts = 0;
    s_request_steer = false;
    esp_timer_stop(s_steer_retry_timer);
    app_zigbee_set_tx_power(APP_ZB_TX_POWER_DBM, "joined (after-join power)");

    power_status_t joined_power = {0};
    power_read_status(&joined_power);

    s_no_sleep_until_us = esp_timer_get_time() + APP_SLEEP_JOIN_BLOCK_US;
    app_update_sleep_policy();
    app_zigbee_update_metering_attrs_static();
    app_zigbee_update_metering_attrs_dynamic();
    app_zigbee_update_power_attrs(&joined_power);
    app_zigbee_configure_reporting();
    app_bind_cluster(ESP_ZB_ZCL_CLUSTER_ID_METERING, "seMetering");
    app_bind_cluster(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, "genPowerCfg");

    app_log_network_info("Joined info");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    esp_zb_app_signal_type_t sig_type = *(esp_zb_app_signal_type_t *)signal_struct->p_app_signal;
    esp_err_t status = signal_struct->esp_err_status;

    if (sig_type != ESP_ZB_COMMON_SIGNAL_CAN_SLEEP) {
        ESP_LOGI(TAG, "Zigbee signal %d (%s) status %s (0x%x)", sig_type, app_signal_to_str(sig_type),
                 esp_err_to_name(status), status);
    }

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_ERROR:
        app_log_commissioning_state("ZDO error");
        break;
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized (autostart)");
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        if (status == ESP_OK) {
            bool factory_new = esp_zb_bdb_is_factory_new();
            bool joined = esp_zb_bdb_dev_joined();
            ESP_LOGI(TAG, "First start: factory_new=%d joined=%d", factory_new, joined);
            if (joined) {
                /* The stack may report joined=true on DEVICE_FIRST_START after reboot.
                 * Without this, s_joined stays false and sleep never enables.
                 */
                app_on_joined("DEVICE_FIRST_START (already joined)");
            } else {
                app_start_network_steering(factory_new ? "Device first start (factory-new)"
                                                       : "Device first start (not joined)");
            }
        } else {
            ESP_LOGW(TAG, "Device first start failed: %s (0x%x)", esp_err_to_name(status), status);
            app_log_commissioning_state("Device first start failed");
            app_schedule_steer_retry("device first start failed");
        }
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (status == ESP_OK) {
            app_on_joined("DEVICE_REBOOT");
        } else {
            ESP_LOGW(TAG, "Zigbee join failed: %s (0x%x)", esp_err_to_name(status), status);
            app_log_commissioning_state("Join failed");
            app_schedule_steer_retry("join failed");
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (status == ESP_OK) {
            app_on_joined("BDB_STEERING");
        } else {
            ESP_LOGW(TAG, "Network steering failed: %s (0x%x)", esp_err_to_name(status), status);
            app_log_commissioning_state("Steering failed");
            app_schedule_steer_retry("steering failed");
            s_steer_started = false;
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        ESP_LOGI(TAG, "Production config status: %s (0x%x)", esp_err_to_name(status), status);
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        /* Zigbee light sleep: enter only on battery when wakeup pins are idle. */
#if CONFIG_SLEEPY_END_DEVICE
        {
            const char *wake_reason = NULL;
            bool wake_low = app_any_wakeup_pin_asserted(&wake_reason);
            int64_t now_us = esp_timer_get_time();

            if (!s_joined || wake_low) {
                if (now_us - s_last_can_sleep_skip_log_us > 5000000LL) {
                    ESP_LOGI(TAG, "CAN_SLEEP skipped: joined=%d wake_low=%d reason=%s",
                             s_joined, wake_low, wake_reason ? wake_reason : "none");
                    s_last_can_sleep_skip_log_us = now_us;
                }
                break;
            }

            if (now_us < s_no_sleep_until_us) {
                if (now_us - s_last_can_sleep_skip_log_us > 5000000LL) {
                    ESP_LOGI(TAG, "CAN_SLEEP skipped: post-join block active (%lld ms left)",
                             (long long)((s_no_sleep_until_us - now_us) / 1000LL));
                    s_last_can_sleep_skip_log_us = now_us;
                }
                break;
            }

            int64_t t0 = now_us;
            esp_zb_sleep_now();
            int64_t slept_ms = (esp_timer_get_time() - t0) / 1000;
            app_log_wakeup_info(slept_ms);

            if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
                uint64_t ext1_status = esp_sleep_get_ext1_wakeup_status();
                if (ext1_status & (1ULL << CONFIG_PULSE_GPIO)) {
                    int64_t now2 = esp_timer_get_time();
                    bool counted = pulse_record_wakeup(now2);
                    ESP_LOGI(TAG, "EXT1 wake on pulse GPIO%d (status=0x%" PRIx64 "): %s",
                             CONFIG_PULSE_GPIO, (uint64_t)ext1_status, counted ? "counted" : "ignored");
                }
            }
        }
#endif
        break;
    default:
        break;
    }
}

static void app_zigbee_init(void)
{
    esp_zb_platform_config_t platform_config = {0};
    platform_config.radio_config.radio_mode = ZB_RADIO_MODE_NATIVE;
    platform_config.host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE;
    esp_zb_platform_config(&platform_config);

    esp_zb_cfg_t zb_nwk_cfg = {0};
    zb_nwk_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_ED;
#ifdef CONFIG_ZB_INSTALL_CODE_POLICY
    zb_nwk_cfg.install_code_policy = true;
#else
    zb_nwk_cfg.install_code_policy = false;
#endif
    zb_nwk_cfg.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN;
    zb_nwk_cfg.nwk_cfg.zed_cfg.keep_alive = CONFIG_ZB_KEEP_ALIVE_MS;
    esp_zb_init(&zb_nwk_cfg);
#if CONFIG_ESP_ZB_TRACE_ENABLE
    esp_zb_set_trace_level_mask((esp_zb_trace_level_cfg_t)CONFIG_ZB_TRACE_LEVEL, CONFIG_ZB_TRACE_MASK);
#endif
    app_zigbee_set_tx_power(APP_ZB_TX_POWER_DBM, "init");
    esp_zb_secur_network_min_join_lqi_set(CONFIG_ZB_MIN_JOIN_LQI);

    esp_zb_set_primary_network_channel_set(CONFIG_ZB_CHANNEL_MASK);
    esp_zb_bdb_set_scan_duration(CONFIG_ZB_BDB_SCAN_DURATION);
    if (CONFIG_ZB_SECONDARY_CHANNEL_MASK != 0) {
        esp_zb_set_secondary_network_channel_set(CONFIG_ZB_SECONDARY_CHANNEL_MASK);
    }

    s_zb_manufacturer_name[0] = APP_ZCL_STR_LEN(APP_MANUFACTURER_NAME);
    memcpy(&s_zb_manufacturer_name[1], APP_MANUFACTURER_NAME, APP_ZCL_STR_LEN(APP_MANUFACTURER_NAME));
    s_zb_model_identifier[0] = APP_ZCL_STR_LEN(APP_MODEL_IDENTIFIER);
    memcpy(&s_zb_model_identifier[1], APP_MODEL_IDENTIFIER, APP_ZCL_STR_LEN(APP_MODEL_IDENTIFIER));
    app_update_sw_build_id();

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
    };
    esp_zb_attribute_list_t *basic_attr_list = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                            ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            s_zb_manufacturer_name);
    esp_zb_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                            ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            s_zb_model_identifier);
    esp_zb_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                            ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            s_zb_sw_build_id);

    esp_zb_power_config_cluster_cfg_t power_cfg = {
        .main_voltage = 0,
        .main_freq = 0,
        .main_alarm_mask = 0,
        .main_voltage_min = 0,
        .main_voltage_max = 0,
        .main_voltage_dwell = 0,
    };
    esp_zb_attribute_list_t *power_attr_list = esp_zb_power_config_cluster_create(&power_cfg);
    s_attr_battery_voltage = 0xFF;
    s_attr_battery_percent = 0xFF;
    esp_zb_cluster_add_attr(power_attr_list, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, ESP_ZB_ZCL_ATTR_TYPE_U8,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                            &s_attr_battery_voltage);
    esp_zb_cluster_add_attr(power_attr_list, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, ESP_ZB_ZCL_ATTR_TYPE_U8,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                            &s_attr_battery_percent);

    esp_zb_attribute_list_t *metering_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_METERING);
    s_attr_summation = app_to_uint48(metering_get_summation());
    s_attr_unit = s_cfg.unit_of_measure;
    s_attr_summation_formatting = metering_get_summation_formatting();
    s_attr_demand_formatting = metering_get_demand_formatting();
    s_attr_device_type = s_cfg.metering_device_type;
    s_attr_multiplier = app_to_uint24(metering_get_multiplier());
    s_attr_divisor = app_to_uint24(metering_get_divisor());
    s_attr_demand = app_to_int24(0);

    esp_zb_cluster_add_attr(metering_attr_list, ESP_ZB_ZCL_CLUSTER_ID_METERING,
                            ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U48,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                            &s_attr_summation);
    esp_zb_cluster_add_attr(metering_attr_list, ESP_ZB_ZCL_CLUSTER_ID_METERING,
                            ESP_ZB_ZCL_ATTR_METERING_UNIT_OF_MEASURE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &s_attr_unit);
    esp_zb_cluster_add_attr(metering_attr_list, ESP_ZB_ZCL_CLUSTER_ID_METERING,
                            ESP_ZB_ZCL_ATTR_METERING_SUMMATION_FORMATTING_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BITMAP,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &s_attr_summation_formatting);
    esp_zb_cluster_add_attr(metering_attr_list, ESP_ZB_ZCL_CLUSTER_ID_METERING,
                            ESP_ZB_ZCL_ATTR_METERING_DEMAND_FORMATTING_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BITMAP,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &s_attr_demand_formatting);
    esp_zb_cluster_add_attr(metering_attr_list, ESP_ZB_ZCL_CLUSTER_ID_METERING,
                            ESP_ZB_ZCL_ATTR_METERING_METERING_DEVICE_TYPE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &s_attr_device_type);
    esp_zb_cluster_add_attr(metering_attr_list, ESP_ZB_ZCL_CLUSTER_ID_METERING,
                            ESP_ZB_ZCL_ATTR_METERING_MULTIPLIER_ID, ESP_ZB_ZCL_ATTR_TYPE_U24,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &s_attr_multiplier);
    esp_zb_cluster_add_attr(metering_attr_list, ESP_ZB_ZCL_CLUSTER_ID_METERING,
                            ESP_ZB_ZCL_ATTR_METERING_DIVISOR_ID, ESP_ZB_ZCL_ATTR_TYPE_U24,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &s_attr_divisor);
    esp_zb_cluster_add_attr(metering_attr_list, ESP_ZB_ZCL_CLUSTER_ID_METERING,
                            ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_S24,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                            &s_attr_demand);

    esp_zb_ieee_addr_t ota_server_id = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_DEF_VALUE;
    esp_zb_ota_cluster_cfg_t ota_cfg = {
        .ota_upgrade_file_version = CONFIG_ZB_OTA_FILE_VERSION,
        .ota_upgrade_manufacturer = APP_MFG_CODE,
        .ota_upgrade_image_type = CONFIG_ZB_OTA_IMAGE_TYPE,
        .ota_min_block_reque = ESP_ZB_OTA_UPGRADE_MIN_BLOCK_PERIOD_DEF_VALUE,
        .ota_upgrade_file_offset = ESP_ZB_ZCL_OTA_UPGRADE_FILE_OFFSET_DEF_VALUE,
        .ota_upgrade_downloaded_file_ver = CONFIG_ZB_OTA_FILE_VERSION,
        .ota_upgrade_server_id = {0},
        .ota_image_upgrade_status = ESP_ZB_ZCL_OTA_UPGRADE_IMAGE_STATUS_DEF_VALUE,
    };
    memcpy(ota_cfg.ota_upgrade_server_id, ota_server_id, sizeof(ota_server_id));

    esp_zb_attribute_list_t *ota_attr_list = esp_zb_ota_cluster_create(&ota_cfg);
    s_attr_ota_stack_version = ESP_ZB_ZCL_OTA_UPGRADE_STACK_VERSION_DEF_VALUE;
    s_attr_ota_downloaded_stack_version = ESP_ZB_ZCL_OTA_UPGRADE_DOWNLOADED_STACK_DEF_VALUE;
    s_attr_ota_image_stamp = CONFIG_ZB_OTA_FILE_VERSION;
    s_attr_ota_server_addr = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_ADDR_DEF_VALUE;
    s_attr_ota_server_ep = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_ENDPOINT_DEF_VALUE;
    s_attr_ota_client_var = (esp_zb_zcl_ota_upgrade_client_variable_t){
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = 0x0001,
        .max_data_size = 64,
    };
    esp_zb_ota_cluster_add_attr(ota_attr_list, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_STACK_VERSION_ID, &s_attr_ota_stack_version);
    esp_zb_ota_cluster_add_attr(ota_attr_list, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_DOWNLOADED_STACK_VERSION_ID,
                                &s_attr_ota_downloaded_stack_version);
    esp_zb_ota_cluster_add_attr(ota_attr_list, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_IMAGE_STAMP_ID, &s_attr_ota_image_stamp);
    esp_zb_ota_cluster_add_attr(ota_attr_list, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, &s_attr_ota_server_addr);
    esp_zb_ota_cluster_add_attr(ota_attr_list, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, &s_attr_ota_server_ep);
    esp_zb_ota_cluster_add_attr(ota_attr_list, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &s_attr_ota_client_var);

    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_power_config_cluster(cluster_list, power_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_metering_cluster(cluster_list, metering_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_ota_cluster(cluster_list, ota_attr_list, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    config_cluster_add(cluster_list, &s_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_cfg = {
        .endpoint = APP_ZB_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = APP_ZB_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_cfg);

    esp_zb_device_register(ep_list);

    ota_init();
    config_cluster_register_callbacks();

    app_log_commissioning_state("Before steering start");
}

static void zigbee_task(void *arg)
{
    app_zigbee_init();

    /* Startup factory reset: hold the button during boot. */
#if CONFIG_FACTORY_RESET_BUTTON_GPIO >= 0
    if (gpio_get_level(CONFIG_FACTORY_RESET_BUTTON_GPIO) == 0) {
        ESP_LOGW(TAG, "Factory reset button held at boot, waiting %u ms...",
                 (unsigned)APP_FACTORY_RESET_HOLD_MS);
        int64_t start = esp_timer_get_time();
        while (gpio_get_level(CONFIG_FACTORY_RESET_BUTTON_GPIO) == 0 &&
               (esp_timer_get_time() - start) < (int64_t)APP_FACTORY_RESET_HOLD_US) {
            vTaskDelay(pdMS_TO_TICKS(APP_FACTORY_RESET_POLL_MS));
        }
        if (gpio_get_level(CONFIG_FACTORY_RESET_BUTTON_GPIO) == 0) {
            s_factory_reset_requested = true;
        }
    }
#endif

    if (s_factory_reset_requested) {
        s_factory_reset_requested = false;
        app_handle_factory_reset_request("button long press (startup)");
    }

    esp_zb_core_action_handler_register(app_core_action_handler);
    esp_zb_zcl_command_send_status_handler_register(app_zcl_send_status_cb);
    ESP_LOGI(TAG, "Starting Zigbee stack (autostart=true)");
    esp_zb_start(true);

    power_status_t initial_power = {0};
    power_read_status(&initial_power);
    app_log_power_status(&initial_power, "Startup");
    app_update_sleep_policy();
    app_zigbee_update_power_attrs(&initial_power);
    app_zigbee_update_metering_attrs_static();
    app_zigbee_update_metering_attrs_dynamic();

    while (true) {
        app_factory_reset_button_service();
        if (s_factory_reset_requested) {
            s_factory_reset_requested = false;
            app_handle_factory_reset_request("button long press");
        }

        esp_zb_stack_main_loop_iteration();

        ulTaskNotifyTake(pdTRUE, 0);
        app_handle_pending_pulses();

        app_event_t evt;
        while (xQueueReceive(s_app_event_queue, &evt, 0) == pdTRUE) {
            if (evt.type == APP_EVENT_BATTERY) {
                app_zigbee_update_power_attrs(&evt.power);
            }
        }

        int64_t now = esp_timer_get_time();
        if (s_total_dirty &&
            ((now - s_last_save_us) >= (int64_t)APP_SAVE_INTERVAL_US ||
             (metering_get_last_pulse_us() > 0 &&
              (now - metering_get_last_pulse_us()) >= (int64_t)APP_SAVE_DEBOUNCE_US))) {
            app_pulse_save_total(metering_get_total_pulses());
            s_last_save_us = now;
            s_total_dirty = false;
        }

        if (now - s_last_demand_check_us >= APP_DEMAND_IDLE_CHECK_US) {
            s_last_demand_check_us = now;
            if (metering_tick(now)) {
                app_zigbee_update_metering_attrs_dynamic();
            }
        }

        if (s_request_steer) {
            s_request_steer = false;
            app_log_commissioning_state("Retry steering");
            ESP_LOGI(TAG, "Retrying network steering (attempt %u)", s_steer_retry_count);
            app_start_network_steering("Retry steering");
        }

        config_cluster_apply_pending(&s_cfg);
        if (app_config_consume_reset_request()) {
            ESP_LOGI(TAG, "Resetting metering and pulse counters");
            metering_reset();
            metering_set_instantaneous_demand(0);
            pulse_set_total(0);
            app_pulse_save_total(0);
            s_total_dirty = false;
            s_last_save_us = esp_timer_get_time();
            app_zigbee_update_metering_attrs_dynamic();
            power_status_t current_power = {0};
            power_read_status(&current_power);
            app_zigbee_update_power_attrs(&current_power);
        }

        /* Yield briefly so we do not busy-spin if the Zigbee loop does not block internally. */
        vTaskDelay(1);
    }
}

void app_main(void)
{
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS init requires erase (%s), erasing...", esp_err_to_name(nvs_err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

#if CONFIG_PM_ENABLE
    /* Enable DFS + (optional) automatic light sleep in idle.
     * Zigbee stack typically cooperates via PM locks; if something blocks sleep,
     * this helps highlight it (and still saves active current via DFS).
     */
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        /* Keep min same as max to avoid invalid values on this target; adjust via menuconfig if tested. */
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    esp_err_t pm_err = esp_pm_configure(&pm_cfg);
    if (pm_err != ESP_OK) {
        ESP_LOGW(TAG, "esp_pm_configure failed: %s", esp_err_to_name(pm_err));
    }
#endif

    app_config_load(&s_cfg);
    uint64_t total_pulses = 0;
    app_pulse_load_total(&total_pulses);

    metering_init(&s_cfg, total_pulses);
    ESP_LOGI(TAG, "Meter scaling: pulses_per_unit=%" PRIu32 " divisor=%" PRIu32 " multiplier=%" PRIu32,
             s_cfg.pulse_per_unit_numerator, metering_get_divisor(), metering_get_multiplier());
    s_app_event_queue = xQueueCreate(APP_EVENT_QUEUE_LEN, sizeof(app_event_t));

    pulse_config_t pulse_cfg = {
        .gpio_num = CONFIG_PULSE_GPIO,
        .debounce_ms = s_cfg.debounce_ms,
        .min_width_ms = CONFIG_PULSE_MIN_WIDTH_MS,
    };
    ESP_ERROR_CHECK(pulse_init(&pulse_cfg, NULL, NULL, total_pulses));

    power_init();
    app_factory_reset_button_init();

    app_configure_light_sleep_wakeup_sources();

    s_last_battery_percent = 0xFF;
    s_last_battery_voltage = 0xFF;
    s_last_demand_check_us = 0;
    s_last_save_us = esp_timer_get_time();

    esp_timer_create_args_t retry_timer_args = {
        .callback = app_steer_retry_timer_cb,
        .name = "zb_steer_retry",
    };
    esp_timer_create(&retry_timer_args, &s_steer_retry_timer);

    esp_err_t mon_err = power_start_monitor(APP_BATTERY_TASK_PERIOD_BATT_MS, app_power_status_cb, NULL);
    if (mon_err != ESP_OK) {
        ESP_LOGW(TAG, "power_start_monitor failed: %s", esp_err_to_name(mon_err));
    }

    xTaskCreate(zigbee_task, "zigbee_task", 6144, NULL, 5, &s_zigbee_task_handle);
    pulse_set_consumer_task(s_zigbee_task_handle);
}
