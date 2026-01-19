#include "power.h"

#include "sdkconfig.h"
#include <limits.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#ifndef CONFIG_BATTERY_ADC_ENABLE
#define CONFIG_BATTERY_ADC_ENABLE 0
#endif

#if CONFIG_BATTERY_ADC_ENABLE
static const char *TAG = "power";
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_cali_handle;
static bool s_cali_enabled;
static bool s_adc_ready;
static bool s_warned_no_cali;
static bool s_have_last_good;
static uint16_t s_last_battery_mv;

/* ADC sampling strategy:
 * - High-value dividers mean high source impedance: first samples can be wrong.
 * - Discard a couple of samples, then average calibrated mV readings.
 * - Use a trimmed mean (drop min/max) to suppress spikes.
 */
#define POWER_ADC_DISCARD_SAMPLES          2
#define POWER_ADC_TARGET_SAMPLES           32
#define POWER_ADC_MIN_VALID_SAMPLES        8
#define POWER_ADC_MAX_ATTEMPTS             (POWER_ADC_DISCARD_SAMPLES + POWER_ADC_TARGET_SAMPLES + 16)
#define POWER_ADC_INTER_SAMPLE_DELAY_US    50

static uint16_t calc_battery_mv(uint32_t adc_mv)
{
    uint32_t r_top = CONFIG_BATTERY_RTOP_OHM;
    uint32_t r_bot = CONFIG_BATTERY_RBOT_OHM;

    if (r_bot == 0) {
        return 0;
    }

    uint64_t r_sum = (uint64_t)r_top + (uint64_t)r_bot;
    uint64_t mv = ((uint64_t)adc_mv * r_sum + (uint64_t)r_bot / 2) / (uint64_t)r_bot;
    if (mv > 65000) {
        mv = 65000;
    }
    return (uint16_t)mv;
}

static uint8_t battery_mv_to_zcl_voltage_attr(uint16_t battery_mv)
{
    /* ZCL BatteryVoltage is in 100 mV units; 0xFF means "unknown". */
    uint32_t v_100mv = (uint32_t)battery_mv / 100U;
    if (v_100mv >= 0xFF) {
        v_100mv = 0xFE;
    }
    return (uint8_t)v_100mv;
}

static bool battery_mv_is_sane(uint16_t mv)
{
    /* Reject obvious glitches using a generous window around the configured empty/full range. */
    int32_t empty = CONFIG_BATTERY_EMPTY_MV;
    int32_t full = CONFIG_BATTERY_FULL_MV;
    if (empty <= 0 || full <= 0) {
        return mv > 0;
    }

    int32_t lo = (empty < full) ? empty : full;
    int32_t hi = (empty < full) ? full : empty;
    if (hi <= lo) {
        return mv > 0;
    }

    int32_t span = hi - lo;
    int32_t margin = span;
    if (margin < 500) {
        margin = 500;
    } else if (margin > 5000) {
        margin = 5000;
    }

    int32_t sane_lo = lo - margin;
    int32_t sane_hi = hi + margin;
    if (sane_lo < 0) {
        sane_lo = 0;
    }
    if (sane_hi > 65000) {
        sane_hi = 65000;
    }

    return (mv >= (uint16_t)sane_lo) && (mv <= (uint16_t)sane_hi);
}

static uint8_t calc_battery_percent(uint16_t battery_mv)
{
    int32_t empty_mv = CONFIG_BATTERY_EMPTY_MV;
    int32_t full_mv = CONFIG_BATTERY_FULL_MV;

    if (full_mv <= empty_mv) {
        return 0xFF;
    }

    if (battery_mv <= empty_mv) {
        return 0;
    }

    if (battery_mv >= full_mv) {
        return 200;
    }

    int32_t percent = (battery_mv - empty_mv) * 100 / (full_mv - empty_mv);
    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }

    return (uint8_t)(percent * 2);
}
#endif

static TaskHandle_t s_power_task;
static power_status_cb_t s_power_cb;
static void *s_power_cb_ctx;
static uint32_t s_power_period_ms;

void power_init(void)
{
#if CONFIG_BATTERY_ADC_ENABLE
    adc_unit_t adc_unit = (CONFIG_BATTERY_ADC_UNIT == 2) ? ADC_UNIT_2 : ADC_UNIT_1;
    s_adc_ready = false;
    s_warned_no_cali = false;
    s_have_last_good = false;
    s_last_battery_mv = 0;

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = adc_unit,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = (adc_atten_t)CONFIG_BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, (adc_channel_t)CONFIG_BATTERY_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        return;
    }
    s_adc_ready = true;

    s_cali_enabled = false;
    adc_cali_scheme_ver_t scheme_mask = 0;
    if (adc_cali_check_scheme(&scheme_mask) == ESP_OK) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if (scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING) {
            adc_cali_curve_fitting_config_t cali_cfg = {
                .unit_id = adc_unit,
                .chan = (adc_channel_t)CONFIG_BATTERY_ADC_CHANNEL,
                .atten = (adc_atten_t)CONFIG_BATTERY_ADC_ATTEN,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle) == ESP_OK) {
                s_cali_enabled = true;
            }
        }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if (!s_cali_enabled && (scheme_mask & ADC_CALI_SCHEME_VER_LINE_FITTING)) {
            adc_cali_line_fitting_config_t cali_cfg = {
                .unit_id = adc_unit,
                .atten = (adc_atten_t)CONFIG_BATTERY_ADC_ATTEN,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali_handle) == ESP_OK) {
                s_cali_enabled = true;
            }
        }
#endif
    }
    ESP_LOGI(TAG, "Battery ADC init: unit=%d chan=%d atten=%d cali=%s",
             (int)adc_unit,
             (int)CONFIG_BATTERY_ADC_CHANNEL,
             (int)CONFIG_BATTERY_ADC_ATTEN,
             s_cali_enabled ? "yes" : "no");
#endif

}

void power_read_status(power_status_t *status)
{
    if (!status) {
        return;
    }
    status->battery_mv = 0;
    status->battery_voltage_attr = 0xFF;
    status->battery_percent_attr = 0xFF;

#if CONFIG_BATTERY_ADC_ENABLE
    if (!s_adc_ready) {
        return;
    }

    if (!s_cali_enabled) {
        if (!s_warned_no_cali) {
            s_warned_no_cali = true;
            ESP_LOGW(TAG, "ADC calibration not available; holding last battery voltage");
        }
        if (s_have_last_good) {
            status->battery_mv = s_last_battery_mv;
            status->battery_voltage_attr = battery_mv_to_zcl_voltage_attr(s_last_battery_mv);
            status->battery_percent_attr = calc_battery_percent(s_last_battery_mv);
        }
        return;
    }

    int mv_acc = 0;
    int mv_min = INT_MAX;
    int mv_max = INT_MIN;
    int good = 0;

    for (int i = 0; i < POWER_ADC_MAX_ATTEMPTS && good < POWER_ADC_TARGET_SAMPLES; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc_handle, (adc_channel_t)CONFIG_BATTERY_ADC_CHANNEL, &raw);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(err));
            continue;
        }

        int adc_mv = 0;
        err = adc_cali_raw_to_voltage(s_cali_handle, raw, &adc_mv);
        if (err != ESP_OK || adc_mv < 0) {
            continue;
        }

        if (POWER_ADC_INTER_SAMPLE_DELAY_US > 0) {
            esp_rom_delay_us(POWER_ADC_INTER_SAMPLE_DELAY_US);
        }

        if (i < POWER_ADC_DISCARD_SAMPLES) {
            continue;
        }

        mv_acc += adc_mv;
        if (adc_mv < mv_min) {
            mv_min = adc_mv;
        }
        if (adc_mv > mv_max) {
            mv_max = adc_mv;
        }
        good++;
    }

    if (good < POWER_ADC_MIN_VALID_SAMPLES) {
        if (s_have_last_good) {
            status->battery_mv = s_last_battery_mv;
            status->battery_voltage_attr = battery_mv_to_zcl_voltage_attr(s_last_battery_mv);
            status->battery_percent_attr = calc_battery_percent(s_last_battery_mv);
        }
        return;
    }

    int adc_mv_avg = 0;
    if (good >= 5) {
        adc_mv_avg = (mv_acc - mv_min - mv_max) / (good - 2);
    } else {
        adc_mv_avg = mv_acc / good;
    }
    if (adc_mv_avg < 0) {
        adc_mv_avg = 0;
    }

    uint16_t battery_mv = calc_battery_mv((uint32_t)adc_mv_avg);

    if (!battery_mv_is_sane(battery_mv)) {
        ESP_LOGW(TAG, "Battery ADC glitch: adc_mv=%d -> battery_mv=%u (reject), holding last",
                 adc_mv_avg, (unsigned)battery_mv);
        if (s_have_last_good) {
            battery_mv = s_last_battery_mv;
        } else {
            return;
        }
    }

    status->battery_mv = battery_mv;
    status->battery_voltage_attr = battery_mv_to_zcl_voltage_attr(battery_mv);
    status->battery_percent_attr = calc_battery_percent(battery_mv);

    s_last_battery_mv = battery_mv;
    s_have_last_good = true;

    ESP_LOGI(TAG, "Battery ADC: adc_mv_avg=%d battery_mv=%u voltage_attr=0x%02x percent_attr=0x%02x samples=%d",
             adc_mv_avg,
             (unsigned)battery_mv,
             (unsigned)status->battery_voltage_attr,
             (unsigned)status->battery_percent_attr,
             good);
#endif
}

static void power_monitor_task(void *arg)
{
    (void)arg;
    while (true) {
        power_status_t status = {0};
        power_read_status(&status);

        if (s_power_cb) {
            s_power_cb(&status, s_power_cb_ctx);
        }

        uint32_t period_ms = s_power_period_ms;
        if (period_ms == 0) {
            period_ms = 1000;
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

esp_err_t power_start_monitor(uint32_t period_ms, power_status_cb_t cb, void *ctx)
{
    if (s_power_task) {
        return ESP_ERR_INVALID_STATE;
    }

    s_power_cb = cb;
    s_power_cb_ctx = ctx;
    s_power_period_ms = period_ms;

    if (xTaskCreate(power_monitor_task, "power_mon", 3072, NULL, 5, &s_power_task) != pdPASS) {
        s_power_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
