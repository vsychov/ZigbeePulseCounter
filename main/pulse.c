#include "pulse.h"

#include "sdkconfig.h"

#include <limits.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "iot_button.h"
#include "button_gpio.h"

static const char *TAG = "pulse";

static pulse_config_t s_cfg;
static button_handle_t s_button;
static volatile uint64_t s_total_pulses;
static int64_t s_last_valid_us;
static int64_t s_prev_valid_us;
static volatile uint32_t s_pending_pulses;
static volatile uint32_t s_dropped_pulses;
static TaskHandle_t s_consumer_task;
static pulse_cb_t s_cb;
static void *s_cb_arg;
static portMUX_TYPE s_pulse_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_blocked;
static volatile bool s_enabled = true;
static int64_t s_last_down_us;

static void pulse_notify_consumer(void)
{
    if (s_consumer_task) {
        xTaskNotifyGive(s_consumer_task);
    }
}

static void pulse_record_valid(int64_t now_us)
{
    portENTER_CRITICAL(&s_pulse_mux);
    s_prev_valid_us = s_last_valid_us;
    s_last_valid_us = now_us;
    if (s_total_pulses != UINT64_MAX) {
        s_total_pulses++;
    }
    if (s_pending_pulses == UINT32_MAX) {
        s_dropped_pulses++;
    } else {
        s_pending_pulses++;
    }
    portEXIT_CRITICAL(&s_pulse_mux);

    if (s_cb) {
        s_cb(s_cb_arg);
    }
    pulse_notify_consumer();
}

static void pulse_on_press_down(void *arg, void *data)
{
    (void)arg;
    (void)data;
    s_last_down_us = esp_timer_get_time();
}

static void pulse_on_press_up(void *arg, void *data)
{
    (void)arg;
    (void)data;

    if (!s_enabled || s_blocked) {
        return;
    }

    int64_t now = esp_timer_get_time();
    int64_t down = s_last_down_us;
    if (down == 0 || down > now) {
        down = now;
    }

    int64_t width_us = now - down;
    if (s_cfg.min_width_ms > 0 && width_us < ((int64_t)s_cfg.min_width_ms * 1000LL)) {
        return;
    }

    int64_t last_valid_us = 0;
    uint16_t debounce_ms = 0;
    portENTER_CRITICAL(&s_pulse_mux);
    last_valid_us = s_last_valid_us;
    debounce_ms = s_cfg.debounce_ms;
    portEXIT_CRITICAL(&s_pulse_mux);

    if (debounce_ms > 0 && (now - last_valid_us) < ((int64_t)debounce_ms * 1000LL)) {
        return;
    }

    pulse_record_valid(now);
}

esp_err_t pulse_init(const pulse_config_t *cfg, pulse_cb_t cb, void *cb_arg, uint64_t initial_total)
{
    if (!cfg || cfg->gpio_num < 0 || cfg->gpio_num >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid pulse GPIO %d", cfg ? cfg->gpio_num : -1);
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *cfg;
    s_cb = cb;
    s_cb_arg = cb_arg;
    s_total_pulses = initial_total;
    s_last_valid_us = 0;
    s_prev_valid_us = 0;
    s_pending_pulses = 0;
    s_dropped_pulses = 0;
    s_blocked = false;
    s_enabled = true;
    s_last_down_us = 0;
    s_button = NULL;
    s_consumer_task = NULL;

    button_config_t btn_cfg = {
        .long_press_time = 0,
        .short_press_time = 0,
    };
    button_gpio_config_t gpio_cfg = {
        .gpio_num = cfg->gpio_num,
        .active_level = 0, /* pulses are active-low */
        /* Power-save lets the driver stop its timer when idle; wake via GPIO interrupt. */
        .enable_power_save = true,
        .disable_pull = false,
    };

    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &s_button);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init button on GPIO%d: %s", cfg->gpio_num, esp_err_to_name(err));
        return err;
    }

    err = iot_button_register_cb(s_button, BUTTON_PRESS_DOWN, NULL, pulse_on_press_down, NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = iot_button_register_cb(s_button, BUTTON_PRESS_UP, NULL, pulse_on_press_up, NULL);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

void pulse_update_debounce(uint16_t debounce_ms)
{
    portENTER_CRITICAL(&s_pulse_mux);
    s_cfg.debounce_ms = debounce_ms;
    portEXIT_CRITICAL(&s_pulse_mux);
}

void pulse_set_total(uint64_t total)
{
    portENTER_CRITICAL(&s_pulse_mux);
    s_total_pulses = total;
    s_pending_pulses = 0;
    s_dropped_pulses = 0;
    portEXIT_CRITICAL(&s_pulse_mux);
}

uint64_t pulse_get_total(void)
{
    uint64_t total = 0;
    portENTER_CRITICAL(&s_pulse_mux);
    total = s_total_pulses;
    portEXIT_CRITICAL(&s_pulse_mux);
    return total;
}

void pulse_set_consumer_task(TaskHandle_t task)
{
    s_consumer_task = task;
    if (s_pending_pulses > 0) {
        pulse_notify_consumer();
    }
}

bool pulse_take_pending(pulse_pending_info_t *info)
{
    if (!info) {
        return false;
    }

    bool has = false;
    portENTER_CRITICAL(&s_pulse_mux);
    if (s_pending_pulses > 0 || s_dropped_pulses > 0) {
        info->count = s_pending_pulses;
        info->lost = s_dropped_pulses;
        info->last_ts_us = s_last_valid_us;
        info->prev_ts_us = s_prev_valid_us;
        s_pending_pulses = 0;
        s_dropped_pulses = 0;
        has = true;
    } else {
        info->count = 0;
        info->lost = 0;
        info->last_ts_us = s_last_valid_us;
        info->prev_ts_us = s_prev_valid_us;
    }
    portEXIT_CRITICAL(&s_pulse_mux);
    return has;
}

void pulse_block(bool block)
{
    s_blocked = block;
}

void pulse_enable(bool enable)
{
    s_enabled = enable;
    if (enable) {
        (void)iot_button_resume();
    } else {
        (void)iot_button_stop();
    }
}

bool pulse_record_wakeup(int64_t now_us)
{
    if (!s_enabled || s_blocked) {
        return false;
    }

    bool counted = false;
    portENTER_CRITICAL(&s_pulse_mux);
    if ((now_us - s_last_valid_us) >= ((int64_t)s_cfg.debounce_ms * 1000)) {
        s_prev_valid_us = s_last_valid_us;
        s_last_valid_us = now_us;
        if (s_total_pulses != UINT64_MAX) {
            s_total_pulses++;
        }
        if (s_pending_pulses == UINT32_MAX) {
            s_dropped_pulses++;
        } else {
            s_pending_pulses++;
        }
        counted = true;
    }
    portEXIT_CRITICAL(&s_pulse_mux);

    if (counted) {
        if (s_cb) {
            s_cb(s_cb_arg);
        }
        pulse_notify_consumer();
    }
    return counted;
}
