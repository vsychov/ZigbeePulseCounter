#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef void (*pulse_cb_t)(void *arg);

typedef struct {
    int gpio_num;
    uint16_t debounce_ms;
    uint16_t min_width_ms;
} pulse_config_t;

typedef struct {
    uint32_t count;
    uint32_t lost;
    int64_t last_ts_us;
    int64_t prev_ts_us;
} pulse_pending_info_t;

esp_err_t pulse_init(const pulse_config_t *cfg, pulse_cb_t cb, void *cb_arg, uint64_t initial_total);
void pulse_update_debounce(uint16_t debounce_ms);
void pulse_set_total(uint64_t total);
uint64_t pulse_get_total(void);
void pulse_set_consumer_task(TaskHandle_t task);
bool pulse_take_pending(pulse_pending_info_t *info);
/* Temporarily drop incoming pulses (e.g., while a shared button is held). */
void pulse_block(bool block);
/* Enable/disable pulse interrupt processing. */
void pulse_enable(bool enable);

/* Record a pulse after a level-based wakeup (EXT1 ANY_LOW / GPIO low wake),
 * where the NEGEDGE IRQ might not have been delivered while CPU slept.
 */
bool pulse_record_wakeup(int64_t now_us);
