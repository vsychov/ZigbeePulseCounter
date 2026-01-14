#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    uint16_t battery_mv;
    uint8_t battery_voltage_attr;
    uint8_t battery_percent_attr;
} power_status_t;

void power_init(void);
void power_read_status(power_status_t *status);

typedef void (*power_status_cb_t)(const power_status_t *status, void *ctx);

/* Start a background task that periodically reads battery status and notifies via callback. */
esp_err_t power_start_monitor(uint32_t period_ms, power_status_cb_t cb, void *ctx);
