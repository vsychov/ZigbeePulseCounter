#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

void metering_init(const app_metering_cfg_t *cfg, uint64_t total_pulses);
void metering_on_pulse(int64_t now_us);
void metering_on_pulses(uint32_t count, int64_t last_us, int64_t prev_us);
bool metering_tick(int64_t now_us);
void metering_set_config(const app_metering_cfg_t *cfg);
uint64_t metering_get_total_pulses(void);
uint64_t metering_get_summation(void);
void metering_reset(void);

uint8_t metering_get_unit_of_measure(void);
uint8_t metering_get_device_type(void);
uint8_t metering_get_summation_formatting(void);
uint8_t metering_get_demand_formatting(void);
uint32_t metering_get_multiplier(void);
uint32_t metering_get_divisor(void);
int32_t metering_get_instantaneous_demand(void);
void metering_set_instantaneous_demand(int32_t demand);
int64_t metering_get_last_pulse_us(void);
