#include "metering.h"
#include <limits.h>
#include <string.h>
#include <math.h>

#ifndef CONFIG_DEMAND_IDLE_TIMEOUT_S
#define CONFIG_DEMAND_IDLE_TIMEOUT_S 0
#endif

static app_metering_cfg_t s_cfg;
static uint64_t s_total_pulses;
static uint8_t s_summation_formatting;
static uint8_t s_demand_formatting;
static uint32_t s_multiplier = 1;
static uint32_t s_divisor = 1;
static int32_t s_instantaneous_demand;
static int64_t s_last_pulse_us;
static double s_rate_est_ph;
static int64_t s_rate_last_update_us;

static int32_t clamp_demand_int24(int32_t value)
{
    if (value > 0x7FFFFF) {
        return 0x7FFFFF;
    }
    if (value < 0) {
        return 0;
    }
    return value;
}

static uint32_t clamp_divisor(uint32_t v)
{
    if (v == 0) {
        v = 1;
    }
    if (v > 0xFFFFFF) {
        v = 0xFFFFFF;
    }
    return v;
}

static double decay_mul(double dt_s, double tau_s)
{
    if (tau_s <= 0.0 || dt_s <= 0.0) {
        return 1.0;
    }
    return exp(-dt_s / tau_s);
}

static uint8_t calc_digits_right(uint32_t divisor)
{
    uint8_t digits = 0;
    while ((divisor % 10 == 0) && divisor > 1 && digits < 7) {
        divisor /= 10;
        digits++;
    }
    return digits;
}

static uint8_t calc_summation_formatting(const app_metering_cfg_t *cfg)
{
    uint32_t div = clamp_divisor(cfg->pulse_per_unit_numerator);
    uint8_t digits_right = calc_digits_right(div);
    uint8_t digits_left = 10;

    return (digits_right & 0x07) | ((digits_left & 0x0F) << 3) | 0x80;
}

static uint8_t calc_demand_formatting(const app_metering_cfg_t *cfg)
{
    return calc_summation_formatting(cfg);
}

static uint64_t calc_summation(uint64_t total_pulses)
{
    return total_pulses;
}

static void update_scaling(const app_metering_cfg_t *cfg)
{
    s_multiplier = 1;
    s_divisor = clamp_divisor(cfg->pulse_per_unit_numerator);
}

void metering_init(const app_metering_cfg_t *cfg, uint64_t total_pulses)
{
    memcpy(&s_cfg, cfg, sizeof(s_cfg));
    s_total_pulses = total_pulses;
    s_summation_formatting = calc_summation_formatting(cfg);
    s_demand_formatting = calc_demand_formatting(cfg);
    update_scaling(cfg);
    s_instantaneous_demand = 0;
    s_last_pulse_us = 0;
    s_rate_est_ph = 0.0;
    s_rate_last_update_us = 0;
}

bool metering_tick(int64_t now_us)
{
    if (s_rate_last_update_us == 0) {
        s_rate_last_update_us = now_us;
        return false;
    }

#if CONFIG_DEMAND_IDLE_TIMEOUT_S > 0
    if (s_last_pulse_us > 0 &&
        (now_us - s_last_pulse_us) >= ((int64_t)CONFIG_DEMAND_IDLE_TIMEOUT_S * 1000000LL)) {
        bool changed = (s_instantaneous_demand != 0);
        s_rate_est_ph = 0.0;
        s_instantaneous_demand = 0;
        s_rate_last_update_us = now_us;
        return changed;
    }
#endif

    double dt_s = (double)(now_us - s_rate_last_update_us) / 1000000.0;
    if (dt_s <= 0.0) {
        return false;
    }

    s_rate_est_ph *= decay_mul(dt_s, CONFIG_DEMAND_DECAY_TAU_S);
    int32_t new_demand = clamp_demand_int24((int32_t)llround(s_rate_est_ph));
    bool changed = (new_demand != s_instantaneous_demand);
    s_instantaneous_demand = new_demand;
    s_rate_last_update_us = now_us;
    return changed;
}

static void update_instantaneous_demand(int64_t last_us, int64_t prev_us)
{
    if (prev_us > 0 && last_us > prev_us) {
        int64_t dt_us = last_us - prev_us;
        double dt_s = (double)dt_us / 1000000.0;
        if (dt_s > 0.0) {
            double demand_pulses_per_hour = (1.0 / dt_s) * 3600.0;
            double alpha = 1.0 - decay_mul(dt_s, CONFIG_DEMAND_RISE_TAU_S);
            double inst_ph = demand_pulses_per_hour;
            s_rate_est_ph = s_rate_est_ph + alpha * (inst_ph - s_rate_est_ph);
            if (s_rate_est_ph < 0.0) {
                s_rate_est_ph = 0.0;
            }
            s_instantaneous_demand = clamp_demand_int24((int32_t)llround(s_rate_est_ph));
        }
    }
}

void metering_on_pulses(uint32_t count, int64_t last_us, int64_t prev_us)
{
    if (count == 0) {
        return;
    }

    if (UINT64_MAX - s_total_pulses < count) {
        s_total_pulses = UINT64_MAX;
    } else {
        s_total_pulses += count;
    }

    if (last_us > 0) {
        /* Apply decay up to the timestamp of this pulse. */
        metering_tick(last_us);
        /* Demand needs real timing data; skip updates when we do not have two timestamps. */
        if (prev_us > 0) {
            update_instantaneous_demand(last_us, prev_us);
        }
        s_last_pulse_us = last_us;
    }
}

void metering_on_pulse(int64_t now_us)
{
    metering_on_pulses(1, now_us, s_last_pulse_us);
}

void metering_set_config(const app_metering_cfg_t *cfg)
{
    memcpy(&s_cfg, cfg, sizeof(s_cfg));
    s_summation_formatting = calc_summation_formatting(cfg);
    s_demand_formatting = calc_demand_formatting(cfg);
    update_scaling(cfg);
}

uint64_t metering_get_total_pulses(void)
{
    return s_total_pulses;
}

uint64_t metering_get_summation(void)
{
    return calc_summation(s_total_pulses);
}

void metering_reset(void)
{
    s_total_pulses = 0;
    s_instantaneous_demand = 0;
    s_last_pulse_us = 0;
    s_rate_est_ph = 0.0;
    s_rate_last_update_us = 0;
}

uint8_t metering_get_unit_of_measure(void)
{
    return s_cfg.unit_of_measure;
}

uint8_t metering_get_device_type(void)
{
    return s_cfg.metering_device_type;
}

uint8_t metering_get_summation_formatting(void)
{
    return s_summation_formatting;
}

uint8_t metering_get_demand_formatting(void)
{
    return s_demand_formatting;
}

uint32_t metering_get_multiplier(void)
{
    return s_multiplier;
}

uint32_t metering_get_divisor(void)
{
    return s_divisor;
}

int32_t metering_get_instantaneous_demand(void)
{
    return s_instantaneous_demand;
}

void metering_set_instantaneous_demand(int32_t demand)
{
    s_instantaneous_demand = demand;
}

int64_t metering_get_last_pulse_us(void)
{
    return s_last_pulse_us;
}
