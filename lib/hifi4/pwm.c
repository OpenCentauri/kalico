/**
 * @file pwm.c
 * @brief PWM driver for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * Supports up to 8 PWM channels.
 */

#include "hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Store period for each channel (for duty cycle calculations) */
static uint16_t pwm_period[PWM_MAX_CHANNELS];
static uint8_t pwm_prescaler[PWM_MAX_CHANNELS];

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline void pwm_write_reg(uint32_t offset, uint32_t value)
{
    REG32(PWM_BASE + offset) = value;
}

static inline uint32_t pwm_read_reg(uint32_t offset)
{
    return REG32(PWM_BASE + offset);
}

/*============================================================================
 * Public Functions
 *============================================================================*/

int pwm_init(pwm_channel_t channel, const pwm_config_t *config, uint32_t clk_hz)
{
    static bool pwm_clk_enabled = false;
    
    if (channel >= PWM_MAX_CHANNELS || config == NULL) {
        return -1;
    }
    
    /* Enable PWM clock once (shared by all channels) */
    if (!pwm_clk_enabled) {
        ccu_pwm_enable();
        pwm_clk_enabled = true;
    }
    
    /* Disable channel first */
    uint32_t per = pwm_read_reg(PWM_PER);
    per &= ~PWM_PER_EN(channel);
    pwm_write_reg(PWM_PER, per);
    
    /* Configure clock source (use HOSC 24MHz) */
    uint32_t pccr = PWM_CLK_SRC_HOSC;
    
    /* Calculate prescaler and period
     * PWM freq = clk_hz / ((prescaler + 1) * (period + 1) * clock_div)
     * 
     * First, find the best clock divider (2^n where n=0..15)
     * Then calculate prescaler and period
     */
    uint32_t target_freq = config->frequency_hz;
    if (target_freq == 0) target_freq = 1000; /* Default 1 kHz */
    
    /* Start with no additional clock divider */
    uint32_t clk_div = 0;
    uint32_t effective_clk = clk_hz;
    
    /* Find appropriate clock divider */
    while (clk_div < 15) {
        uint32_t cycles = effective_clk / target_freq;
        if (cycles <= 65536 * 256) {
            break; /* Can fit in prescaler * period */
        }
        clk_div++;
        effective_clk = clk_hz >> clk_div;
    }
    
    pccr |= PWM_CLK_DIV(clk_div);
    pwm_write_reg(PWM_PCCR(channel / 2), pccr); /* Two channels share clock config */
    
    /* Enable clock gating for this channel */
    uint32_t pcgr = pwm_read_reg(PWM_PCGR);
    pcgr |= BIT(channel);
    pwm_write_reg(PWM_PCGR, pcgr);
    
    /* Calculate prescaler and period
     * total_cycles = effective_clk / target_freq
     * prescaler * period = total_cycles - 1 (approximately)
     */
    uint32_t total_cycles = effective_clk / target_freq;
    
    uint8_t prescaler = 0;
    uint32_t period = 0;
    
    /* Try to maximize period for better resolution */
    if (total_cycles <= 65536) {
        prescaler = 0;
        period = total_cycles - 1;
    } else {
        /* Find prescaler that gives best period */
        for (prescaler = 1; prescaler < 255; prescaler++) {
            period = (total_cycles / (prescaler + 1)) - 1;
            if (period <= 65535) {
                break;
            }
        }
    }
    
    if (period > 65535) period = 65535;
    
    pwm_period[channel] = (uint16_t)period;
    pwm_prescaler[channel] = prescaler;
    
    /* Configure control register */
    uint32_t pcr = prescaler & PWM_PCR_PRESCAL_MASK;
    
    if (config->active_high) {
        pcr |= PWM_PCR_ACT_STA;
    }
    
    /* Cycle mode (continuous) */
    pcr &= ~PWM_PCR_MODE;
    
    pwm_write_reg(PWM_PCR(channel), pcr);
    
    /* Set period and initial duty cycle */
    uint16_t active_cycles;
    if (config->duty_percent >= 100) {
        active_cycles = period + 1;
    } else {
        active_cycles = ((uint32_t)(period + 1) * config->duty_percent) / 100;
    }
    
    uint32_t ppr = ((uint32_t)(period) << 16) | active_cycles;
    pwm_write_reg(PWM_PPR(channel), ppr);
    
    return 0;
}

void pwm_enable(pwm_channel_t channel)
{
    if (channel >= PWM_MAX_CHANNELS) return;
    
    uint32_t per = pwm_read_reg(PWM_PER);
    per |= PWM_PER_EN(channel);
    pwm_write_reg(PWM_PER, per);
}

void pwm_disable(pwm_channel_t channel)
{
    if (channel >= PWM_MAX_CHANNELS) return;
    
    uint32_t per = pwm_read_reg(PWM_PER);
    per &= ~PWM_PER_EN(channel);
    pwm_write_reg(PWM_PER, per);
}

void pwm_set_duty(pwm_channel_t channel, uint16_t duty_percent)
{
    if (channel >= PWM_MAX_CHANNELS) return;
    if (duty_percent > 100) duty_percent = 100;
    
    uint16_t period = pwm_period[channel];
    uint16_t active_cycles;
    
    if (duty_percent >= 100) {
        active_cycles = period + 1;
    } else if (duty_percent == 0) {
        active_cycles = 0;
    } else {
        active_cycles = ((uint32_t)(period + 1) * duty_percent) / 100;
    }
    
    uint32_t ppr = ((uint32_t)period << 16) | active_cycles;
    pwm_write_reg(PWM_PPR(channel), ppr);
}

void pwm_set_duty_raw(pwm_channel_t channel, uint16_t active_cycles, uint16_t total_cycles)
{
    if (channel >= PWM_MAX_CHANNELS) return;
    
    uint16_t period = total_cycles > 0 ? total_cycles - 1 : 0;
    pwm_period[channel] = period;
    
    uint32_t ppr = ((uint32_t)period << 16) | active_cycles;
    pwm_write_reg(PWM_PPR(channel), ppr);
}

void pwm_set_frequency(pwm_channel_t channel, uint32_t frequency_hz, uint32_t clk_hz)
{
    if (channel >= PWM_MAX_CHANNELS) return;
    
    /* Keep current duty cycle percentage */
    uint32_t ppr = pwm_read_reg(PWM_PPR(channel));
    uint16_t old_period = (ppr >> 16) & 0xFFFF;
    uint16_t old_active = ppr & 0xFFFF;
    uint32_t duty_percent = 0;
    
    if (old_period > 0) {
        duty_percent = ((uint32_t)old_active * 100) / (old_period + 1);
    }
    
    /* Reconfigure with new frequency */
    pwm_config_t config = {
        .frequency_hz = frequency_hz,
        .duty_percent = (uint16_t)duty_percent,
        .active_high = (pwm_read_reg(PWM_PCR(channel)) & PWM_PCR_ACT_STA) != 0,
    };
    
    /* Disable, reconfigure, re-enable if was enabled */
    uint32_t per = pwm_read_reg(PWM_PER);
    bool was_enabled = (per & PWM_PER_EN(channel)) != 0;
    
    if (was_enabled) {
        pwm_disable(channel);
    }
    
    pwm_init(channel, &config, clk_hz);
    
    if (was_enabled) {
        pwm_enable(channel);
    }
}

uint16_t pwm_get_counter(pwm_channel_t channel)
{
    if (channel >= PWM_MAX_CHANNELS) return 0;
    
    return (uint16_t)(pwm_read_reg(PWM_PCNTR(channel)) & 0xFFFF);
}

/*============================================================================
 * Dead Zone Configuration (for complementary outputs)
 *============================================================================*/

/**
 * @brief Configure dead zone for a PWM channel pair
 * @param pair Channel pair (0 for CH0/CH1, 1 for CH2/CH3, etc.)
 * @param dead_time Dead time in clock cycles (0-255)
 */
void pwm_set_dead_zone(uint8_t pair, uint8_t dead_time)
{
    if (pair >= PWM_MAX_CHANNELS / 2) return;
    
    uint32_t dzcr = dead_time | BIT(8); /* Enable dead zone */
    pwm_write_reg(PWM_PDZCR(pair), dzcr);
}

/**
 * @brief Disable dead zone for a channel pair
 * @param pair Channel pair
 */
void pwm_disable_dead_zone(uint8_t pair)
{
    if (pair >= PWM_MAX_CHANNELS / 2) return;
    
    pwm_write_reg(PWM_PDZCR(pair), 0);
}
