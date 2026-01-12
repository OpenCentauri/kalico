/**
 * @file hstimer.c
 * @brief High-Speed Timer driver for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * Implements the 56-bit down counter high-speed timers.
 * These run at 24 MHz with optional prescaler.
 * 
 * Note: API uses microseconds (uint32_t) to avoid 64-bit division
 * which requires libgcc on baremetal. Max interval ~4294 seconds.
 */

#include "hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

static struct {
    irq_handler_t handler;
    void *arg;
} hstimer_handlers[2];

static uint32_t hstimer_clock_hz[2] = {CLK_FREQ_AHB0, CLK_FREQ_AHB0};

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline void hstimer_write_reg(uint32_t offset, uint32_t value)
{
    REG32(HSTIMER_BASE + offset) = value;
}

static inline uint32_t hstimer_read_reg(uint32_t offset)
{
    return REG32(HSTIMER_BASE + offset);
}

static void hstimer_get_offsets(hstimer_id_t id, uint32_t *ctrl, uint32_t *intv_lo,
                                 uint32_t *intv_hi, uint32_t *cur_lo, uint32_t *cur_hi)
{
    if (id == HSTIMER_0) {
        *ctrl = HSTMR0_CTRL;
        *intv_lo = HSTMR0_INTV_LO;
        *intv_hi = HSTMR0_INTV_HI;
        *cur_lo = HSTMR0_CUR_LO;
        *cur_hi = HSTMR0_CUR_HI;
    } else {
        *ctrl = HSTMR1_CTRL;
        *intv_lo = HSTMR1_INTV_LO;
        *intv_hi = HSTMR1_INTV_HI;
        *cur_lo = HSTMR1_CUR_LO;
        *cur_hi = HSTMR1_CUR_HI;
    }
}

static uint32_t hstimer_get_irq_bit(hstimer_id_t id)
{
    return (id == HSTIMER_0) ? HSTMR_IRQ_TMR0 : HSTMR_IRQ_TMR1;
}

/**
 * @brief Convert microseconds to ticks without 64-bit division
 * @param hstimer_id HSTimer identifier
 * @param interval_us Interval in microseconds
 * @param ticks_lo Output: lower 32 bits of ticks
 * @param ticks_hi Output: upper 24 bits of ticks
 */
void hstimer_us_to_ticks(hstimer_id_t hstimer_id, uint32_t interval_us, 
                         uint32_t *ticks_lo, uint32_t *ticks_hi)
{
    uint32_t clock_hz = hstimer_clock_hz[hstimer_id];

    /* ticks = clock_hz * interval_us / 1000000 */
    /* For 24 MHz: ticks_per_us = 24 */
    uint32_t ticks_per_us = clock_hz / 1000000;
    
    /* For intervals up to ~178 seconds at 24MHz, this won't overflow 32 bits */
    /* 24 * 178000000 = 4.27 billion > 2^32, so we need to be careful */
    
    if (ticks_per_us > 0 && interval_us <= 0xFFFFFFFF / ticks_per_us) {
        /* Safe to multiply directly */
        uint32_t ticks = ticks_per_us * interval_us;
        *ticks_lo = ticks;
        *ticks_hi = 0;
    } else {
        /* Split the calculation to avoid overflow */
        /* ticks = (interval_us / 1000000) * clock_hz + (interval_us % 1000000) * ticks_per_us */
        uint32_t seconds = interval_us / 1000000;
        uint32_t remainder_us = interval_us % 1000000;
        
        /* This gives us ticks for whole seconds */
        uint32_t ticks_seconds = seconds * clock_hz;
        uint32_t ticks_remainder = remainder_us * ticks_per_us;
        
        /* Combine - may need to carry into high bits */
        uint32_t total_lo = ticks_seconds + ticks_remainder;
        uint32_t carry = (total_lo < ticks_seconds) ? 1 : 0;
        
        *ticks_lo = total_lo;
        *ticks_hi = carry;
    }
    
    /* Ensure at least 1 tick */
    if (*ticks_lo == 0 && *ticks_hi == 0) {
        *ticks_lo = 1;
    }
}

/*============================================================================
 * Interrupt Handlers
 *============================================================================*/

static void hstimer0_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    
    /* Clear interrupt */
    hstimer_write_reg(HSTMR_IRQ_STA, HSTMR_IRQ_TMR0);
    
    if (hstimer_handlers[0].handler) {
        hstimer_handlers[0].handler(HSTIMER_0, hstimer_handlers[0].arg);
    }
}

static void hstimer1_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    
    /* Clear interrupt */
    hstimer_write_reg(HSTMR_IRQ_STA, HSTMR_IRQ_TMR1);
    
    if (hstimer_handlers[1].handler) {
        hstimer_handlers[1].handler(HSTIMER_1, hstimer_handlers[1].arg);
    }
}

/*============================================================================
 * Public Functions
 *============================================================================*/

void hstimer_init(hstimer_id_t hstimer_id, uint8_t prescaler)
{
    ccu_hstimer_enable();

    uint32_t ctrl_off, intv_lo_off, intv_hi_off, cur_lo_off, cur_hi_off;
    hstimer_get_offsets(hstimer_id, &ctrl_off, &intv_lo_off, &intv_hi_off, 
                        &cur_lo_off, &cur_hi_off);
    
    /* Stop timer */
    hstimer_write_reg(ctrl_off, 0);
    
    /* Set prescaler (0-7, divides by 2^n) */
    prescaler &= 0x07;
    hstimer_write_reg(ctrl_off, HSTMR_CTRL_CLK_PRE(prescaler));
    
    hstimer_clock_hz[hstimer_id] = CLK_FREQ_AHB0 >> prescaler;
    
    /* Disable interrupt */
    uint32_t irq_en = hstimer_read_reg(HSTMR_IRQ_EN);
    irq_en &= ~hstimer_get_irq_bit(hstimer_id);
    hstimer_write_reg(HSTMR_IRQ_EN, irq_en);
    
    /* Clear pending */
    hstimer_write_reg(HSTMR_IRQ_STA, hstimer_get_irq_bit(hstimer_id));
}

void hstimer_start_oneshot(hstimer_id_t hstimer_id, uint32_t ticks_lo,
                           uint32_t ticks_hi, irq_handler_t handler, void *arg)
{
    uint32_t ctrl_off, intv_lo_off, intv_hi_off, cur_lo_off, cur_hi_off;
    hstimer_get_offsets(hstimer_id, &ctrl_off, &intv_lo_off, &intv_hi_off,
                        &cur_lo_off, &cur_hi_off);
    
    /* Stop timer */
    uint32_t ctrl = hstimer_read_reg(ctrl_off);
    ctrl &= ~HSTMR_CTRL_EN;
    hstimer_write_reg(ctrl_off, ctrl);
    
    /* Set interval (56-bit split into low 32 and high 24 bits) */
    hstimer_write_reg(intv_lo_off, ticks_lo);
    hstimer_write_reg(intv_hi_off, ticks_hi & 0x00FFFFFF);
    
    /* Store handler */
    hstimer_handlers[hstimer_id].handler = handler;
    hstimer_handlers[hstimer_id].arg = arg;
    
    if (handler) {
        uint32_t irq_num = (hstimer_id == HSTIMER_0) ? IRQ_HSTIMER0 : IRQ_HSTIMER1;
        irq_handler_t sys_handler = (hstimer_id == HSTIMER_0) ? hstimer0_irq_handler : hstimer1_irq_handler;
        irq_register(irq_num, sys_handler, NULL);
        irq_enable_interrupt(irq_num);
        
        uint32_t irq_en = hstimer_read_reg(HSTMR_IRQ_EN);
        irq_en |= hstimer_get_irq_bit(hstimer_id);
        hstimer_write_reg(HSTMR_IRQ_EN, irq_en);
    }
    
    /* Clear pending interrupt */
    hstimer_write_reg(HSTMR_IRQ_STA, hstimer_get_irq_bit(hstimer_id));

    /* Configure for single-shot mode (bit 7 = 1) and trigger reload */
    ctrl |= HSTMR_CTRL_MODE_SINGLE | HSTMR_CTRL_RELOAD;
    hstimer_write_reg(ctrl_off, ctrl);

    /* Enable timer */
    ctrl |= HSTMR_CTRL_EN;
    hstimer_write_reg(ctrl_off, ctrl);
}

void hstimer_start_periodic(hstimer_id_t hstimer_id, uint32_t ticks_lo,
                            uint32_t ticks_hi, irq_handler_t handler, void *arg)
{
    uint32_t ctrl_off, intv_lo_off, intv_hi_off, cur_lo_off, cur_hi_off;
    hstimer_get_offsets(hstimer_id, &ctrl_off, &intv_lo_off, &intv_hi_off,
                        &cur_lo_off, &cur_hi_off);
    
    uint32_t ctrl = hstimer_read_reg(ctrl_off);
    ctrl &= ~HSTMR_CTRL_EN;
    hstimer_write_reg(ctrl_off, ctrl);
    
    hstimer_write_reg(intv_lo_off, ticks_lo);
    hstimer_write_reg(intv_hi_off, ticks_hi & 0x00FFFFFF);
    
    hstimer_handlers[hstimer_id].handler = handler;
    hstimer_handlers[hstimer_id].arg = arg;
    
    if (handler) {
        uint32_t irq_num = (hstimer_id == HSTIMER_0) ? IRQ_HSTIMER0 : IRQ_HSTIMER1;
        irq_handler_t sys_handler = (hstimer_id == HSTIMER_0) ? hstimer0_irq_handler : hstimer1_irq_handler;
        irq_register(irq_num, sys_handler, NULL);
        irq_enable_interrupt(irq_num);
        
        uint32_t irq_en = hstimer_read_reg(HSTMR_IRQ_EN);
        irq_en |= hstimer_get_irq_bit(hstimer_id);
        hstimer_write_reg(HSTMR_IRQ_EN, irq_en);
    }

    /* Clear any pending interrupt */
    hstimer_write_reg(HSTMR_IRQ_STA, hstimer_get_irq_bit(hstimer_id));

    /* 
     * Set periodic mode (bit 7 = 0) and trigger reload
     * The reload bit loads the interval value into the counter
     */
    ctrl &= ~HSTMR_CTRL_MODE_SINGLE;  /* Clear bit 7 = periodic mode */
    ctrl |= HSTMR_CTRL_RELOAD;        /* Trigger reload of interval value */
    hstimer_write_reg(ctrl_off, ctrl);

    /* Enable the timer */
    ctrl |= HSTMR_CTRL_EN;
    hstimer_write_reg(ctrl_off, ctrl);
}

void hstimer_stop(hstimer_id_t hstimer_id)
{
    uint32_t ctrl_off, intv_lo_off, intv_hi_off, cur_lo_off, cur_hi_off;
    hstimer_get_offsets(hstimer_id, &ctrl_off, &intv_lo_off, &intv_hi_off,
                        &cur_lo_off, &cur_hi_off);
    
    uint32_t ctrl = hstimer_read_reg(ctrl_off);
    ctrl &= ~HSTMR_CTRL_EN;
    hstimer_write_reg(ctrl_off, ctrl);
    
    uint32_t irq_en = hstimer_read_reg(HSTMR_IRQ_EN);
    irq_en &= ~hstimer_get_irq_bit(hstimer_id);
    hstimer_write_reg(HSTMR_IRQ_EN, irq_en);
    
    hstimer_write_reg(HSTMR_IRQ_STA, hstimer_get_irq_bit(hstimer_id));
    hstimer_handlers[hstimer_id].handler = NULL;
}

void hstimer_get_counter(hstimer_id_t hstimer_id, uint32_t *lo, uint32_t *hi)
{
    uint32_t ctrl_off, intv_lo_off, intv_hi_off, cur_lo_off, cur_hi_off;
    hstimer_get_offsets(hstimer_id, &ctrl_off, &intv_lo_off, &intv_hi_off,
                        &cur_lo_off, &cur_hi_off);
    
    /* Read high first, then low, then high again to detect wrap */
    uint32_t hi1 = hstimer_read_reg(cur_hi_off);
    uint32_t lo_val = hstimer_read_reg(cur_lo_off);
    uint32_t hi2 = hstimer_read_reg(cur_hi_off);
    
    /* If high changed, re-read low */
    if (hi1 != hi2) {
        lo_val = hstimer_read_reg(cur_lo_off);
    }
    
    *lo = lo_val;
    *hi = hi2 & 0x00FFFFFF;
}
