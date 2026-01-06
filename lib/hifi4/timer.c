/**
 * @file timer.c
 * @brief Timer driver implementation for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 *
 * Implements the 32-bit down counter timers (Timer0 and Timer1).
 */

#include "hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

static struct {
    irq_handler_t handler;
    void *arg;
} timer_handlers[2];

static uint32_t timer_clock_hz[2] = {24000000, 24000000}; /* Default 24 MHz */

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline void timer_write_reg(uint32_t offset, uint32_t value)
{
    REG32(TIMER_BASE + offset) = value;
}

static inline uint32_t timer_read_reg(uint32_t offset)
{
    return REG32(TIMER_BASE + offset);
}

static uint32_t timer_get_ctrl_offset(timer_id_t id)
{
    return (id == TIMER_0) ? TMR0_CTRL : TMR1_CTRL;
}

static uint32_t timer_get_intv_offset(timer_id_t id)
{
    return (id == TIMER_0) ? TMR0_INTV : TMR1_INTV;
}

static uint32_t timer_get_cur_offset(timer_id_t id)
{
    return (id == TIMER_0) ? TMR0_CUR : TMR1_CUR;
}

static uint32_t timer_get_irq_bit(timer_id_t id)
{
    return (id == TIMER_0) ? TMR_IRQ_TMR0 : TMR_IRQ_TMR1;
}

/**
 * @brief Calculate timer ticks from microseconds without 64-bit division
 * 
 * Uses the formula: ticks = (clock_hz / 1000000) * interval_us
 * with adjustment for clocks not evenly divisible by 1MHz
 */
static uint32_t timer_us_to_ticks(uint32_t clock_hz, uint32_t interval_us)
{
    /* For 24 MHz: ticks_per_us = 24 */
    uint32_t ticks_per_us = clock_hz / 1000000;
    uint32_t remainder = clock_hz % 1000000;
    
    uint32_t ticks = ticks_per_us * interval_us;
    
    /* Add correction for remainder: (remainder * interval_us) / 1000000 */
    /* To avoid overflow, limit interval_us or use iterative approach */
    if (remainder > 0 && interval_us > 0) {
        /* For small intervals, this is accurate enough */
        /* remainder * interval_us / 1000000 ≈ remainder / (1000000 / interval_us) */
        if (interval_us <= 1000000) {
            ticks += (remainder / (1000000 / interval_us));
        }
    }
    
    if (ticks == 0) ticks = 1;
    return ticks;
}

/*============================================================================
 * Interrupt Handlers
 *============================================================================*/

static void timer0_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    
    /* Clear interrupt */
    timer_write_reg(TMR_IRQ_STA, TMR_IRQ_TMR0);
    
    /* Call user handler */
    if (timer_handlers[0].handler) {
        timer_handlers[0].handler(TIMER_0, timer_handlers[0].arg);
    }
}

static void timer1_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    
    /* Clear interrupt */
    timer_write_reg(TMR_IRQ_STA, TMR_IRQ_TMR1);
    
    /* Call user handler */
    if (timer_handlers[1].handler) {
        timer_handlers[1].handler(TIMER_1, timer_handlers[1].arg);
    }
}

/*============================================================================
 * Public Functions
 *============================================================================*/

void timer_init(timer_id_t timer_id, timer_clk_src_t clk_src, uint8_t prescaler)
{
    uint32_t ctrl_offset = timer_get_ctrl_offset(timer_id);
    uint32_t ctrl = 0;
    
    /* Stop timer first */
    timer_write_reg(ctrl_offset, 0);
    
    /* Configure clock source */
    if (clk_src == TIMER_CLK_LOSC) {
        ctrl |= TMR_CTRL_CLK_LOSC;
        timer_clock_hz[timer_id] = 32768;
    } else {
        ctrl |= TMR_CTRL_CLK_OSC24M;
        timer_clock_hz[timer_id] = 24000000;
    }
    
    /* Set prescaler (0-7) */
    prescaler &= 0x07;
    ctrl |= TMR_CTRL_PRESCALE(prescaler);
    timer_clock_hz[timer_id] >>= prescaler;
    
    timer_write_reg(ctrl_offset, ctrl);
    
    /* Disable interrupt initially */
    uint32_t irq_en = timer_read_reg(TMR_IRQ_EN);
    irq_en &= ~timer_get_irq_bit(timer_id);
    timer_write_reg(TMR_IRQ_EN, irq_en);
    
    /* Clear any pending interrupt */
    timer_write_reg(TMR_IRQ_STA, timer_get_irq_bit(timer_id));
}

void timer_start_oneshot(timer_id_t timer_id, uint32_t interval_us,
                         irq_handler_t handler, void *arg)
{
    uint32_t ctrl_offset = timer_get_ctrl_offset(timer_id);
    uint32_t intv_offset = timer_get_intv_offset(timer_id);
    
    /* Calculate interval value */
    uint32_t ticks = timer_us_to_ticks(timer_clock_hz[timer_id], interval_us);
    
    /* Stop timer */
    uint32_t ctrl = timer_read_reg(ctrl_offset);
    ctrl &= ~TMR_CTRL_EN;
    timer_write_reg(ctrl_offset, ctrl);
    
    /* Set interval */
    timer_write_reg(intv_offset, ticks);
    
    /* Store handler */
    timer_handlers[timer_id].handler = handler;
    timer_handlers[timer_id].arg = arg;
    
    /* Enable interrupt if handler provided */
    if (handler) {
        /* Register system IRQ handler */
        uint32_t irq_num = (timer_id == TIMER_0) ? IRQ_TIMER0 : IRQ_TIMER1;
        irq_handler_t sys_handler = (timer_id == TIMER_0) ? timer0_irq_handler : timer1_irq_handler;
        irq_register(irq_num, sys_handler, NULL);
        irq_enable_interrupt(irq_num);
        
        /* Enable timer interrupt */
        uint32_t irq_en = timer_read_reg(TMR_IRQ_EN);
        irq_en |= timer_get_irq_bit(timer_id);
        timer_write_reg(TMR_IRQ_EN, irq_en);
    }
    
    /* Clear pending interrupt */
    timer_write_reg(TMR_IRQ_STA, timer_get_irq_bit(timer_id));
    
    /* Configure for single-shot mode (bit 7 = 1) and trigger reload */
    ctrl |= TMR_CTRL_MODE_SINGLE | TMR_CTRL_RELOAD;
    timer_write_reg(ctrl_offset, ctrl);
    
    /* Enable timer */
    ctrl |= TMR_CTRL_EN;
    timer_write_reg(ctrl_offset, ctrl);
}

void timer_start_periodic(timer_id_t timer_id, uint32_t interval_us,
                          irq_handler_t handler, void *arg)
{
    uint32_t ctrl_offset = timer_get_ctrl_offset(timer_id);
    uint32_t intv_offset = timer_get_intv_offset(timer_id);
    
    /* Calculate interval value */
    uint32_t ticks = timer_us_to_ticks(timer_clock_hz[timer_id], interval_us);
    
    /* Stop timer */
    uint32_t ctrl = timer_read_reg(ctrl_offset);
    ctrl &= ~TMR_CTRL_EN;
    timer_write_reg(ctrl_offset, ctrl);
    
    /* Set interval */
    timer_write_reg(intv_offset, ticks);
    
    /* Store handler */
    timer_handlers[timer_id].handler = handler;
    timer_handlers[timer_id].arg = arg;
    
    /* Enable interrupt if handler provided */
    if (handler) {
        uint32_t irq_num = (timer_id == TIMER_0) ? IRQ_TIMER0 : IRQ_TIMER1;
        irq_handler_t sys_handler = (timer_id == TIMER_0) ? timer0_irq_handler : timer1_irq_handler;
        irq_register(irq_num, sys_handler, NULL);
        irq_enable_interrupt(irq_num);
        
        /* Enable timer interrupt in timer peripheral */
        uint32_t irq_en = timer_read_reg(TMR_IRQ_EN);
        irq_en |= timer_get_irq_bit(timer_id);
        timer_write_reg(TMR_IRQ_EN, irq_en);
    }
    
    /* Clear any pending interrupt */
    timer_write_reg(TMR_IRQ_STA, timer_get_irq_bit(timer_id));
    
    /* 
     * Set periodic mode (bit 7 = 0) and trigger reload
     * The reload bit loads the interval value into the counter
     */
    ctrl &= ~TMR_CTRL_MODE_SINGLE;  /* Clear bit 7 = periodic mode */
    ctrl |= TMR_CTRL_RELOAD;        /* Trigger reload of interval value */
    timer_write_reg(ctrl_offset, ctrl);
    
    /* Enable the timer */
    ctrl |= TMR_CTRL_EN;
    timer_write_reg(ctrl_offset, ctrl);
}

void timer_stop(timer_id_t timer_id)
{
    uint32_t ctrl_offset = timer_get_ctrl_offset(timer_id);
    
    /* Disable timer */
    uint32_t ctrl = timer_read_reg(ctrl_offset);
    ctrl &= ~TMR_CTRL_EN;
    timer_write_reg(ctrl_offset, ctrl);
    
    /* Disable interrupt */
    uint32_t irq_en = timer_read_reg(TMR_IRQ_EN);
    irq_en &= ~timer_get_irq_bit(timer_id);
    timer_write_reg(TMR_IRQ_EN, irq_en);
    
    /* Clear pending */
    timer_write_reg(TMR_IRQ_STA, timer_get_irq_bit(timer_id));
    
    timer_handlers[timer_id].handler = NULL;
}

uint32_t timer_get_counter(timer_id_t timer_id)
{
    return timer_read_reg(timer_get_cur_offset(timer_id));
}

bool timer_irq_pending(timer_id_t timer_id)
{
    return (timer_read_reg(TMR_IRQ_STA) & timer_get_irq_bit(timer_id)) != 0;
}

void timer_irq_clear(timer_id_t timer_id)
{
    timer_write_reg(TMR_IRQ_STA, timer_get_irq_bit(timer_id));
}
