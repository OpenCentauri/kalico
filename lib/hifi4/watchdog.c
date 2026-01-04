/**
 * @file watchdog.c
 * @brief Watchdog driver for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license. 
 */

#include "hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

static irq_handler_t wdog_handler = NULL;
static void *wdog_handler_arg = NULL;
static uint32_t wdog_interval = 0;

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline void wdog_write_reg(uint32_t offset, uint32_t value)
{
    REG32(WDOG_BASE + offset) = value;
}

static inline uint32_t wdog_read_reg(uint32_t offset)
{
    return REG32(WDOG_BASE + offset);
}

/*============================================================================
 * Interrupt Handler
 *============================================================================*/

static void wdog_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    
    /* Clear interrupt */
    wdog_write_reg(WDOG_IRQ_STA, BIT(0));
    
    if (wdog_handler) {
        wdog_handler(0, wdog_handler_arg);
    }
}

/*============================================================================
 * Public Functions
 *============================================================================*/

void watchdog_init(uint32_t timeout_ms, bool reset_on_timeout)
{
    /* Stop watchdog first */
    wdog_write_reg(WDOG_MODE, 0);
    
    /* Calculate interval
     * Watchdog intervals: (n+1) * 0.5 seconds
     * n=0: 0.5s, n=1: 1s, n=2: 1.5s, ... n=15: 8s
     */
    uint32_t interval_500ms = (timeout_ms + 250) / 500; /* Round to nearest 0.5s */
    if (interval_500ms == 0) interval_500ms = 1;
    if (interval_500ms > 16) interval_500ms = 16;
    wdog_interval = interval_500ms - 1;
    
    /* Configure mode: reset or IRQ only */
    if (reset_on_timeout) {
        wdog_write_reg(WDOG_CFG, WDOG_CFG_SYS_RESET);
    } else {
        wdog_write_reg(WDOG_CFG, WDOG_CFG_IRQ_ONLY);
        
        /* Enable IRQ */
        wdog_write_reg(WDOG_IRQ_EN, BIT(0));
        irq_register(IRQ_DSP_WDOG, wdog_irq_handler, NULL);
        irq_enable_interrupt(IRQ_DSP_WDOG);
    }
    
    /* Clear any pending interrupt */
    wdog_write_reg(WDOG_IRQ_STA, BIT(0));
}

void watchdog_start(void)
{
    uint32_t mode = WDOG_MODE_EN | WDOG_MODE_INTV(wdog_interval);
    wdog_write_reg(WDOG_MODE, mode);
    
    /* Initial kick to start countdown */
    watchdog_kick();
}

void watchdog_stop(void)
{
    wdog_write_reg(WDOG_MODE, 0);
}

void watchdog_kick(void)
{
    /* Write key + restart bit to reset countdown */
    wdog_write_reg(WDOG_CTRL, WDOG_CTRL_KEY | WDOG_CTRL_RESTART);
}

void watchdog_set_handler(irq_handler_t handler, void *arg)
{
    wdog_handler = handler;
    wdog_handler_arg = arg;
}
