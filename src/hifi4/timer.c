// Timer functions for hifi4.
//
// Copyright (C) 2025  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <hal.h>
#include "timer.h"
#include "compiler.h"
#include "board/misc.h" // timer_read_time
#include "board/timer_irq.h" // timer_dispatch_many
#include "sched.h" // DECL_INIT
#include "board/irq.h" // irq_disable

void __visible __aligned(16)
CCOMPARE0_IRQHandler(uint32_t irq, void *arg)
{
    irq_disable();
    uint32_t next = timer_dispatch_many();
    timer_set(next);
    irq_enable();
}

// Return the current time (in absolute clock ticks).
uint32_t
timer_read_time(void)
{
    uint32_t val;
    __asm__ volatile("rsr.ccount %0" : "=a"(val));
    return val;
}

inline void
timer_set(uint32_t next)
{
    __asm__ volatile("wsr.ccompare0 %0; rsync" :: "a"(next));
}

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    timer_set(timer_read_time() + 500);
}

// Dummy timer to avoid scheduling a SysTick irq greater than 0xffffff
static uint_fast8_t
timer_wrap_event(struct timer *t)
{
    t->waketime += 0xffffff;
    return SF_RESCHEDULE;
}
static struct timer wrap_timer = {
    .func = timer_wrap_event,
    .waketime = 0xffffff,
};
void
timer_reset(void)
{
    if (timer_from_us(100000) <= 0xffffff)
        // Timer in sched.c already ensures SysTick wont overflow
        return;
    sched_add_timer(&wrap_timer);
}
DECL_SHUTDOWN(timer_reset);

/****************************************************************
 * Setup and irqs
 ****************************************************************/

void
timer_hw_init(void)
{
    __asm__ volatile("wsr.ccount %0; rsync" :: "a"(0)); // Reset CCOUNT to 0
    irq_register(IRQ_COMPARE0, CCOMPARE0_IRQHandler, NULL);
    timer_reset();
    timer_kick();
    irq_enable_interrupt(IRQ_COMPARE0);
}
DECL_INIT(timer_hw_init);
