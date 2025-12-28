// Timer functions for hifi4.
//
// Copyright (C) 2025  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <hal.h>
#include "timer.h"
#include "compiler.h"
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "board/timer_irq.h" // timer_dispatch_many
#include "sched.h" // DECL_INIT

static uint32_t timer1_total_times;

/****************************************************************
 * Low level timer code
 ****************************************************************/

// Hardware timer IRQ handler - dispatch software timers
void __visible __aligned(16)
TIMER0_IRQHandler(uint32_t irq, void *arg)
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
    // timer is a count down timer so we have to invert the value
    uint32_t ret = timer_get_counter(TIMER_1);
    ret = 0xffffffff - ret;
    return ret;
}

inline void
timer_set(uint32_t next)
{
    timer_start_periodic(TIMER_0, next, TIMER0_IRQHandler, NULL);
}

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    timer_set(1);
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

void timer_reset(void)
{
    if (timer_from_us(100000) <= 0xffffff)  //100ms Timer in sched.c already ensures SysTick wont overflow
        return;

    sched_add_timer(&wrap_timer);
}
DECL_SHUTDOWN(timer_reset);

/****************************************************************
 * Setup and irqs
 ****************************************************************/
static void TIMER1_IRQHandler(uint32_t irq, void *arg)
{
   timer1_total_times += 1;
}

void
timer_hw_init(void)
{
    timer1_total_times = 0;
    irqstatus_t flag = irq_save();

    timer_init(TIMER_1, TIMER_CLK_OSC24M, 0);
    timer_start_periodic(TIMER_1, 0xffffffff, TIMER1_IRQHandler, NULL);//0xffffffff 20000000
    timer_reset();

    timer_init(TIMER_0, TIMER_CLK_OSC24M, 0);
    timer_start_periodic(TIMER_0, 200000, TIMER0_IRQHandler, NULL);
    timer_kick();
    irq_restore(flag);
}
DECL_INIT(timer_hw_init);
