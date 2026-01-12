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

static uint32_t timer1_total_times;

/****************************************************************
 * Low level timer code
 ****************************************************************/

// Hardware timer IRQ handler - dispatch software timers
void __visible __aligned(16)
HSTIMER0_IRQHandler(uint32_t irq, void *arg)
{
    uint32_t next = timer_dispatch_many();
    uint32_t now = timer_read_time();
    int32_t diff = next - now;
    timer_set(diff);
}

// Return the current time (in absolute clock ticks).
uint32_t
timer_read_time(void)
{
    // timer is a count down timer so we have to invert the value
    uint32_t hi, lo;
    hstimer_get_counter(HSTIMER_1, &lo, &hi);
    lo = 0xffffffff - lo;
    return lo;
}

inline void
timer_set(uint32_t next)
{
    hstimer_start_oneshot(HSTIMER_0, next, 0, HSTIMER0_IRQHandler, NULL);
}

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    timer_set(timer_from_us(2));
}

void timer_reset(void)
{
}
DECL_SHUTDOWN(timer_reset);

/****************************************************************
 * Setup and irqs
 ****************************************************************/
static void HSTIMER1_IRQHandler(uint32_t irq, void *arg)
{
    timer1_total_times += 1;
}

void
timer_hw_init(void)
{
    timer1_total_times = 0;

    hstimer_init(HSTIMER_1, 0);
    hstimer_start_periodic(HSTIMER_1, 0xffffffff, 0x00000000, HSTIMER1_IRQHandler, NULL);
    timer_reset();

    hstimer_init(HSTIMER_0, 0);
    hstimer_start_oneshot(HSTIMER_0, 200000, 0, HSTIMER0_IRQHandler, NULL);
    timer_kick();
}
DECL_INIT(timer_hw_init);
