// Watchdog code on hifi4
//
// Copyright (C) 2026  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint32_t
#include <hal.h> // watchdog_init
#include "sched.h" // DECL_TASK
#include "command.h"    // shutdown
#include "log.h"

// void
// command_reset(uint32_t *args)
// {
//     // Disable all interrupts, including HSTIMER so scheduler doesn't run
//     irq_global_disable();
//     // Wait for watchdog to reset us
//     __asm__ volatile("waiti 0");
// }
// DECL_COMMAND_FLAGS(command_reset, HF_IN_SHUTDOWN, "reset");

void
watchdog_hw_reset(void)
{
    watchdog_kick();
}
DECL_TASK(watchdog_hw_reset);

// Uncomment for software watchdog
// static void WDOG_IRQHandler(uint32_t irq, void *arg)
// {
//     hal_restart();
// }

void
watchdog_hw_init(void)
{
    lprintf("watchdog_hw_init!!!!!!!!!!!\n");
    // Uncomment for software watchdog
    // watchdog_set_handler(WDOG_IRQHandler, NULL);
    // watchdog_init(WDOG_INTV_0_5_SEC, false);
    watchdog_init(WDOG_INTV_1_SEC, true);
    watchdog_start();
}
DECL_INIT(watchdog_hw_init);
