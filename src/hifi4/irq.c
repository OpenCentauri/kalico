// Interrupt functions on hifi4.
//
// Copyright (C) 2025  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <hal.h>
#include "generic/irq.h" // irqstatus_t

// Disable hardware interrupts
void
irq_disable(void)
{
    irq_global_disable();
}

// Enable hardware interrupts
void
irq_enable(void)
{
    irq_global_enable();
}

// Disable hardware interrupts in not already disabled
irqstatus_t
irq_save(void)
{
    irqstatus_t flag = hal_get_intenable();
    irq_disable();
    return flag;
}

// Restore hardware interrupts to state from flag returned by irq_save()
void irq_restore(irqstatus_t flag)
{
    hal_set_intenable(flag);
}

// Atomically enable hardware interrupts and sleep processor until next irq
void irq_wait(void)
{
    irq_global_enable();
    __asm__ volatile("waiti 0");
    irq_global_disable();
}

// Check if an interrupt is active (used only on architectures that do
// not have hardware interrupts)
void irq_poll(void)
{
}
