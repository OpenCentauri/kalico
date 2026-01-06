// Main entry point for hifi4.
//
// Copyright (C) 2025  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <hal.h> // hal_init
#include "sched.h" // sched_main

#include "board/serial_irq.h" // serial_get_tx_byte

void delay(int count) {
    volatile int i;
    for (i = 0; i < count; i++) {
        __asm__ __volatile__ ("nop");
    }
}

static void TIMER0_IRQHandler(uint32_t irq, void *arg)
{
   uart_puts(UART_0, "hello from timer0");
}

static void TIMER1_IRQHandler(uint32_t irq, void *arg)
{
   uart_puts(UART_0, "hello from timer1");
}

static uint32_t exccause;

int
main(void)
{
    delay(1000000);
    // serial_init();
    uart_puts(UART_0, "--------------");
    uart_puts(UART_0, "hello from dsp");
    uart_puts(UART_0, "--------------\n");

    hal_init();

    uart_puts(UART_0, "exccause: ");
    __asm__ volatile("rsr.exccause %0" : "=a"(exccause));
    static const char hex_chars[] = "0123456789ABCDEF";
    char buf[11];
    
    buf[0] = '0';
    buf[1] = 'x';
    
    for (int i = 0; i < 8; i++) {
        buf[9 - i] = hex_chars[exccause & 0xF];
        exccause >>= 4;
    }
    buf[10] = '\0';
    
    uart_puts(UART_0, buf);
    uart_puts(UART_0, "\n");

    hal_debug_hex(hal_get_intenable());
    hal_debug_hex(hal_get_interrupt());
    hal_debug_hex(hal_get_ps());
    uart_puts(UART_0, "\n");

    // uint32_t val;
    // __asm__ volatile("rsr.intenable %0" : "=a"(val));
    // uart_puts(UART_0, "intset: ");
    // hal_debug_hex(val);
    // uart_puts(UART_0, "\n");


    // unsigned int val_to_set = (1 << 18);
    // unsigned int interrupt_before, interrupt_after, intenable;

    // __asm__ volatile("rsr.interrupt %0" : "=a"(interrupt_before));
    // __asm__ volatile("rsr.intenable %0" : "=a"(intenable));

    // __asm__ volatile("wsr.intset %0; rsync" :: "a"(val_to_set));

    // __asm__ volatile("rsr.interrupt %0" : "=a"(interrupt_after));


    // uart_puts(UART_0, "interrupt_before: ");
    // hal_debug_hex(interrupt_before);
    // uart_puts(UART_0, "\n");
    // uart_puts(UART_0, "interrupt_after: ");
    // hal_debug_hex(interrupt_after);
    // uart_puts(UART_0, "\n");
    // uart_puts(UART_0, "intenable: ");
    // hal_debug_hex(intenable);
    // uart_puts(UART_0, "\n");


    // for(int i = 0; i <= 27; i++)
    //     irq_enable_interrupt(i);

    gpio_init();
    gpio_set_mode(GPIO_PIN(GPIO_PORT_G, 15), GPIO_MODE_OUTPUT);

    hstimer_init(HSTIMER_0, 0);
    hstimer_start_periodic(HSTIMER_0, 5000000, TIMER0_IRQHandler, NULL);//0xffffffff 1000000

    hstimer_init(HSTIMER_1, 0);
    hstimer_start_periodic(HSTIMER_1, 10000000, TIMER1_IRQHandler, NULL);//0xffffffff 1000000

    for (;;) {
        gpio_write(GPIO_PIN(GPIO_PORT_G, 15), 1);
        delay(1000000);
        gpio_write(GPIO_PIN(GPIO_PORT_G, 15), 0);

        uint32_t hi, lo;
        uart_puts(UART_0, "hello from dsp timer0: ");
        hstimer_get_counter(HSTIMER_0, &lo, &hi);
        hal_debug_hex(hi);
        hal_debug_hex(lo);
        hal_debug_hex(REG32(HSTIMER_BASE + HSTMR_IRQ_STA));
        hal_debug_hex(REG32(HSTIMER_BASE + HSTMR_IRQ_EN));
        hal_debug_hex(REG32(HSTIMER_BASE + HSTMR0_CTRL));
        uart_puts(UART_0, "\n");
        uart_puts(UART_0, "hello from dsp timer1: ");
        hstimer_get_counter(HSTIMER_1, &lo, &hi);
        hal_debug_hex(hi);
        hal_debug_hex(lo);
        hal_debug_hex(REG32(HSTIMER_BASE + HSTMR_IRQ_STA));
        hal_debug_hex(REG32(HSTIMER_BASE + HSTMR_IRQ_EN));
        hal_debug_hex(REG32(HSTIMER_BASE + HSTMR1_CTRL));
        uart_puts(UART_0, "\n");

        hal_debug_hex(hal_get_intenable());
        hal_debug_hex(hal_get_interrupt());
        hal_debug_hex(hal_get_ps());
        uart_puts(UART_0, "\n");

        hal_debug_hex(REG32(0x01700800UL + 0x10));
        hal_debug_hex(REG32(0x01700800UL + 0x14));
        hal_debug_hex(REG32(0x01700800UL + 0x18));
        uart_puts(UART_0, "\n");

        // if (REG32(TIMER_BASE + TMR_IRQ_STA))
        // {
        //     uart_puts(UART_0, "Triggering interrupt!\n");
        //     hal_trigger_soft_interrupt(IRQ_TIMER0);
        // }
        // uart_puts(UART_0, "\npending");
        // hal_debug_hex(REG32(0x01700800UL + 0x10));
        // hal_debug_hex(REG32(0x01700800UL + 0x14));
        // hal_debug_hex(REG32(0x01700800UL + 0x18));
        // uart_puts(UART_0, "\nenables");
        // hal_debug_hex(REG32(0x01700800UL + 0x40));
        // hal_debug_hex(REG32(0x01700800UL + 0x44));
        // hal_debug_hex(REG32(0x01700800UL + 0x48));
        // uart_puts(UART_0, "\n");
        // REG32(UART0_BASE + UART_THR) = "a";
        delay(1000000);
    }

    sched_main();
}
