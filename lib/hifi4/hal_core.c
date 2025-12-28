/**
 * @file hal_core.c
 * @brief Core HAL implementation for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * This file provides interrupt handling infrastructure and core HAL functions.
 * 
 * Note: The HiFi4 DSP uses Xtensa-specific interrupt handling mechanisms.
 * This implementation provides a generic interrupt dispatch mechanism that
 * you'll need to connect to the Xtensa interrupt vector.
 */

#include "hal.h"
#include <string.h>

/*============================================================================
 * HiFi4 DSP Specific Definitions
 *============================================================================*/

/* 
 * The HiFi4 DSP on R528/T113 has its own interrupt controller.
 * External interrupts are routed through the DSP subsystem.
 * 
 * DSP Interrupt Controller base address
 * Note: This may vary - check your specific configuration
 */
#define DSP_INTC_BASE       0x01700000UL

/* DSP Interrupt Controller Register offsets */
#define DSP_INTC_PEND0      0x00    /* Interrupt Pending 0 (IRQ 0-31) */
#define DSP_INTC_PEND1      0x04    /* Interrupt Pending 1 (IRQ 32-63) */
#define DSP_INTC_PEND2      0x08    /* Interrupt Pending 2 (IRQ 64-95) */
#define DSP_INTC_PEND3      0x0C    /* Interrupt Pending 3 (IRQ 96-127) */
#define DSP_INTC_EN0        0x20    /* Interrupt Enable 0 (IRQ 0-31) */
#define DSP_INTC_EN1        0x24    /* Interrupt Enable 1 (IRQ 32-63) */
#define DSP_INTC_EN2        0x28    /* Interrupt Enable 2 (IRQ 64-95) */
#define DSP_INTC_EN3        0x2C    /* Interrupt Enable 3 (IRQ 96-127) */
#define DSP_INTC_MASK0      0x40    /* Interrupt Mask 0 (IRQ 0-31) */
#define DSP_INTC_MASK1      0x44    /* Interrupt Mask 1 (IRQ 32-63) */
#define DSP_INTC_MASK2      0x48    /* Interrupt Mask 2 (IRQ 64-95) */
#define DSP_INTC_MASK3      0x4C    /* Interrupt Mask 3 (IRQ 96-127) */

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Interrupt handler table */
static struct {
    irq_handler_t handler;
    void *arg;
} irq_table[MAX_IRQ_NUM];

/* Global interrupt enable flag */
static volatile bool global_irq_enabled = false;

/*============================================================================
 * Xtensa-specific Interrupt Primitives
 *============================================================================*/

/* 
 * Xtensa-specific functions - these need to be implemented using
 * Xtensa intrinsics or assembly. Provided here as stubs.
 */

static inline uint32_t xtensa_get_intenable(void)
{
    uint32_t val;
    __asm__ volatile("rsr.intenable %0" : "=a"(val));
    return val;
}

static inline void xtensa_set_intenable(uint32_t val)
{
    __asm__ volatile("wsr.intenable %0" :: "a"(val));
    __asm__ volatile("rsync");
}

static inline uint32_t xtensa_get_ps(void)
{
    uint32_t val;
    __asm__ volatile("rsr.ps %0" : "=a"(val));
    return val;
}

static inline void xtensa_set_ps(uint32_t val)
{
    __asm__ volatile("wsr.ps %0" :: "a"(val));
    __asm__ volatile("rsync");
}

/* PS register bits */
#define PS_INTLEVEL_MASK    0x0F
#define PS_INTLEVEL_SHIFT   0

/*============================================================================
 * Interrupt Management Functions
 *============================================================================*/

void irq_register(uint32_t irq_num, irq_handler_t handler, void *arg)
{
    if (irq_num >= MAX_IRQ_NUM) {
        return;
    }
    
    irq_table[irq_num].handler = handler;
    irq_table[irq_num].arg = arg;
}

void irq_unregister(uint32_t irq_num)
{
    if (irq_num >= MAX_IRQ_NUM) {
        return;
    }
    
    irq_table[irq_num].handler = NULL;
    irq_table[irq_num].arg = NULL;
}

void irq_enable_interrupt(uint32_t irq_num)
{
    if (irq_num >= MAX_IRQ_NUM) {
        return;
    }
    
    uint32_t reg_offset;
    uint32_t bit_pos = irq_num % 32;
    
    /* Determine which enable register to use */
    if (irq_num < 32) {
        reg_offset = DSP_INTC_EN0;
    } else if (irq_num < 64) {
        reg_offset = DSP_INTC_EN1;
    } else if (irq_num < 96) {
        reg_offset = DSP_INTC_EN2;
    } else {
        reg_offset = DSP_INTC_EN3;
    }
    
    /* Enable the interrupt */
    REG32(DSP_INTC_BASE + reg_offset) |= BIT(bit_pos);
}

void irq_disable_interrupt(uint32_t irq_num)
{
    if (irq_num >= MAX_IRQ_NUM) {
        return;
    }
    
    uint32_t reg_offset;
    uint32_t bit_pos = irq_num % 32;
    
    /* Determine which enable register to use */
    if (irq_num < 32) {
        reg_offset = DSP_INTC_EN0;
    } else if (irq_num < 64) {
        reg_offset = DSP_INTC_EN1;
    } else if (irq_num < 96) {
        reg_offset = DSP_INTC_EN2;
    } else {
        reg_offset = DSP_INTC_EN3;
    }
    
    /* Disable the interrupt */
    REG32(DSP_INTC_BASE + reg_offset) &= ~BIT(bit_pos);
}

void irq_global_enable(void)
{
    /* Set interrupt level to 0 (enable all interrupts) */
    uint32_t ps = xtensa_get_ps();
    ps &= ~(PS_INTLEVEL_MASK << PS_INTLEVEL_SHIFT);
    xtensa_set_ps(ps);
    global_irq_enabled = true;
}

void irq_global_disable(void)
{
    /* Set interrupt level to max (disable all interrupts) */
    uint32_t ps = xtensa_get_ps();
    ps |= (PS_INTLEVEL_MASK << PS_INTLEVEL_SHIFT);
    xtensa_set_ps(ps);
    global_irq_enabled = false;
}

/*============================================================================
 * Main Interrupt Dispatcher
 *============================================================================*/

/**
 * @brief Main interrupt handler - called from Xtensa interrupt vector
 * 
 * This function should be called from your interrupt vector handler.
 * It reads the pending interrupts and dispatches to registered handlers.
 */
void hal_irq_dispatch(void)
{
    /* Check each pending register */
    for (int reg = 0; reg < 4; reg++) {
        uint32_t pend_offset = DSP_INTC_PEND0 + reg * 4;
        uint32_t en_offset = DSP_INTC_EN0 + reg * 4;
        
        uint32_t pending = REG32(DSP_INTC_BASE + pend_offset);
        uint32_t enabled = REG32(DSP_INTC_BASE + en_offset);
        
        /* Only process enabled and pending interrupts */
        pending &= enabled;
        
        while (pending) {
            /* Find lowest set bit */
            int bit = __builtin_ctz(pending);
            uint32_t irq_num = reg * 32 + bit;
            
            /* Call handler if registered */
            if (irq_num < MAX_IRQ_NUM && irq_table[irq_num].handler) {
                irq_table[irq_num].handler(irq_num, irq_table[irq_num].arg);
            }
            
            /* Clear this bit */
            pending &= ~BIT(bit);
        }
    }
}

/*============================================================================
 * Delay Functions
 *============================================================================*/

/* 
 * Simple busy-wait delays
 * Note: These are approximate and depend on CPU clock speed.
 * For accurate timing, use a hardware timer.
 */

/* Assumed DSP clock frequency (adjust as needed) */
#define DSP_CLOCK_HZ    600000000UL  /* 600 MHz typical */

void delay_us(uint32_t us)
{
    /* Approximate cycles per microsecond */
    uint32_t cycles = (DSP_CLOCK_HZ / 1000000UL) * us;
    
    /* Simple loop - each iteration is roughly 4 cycles */
    volatile uint32_t i;
    for (i = 0; i < cycles / 4; i++) {
        __asm__ volatile("nop");
    }
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

/*============================================================================
 * HAL Initialization
 *============================================================================*/

void hal_init(void)
{
    /* Clear interrupt handler table */
    memset(irq_table, 0, sizeof(irq_table));
    
    /* 
     * Initialize DSP interrupt controller
     * Disable all interrupts initially
     */
    REG32(DSP_INTC_BASE + DSP_INTC_EN0) = 0;
    REG32(DSP_INTC_BASE + DSP_INTC_EN1) = 0;
    REG32(DSP_INTC_BASE + DSP_INTC_EN2) = 0;
    REG32(DSP_INTC_BASE + DSP_INTC_EN3) = 0;
    
    /* Clear all pending interrupts */
    REG32(DSP_INTC_BASE + DSP_INTC_PEND0) = 0xFFFFFFFF;
    REG32(DSP_INTC_BASE + DSP_INTC_PEND1) = 0xFFFFFFFF;
    REG32(DSP_INTC_BASE + DSP_INTC_PEND2) = 0xFFFFFFFF;
    REG32(DSP_INTC_BASE + DSP_INTC_PEND3) = 0xFFFFFFFF;
    
    /* Enable global interrupts */
    irq_global_enable();
}

/*============================================================================
 * Debug/Utility Functions
 *============================================================================*/

/**
 * @brief Print a hexadecimal number via UART0
 * @param value Value to print
 */
void hal_debug_hex(uint32_t value)
{
    static const char hex_chars[] = "0123456789ABCDEF";
    char buf[11];
    
    buf[0] = '0';
    buf[1] = 'x';
    
    for (int i = 0; i < 8; i++) {
        buf[9 - i] = hex_chars[value & 0xF];
        value >>= 4;
    }
    buf[10] = '\0';
    
    uart_puts(UART_0, buf);
}

/**
 * @brief Print a string via UART0
 * @param str String to print
 */
void hal_debug_print(const char *str)
{
    uart_puts(UART_0, str);
}
