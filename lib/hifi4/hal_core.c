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
 * The HiFi4 DSP on R528/T113 has a two-level interrupt structure:
 * 
 * 1. Direct DSP core interrupts (directly connected to Xtensa interrupt inputs):
 *    - These include TIMER0, TIMER1, HSTIMER0, HSTIMER1, GPADC, etc.
 *    - These are handled directly by enabling the Xtensa INTENABLE bits
 * 
 * 2. DSP_INTC (external interrupt controller at 0x01700800):
 *    - Routes many peripheral interrupts to a single DSP core interrupt (IRQ 17)
 *    - Includes UART, TWI, SPI, GPIO, MSGBOX, etc.
 *    - When IRQ 17 fires, we must check DSP_INTC to find the source
 */

/* DSP Configuration registers */
#define DSP_CFG_BASE        0x01700000UL

/* DSP Interrupt Controller base address */
#define DSP_INTC_BASE       0x01700800UL

/* DSP_INTC Register offsets (for peripheral interrupts) */
#define DSP_INTC_VECTOR     0x00
#define DSP_INTC_BASE_ADDR  0x04
#define DSP_INTC_CTRL       0x0C
#define DSP_INTC_PEND0      0x10    /* Interrupt Pending 0 (IRQ 0-31) */
#define DSP_INTC_PEND1      0x14    /* Interrupt Pending 1 (IRQ 32-63) */
#define DSP_INTC_PEND2      0x18    /* Interrupt Pending 2 (IRQ 64-95) */
#define DSP_INTC_EN0        0x40    /* Interrupt Enable 0 (IRQ 0-31) */
#define DSP_INTC_EN1        0x44    /* Interrupt Enable 1 (IRQ 32-63) */
#define DSP_INTC_EN2        0x48    /* Interrupt Enable 2 (IRQ 64-95) */
#define DSP_INTC_MASK0      0x50    /* Interrupt Mask 0 (IRQ 0-31) */
#define DSP_INTC_MASK1      0x54    /* Interrupt Mask 1 (IRQ 32-63) */
#define DSP_INTC_MASK2      0x58    /* Interrupt Mask 2 (IRQ 64-95) */

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Handler table for direct DSP core interrupts (IRQ 0-31) */
static struct {
    irq_handler_t handler;
    void *arg;
} irq_table[MAX_IRQ_NUM];

/* Handler table for DSP_INTC peripheral interrupts (0-127) */
static struct {
    irq_handler_t handler;
    void *arg;
} intc_table[MAX_INTC_NUM];

/* Global interrupt enable flag */
static volatile bool global_irq_enabled = false;

/*============================================================================
 * Xtensa-specific Interrupt Primitives
 *============================================================================*/

/* 
 * Xtensa-specific functions for Call0 ABI
 * These work with both windowed and Call0 ABI
 */

static inline uint32_t xtensa_get_intenable(void)
{
    uint32_t val;
    __asm__ volatile("rsr.intenable %0" : "=a"(val));
    return val;
}

static inline void xtensa_set_intenable(uint32_t val)
{
    __asm__ volatile("wsr.intenable %0; rsync" :: "a"(val));
}

static inline uint32_t xtensa_get_ps(void)
{
    uint32_t val;
    __asm__ volatile("rsr.ps %0" : "=a"(val));
    return val;
}

static inline void xtensa_set_ps(uint32_t val)
{
    __asm__ volatile("wsr.ps %0; rsync" :: "a"(val));
}

static inline uint32_t xtensa_get_interrupt(void)
{
    uint32_t val;
    __asm__ volatile("rsr.interrupt %0" : "=a"(val));
    return val;
}

static inline void xtensa_clear_interrupt(uint32_t mask)
{
    __asm__ volatile("wsr.intclear %0; rsync" :: "a"(mask));
}

/* PS register bits */
#define PS_INTLEVEL_MASK    0x0F
#define PS_INTLEVEL_SHIFT   0
#define PS_EXCM             0x10    /* Exception mode */
#define PS_UM               0x20    /* User mode */

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

/**
 * @brief Register handler for DSP_INTC peripheral interrupt
 */
void intc_register(uint32_t intc_num, irq_handler_t handler, void *arg)
{
    if (intc_num >= MAX_INTC_NUM) {
        return;
    }
    
    intc_table[intc_num].handler = handler;
    intc_table[intc_num].arg = arg;
}

/**
 * @brief Unregister DSP_INTC peripheral interrupt handler
 */
void intc_unregister(uint32_t intc_num)
{
    if (intc_num >= MAX_INTC_NUM) {
        return;
    }
    
    intc_table[intc_num].handler = NULL;
    intc_table[intc_num].arg = NULL;
}

/**
 * @brief Enable a direct DSP core interrupt
 * 
 * For direct interrupts like TIMER0, TIMER1, HSTIMER0, HSTIMER1, GPADC, etc.
 * These are directly connected to the Xtensa interrupt inputs.
 */
void irq_enable_interrupt(uint32_t irq_num)
{
    if (irq_num >= MAX_IRQ_NUM) {
        return;
    }
    
    /* Enable in Xtensa INTENABLE register */
    uint32_t intenable = xtensa_get_intenable();
    intenable |= BIT(irq_num);
    xtensa_set_intenable(intenable);
}

void irq_disable_interrupt(uint32_t irq_num)
{
    if (irq_num >= MAX_IRQ_NUM) {
        return;
    }
    
    /* Disable in Xtensa INTENABLE register */
    uint32_t intenable = xtensa_get_intenable();
    intenable &= ~BIT(irq_num);
    xtensa_set_intenable(intenable);
}

/**
 * @brief Enable a DSP_INTC peripheral interrupt
 * 
 * For peripheral interrupts like UART, TWI, SPI, GPIO, MSGBOX, etc.
 * These go through the DSP_INTC and trigger IRQ_INTC (17) on the core.
 */
void intc_enable(uint32_t intc_num)
{
    if (intc_num >= MAX_INTC_NUM) {
        return;
    }
    
    uint32_t reg_offset;
    uint32_t bit_pos = intc_num % 32;
    
    /* Determine which enable register to use */
    if (intc_num < 32) {
        reg_offset = DSP_INTC_EN0;
    } else if (intc_num < 64) {
        reg_offset = DSP_INTC_EN1;
    } else {
        reg_offset = DSP_INTC_EN2;
    }
    
    /* Enable the interrupt in DSP_INTC */
    REG32(DSP_INTC_BASE + reg_offset) |= BIT(bit_pos);
    
    /* Make sure IRQ_INTC is enabled at the core level */
    irq_enable_interrupt(IRQ_INTC);
}

void intc_disable(uint32_t intc_num)
{
    if (intc_num >= MAX_INTC_NUM) {
        return;
    }
    
    uint32_t reg_offset;
    uint32_t bit_pos = intc_num % 32;
    
    /* Determine which enable register to use */
    if (intc_num < 32) {
        reg_offset = DSP_INTC_EN0;
    } else if (intc_num < 64) {
        reg_offset = DSP_INTC_EN1;
    } else {
        reg_offset = DSP_INTC_EN2;
    }
    
    /* Disable the interrupt in DSP_INTC */
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
 * Debug Functions
 *============================================================================*/

/**
 * @brief Get current INTENABLE register value
 */
uint32_t hal_get_intenable(void)
{
    return xtensa_get_intenable();
}

/**
 * @brief Set INTENABLE register
 */
void hal_set_intenable(uint32_t irqs)
{
    return xtensa_set_intenable(irqs);
}

/**
 * @brief Get current INTERRUPT register value (pending interrupts)
 */
uint32_t hal_get_interrupt(void)
{
    return xtensa_get_interrupt();
}

/**
 * @brief Get current PS register value
 */
uint32_t hal_get_ps(void)
{
    return xtensa_get_ps();
}

/**
 * @brief Manually trigger a software interrupt for testing
 * @param irq_num Interrupt number to trigger
 */
void hal_trigger_soft_interrupt(uint32_t irq_num)
{
    uint32_t mask = BIT(irq_num);
    __asm__ volatile("wsr.intset %0; rsync" :: "a"(mask));
}

/*============================================================================
 * Main Interrupt Dispatcher
 *============================================================================*/

/**
 * @brief Dispatch DSP_INTC peripheral interrupts
 * 
 * Called when IRQ_INTC (17) fires. Checks DSP_INTC pending registers
 * to find which peripheral(s) triggered the interrupt.
 */
static void dispatch_intc_interrupts(void)
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
            uint32_t intc_num = reg * 32 + bit;
            
            /* Call handler if registered */
            if (intc_num < MAX_INTC_NUM && intc_table[intc_num].handler) {
                intc_table[intc_num].handler(intc_num, intc_table[intc_num].arg);
            }
            
            /* Clear this bit in our local copy */
            pending &= ~BIT(bit);
        }
    }
}

/**
 * @brief Main interrupt handler - called from Xtensa interrupt vector
 * 
 * This function is called from the interrupt handler in startup.S.
 * It reads the Xtensa INTERRUPT register to find pending interrupts
 * and dispatches to registered handlers.
 */

void hal_irq_dispatch(void) __attribute__((used));

void hal_irq_dispatch(void)
{
    /* Get pending interrupts from Xtensa core */
    uint32_t pending = xtensa_get_interrupt();
    uint32_t enabled = xtensa_get_intenable();
    
    /* Only process enabled and pending interrupts */
    pending &= enabled;
    
    while (pending) {
        /* Find lowest set bit */
        int irq = __builtin_ctz(pending);
        
        if (irq == IRQ_INTC) {
            /* This is the INTC interrupt - dispatch peripheral interrupts */
            dispatch_intc_interrupts();
        } else {
            /* Direct DSP core interrupt - call registered handler */
            if ((uint32_t)irq < MAX_IRQ_NUM && irq_table[irq].handler) {
                irq_table[irq].handler(irq, irq_table[irq].arg);
            }
        }
        
        /* Clear this interrupt if it's edge-triggered 
         * Note: Level-triggered interrupts must be cleared at the source
         */
        xtensa_clear_interrupt(BIT(irq));
        
        /* Clear this bit in our local copy */
        pending &= ~BIT(irq);
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
    /* Clear interrupt handler tables */
    memset(irq_table, 0, sizeof(irq_table));
    memset(intc_table, 0, sizeof(intc_table));
    
    /* Disable all Xtensa core interrupt inputs */
    xtensa_set_intenable(0);
    
    /* 
     * Initialize DSP_INTC (peripheral interrupt controller)
     * Disable all peripheral interrupts initially
     */
    REG32(DSP_INTC_BASE + DSP_INTC_EN0) = 0;
    REG32(DSP_INTC_BASE + DSP_INTC_EN1) = 0;
    REG32(DSP_INTC_BASE + DSP_INTC_EN2) = 0;

    REG32(DSP_INTC_BASE + DSP_INTC_MASK0) = 0;
    REG32(DSP_INTC_BASE + DSP_INTC_MASK1) = 0;
    REG32(DSP_INTC_BASE + DSP_INTC_MASK2) = 0;
    
    /* Clear all pending interrupts in DSP_INTC */
    /* Note: Writing 1 to clear pending bits */
    REG32(DSP_INTC_BASE + DSP_INTC_PEND0) = 0xFFFFFFFF;
    REG32(DSP_INTC_BASE + DSP_INTC_PEND1) = 0xFFFFFFFF;
    REG32(DSP_INTC_BASE + DSP_INTC_PEND2) = 0xFFFFFFFF;
    
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
