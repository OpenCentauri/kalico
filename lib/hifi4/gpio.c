/**
 * @file gpio.c
 * @brief GPIO driver implementation for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 */

#include "hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* GPIO interrupt handlers and arguments */
static struct {
    irq_handler_t handler;
    void *arg;
} gpio_irq_handlers[7][32]; /* Up to 7 ports, 32 pins each */

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline uint32_t gpio_port_base(gpio_port_t port)
{
    return PIO_BASE + GPIO_PORT_OFFSET(port);
}

static inline uint32_t gpio_eint_base(gpio_port_t port)
{
    return GPIO_EINT_BASE(port);
}

/*============================================================================
 * GPIO Mode Configuration
 *============================================================================*/

void gpio_set_mode(gpio_pin_t pin, gpio_mode_t mode)
{
    uint32_t base = gpio_port_base(pin.port);
    uint32_t reg_offset = GPIO_CFG0 + (pin.pin / 8) * 4;
    uint32_t bit_offset = (pin.pin % 8) * 4;
    uint32_t mask = 0x0F << bit_offset;
    
    uint32_t val = REG32(base + reg_offset);
    val &= ~mask;
    val |= ((uint32_t)mode << bit_offset);
    REG32(base + reg_offset) = val;
}

void gpio_set_pull(gpio_pin_t pin, gpio_pull_t pull)
{
    uint32_t base = gpio_port_base(pin.port);
    uint32_t reg_offset = GPIO_PULL0 + (pin.pin / 16) * 4;
    uint32_t bit_offset = (pin.pin % 16) * 2;
    uint32_t mask = 0x03 << bit_offset;
    
    uint32_t val = REG32(base + reg_offset);
    val &= ~mask;
    val |= ((uint32_t)pull << bit_offset);
    REG32(base + reg_offset) = val;
}

/*============================================================================
 * GPIO Data Access
 *============================================================================*/

void gpio_write(gpio_pin_t pin, bool value)
{
    uint32_t base = gpio_port_base(pin.port);
    uint32_t dat = REG32(base + GPIO_DAT);
    
    if (value) {
        dat |= BIT(pin.pin);
    } else {
        dat &= ~BIT(pin.pin);
    }
    
    REG32(base + GPIO_DAT) = dat;
}

bool gpio_read(gpio_pin_t pin)
{
    uint32_t base = gpio_port_base(pin.port);
    return (REG32(base + GPIO_DAT) & BIT(pin.pin)) != 0;
}

void gpio_toggle(gpio_pin_t pin)
{
    uint32_t base = gpio_port_base(pin.port);
    REG32(base + GPIO_DAT) ^= BIT(pin.pin);
}

/*============================================================================
 * GPIO Interrupt Functions
 *============================================================================*/

void gpio_irq_configure(gpio_pin_t pin, gpio_irq_trigger_t trigger,
                        irq_handler_t handler, void *arg)
{
    /* Set pin to external interrupt mode */
    gpio_set_mode(pin, GPIO_MODE_EINT);
    
    /* Configure trigger type */
    uint32_t base = gpio_eint_base(pin.port);
    uint32_t reg_offset = GPIO_EINT_CFG0 + (pin.pin / 8) * 4;
    uint32_t bit_offset = (pin.pin % 8) * 4;
    uint32_t mask = 0x0F << bit_offset;
    
    uint32_t val = REG32(base + reg_offset);
    val &= ~mask;
    val |= ((uint32_t)trigger << bit_offset);
    REG32(base + reg_offset) = val;
    
    /* Store handler */
    gpio_irq_handlers[pin.port][pin.pin].handler = handler;
    gpio_irq_handlers[pin.port][pin.pin].arg = arg;
}

void gpio_irq_enable(gpio_pin_t pin)
{
    uint32_t base = gpio_eint_base(pin.port);
    REG32(base + GPIO_EINT_CTL) |= BIT(pin.pin);
}

void gpio_irq_disable(gpio_pin_t pin)
{
    uint32_t base = gpio_eint_base(pin.port);
    REG32(base + GPIO_EINT_CTL) &= ~BIT(pin.pin);
}

void gpio_irq_clear(gpio_pin_t pin)
{
    uint32_t base = gpio_eint_base(pin.port);
    /* Write 1 to clear the interrupt pending bit */
    REG32(base + GPIO_EINT_STATUS) = BIT(pin.pin);
}

/*============================================================================
 * GPIO Interrupt Handler (called from main IRQ handler)
 *============================================================================*/

/**
 * @brief Handle GPIO external interrupts for a specific port
 * @param port GPIO port that generated the interrupt
 */
void gpio_irq_handler(gpio_port_t port)
{
    uint32_t base = gpio_eint_base(port);
    uint32_t status = REG32(base + GPIO_EINT_STATUS);
    uint32_t enabled = REG32(base + GPIO_EINT_CTL);
    
    /* Process only enabled and pending interrupts */
    uint32_t pending = status & enabled;
    
    while (pending) {
        /* Find the lowest set bit */
        int pin = __builtin_ctz(pending);
        
        /* Clear the interrupt first */
        REG32(base + GPIO_EINT_STATUS) = BIT(pin);
        
        /* Call the handler if registered */
        if (gpio_irq_handlers[port][pin].handler) {
            gpio_irq_handlers[port][pin].handler(
                (port << 8) | pin, /* Encode port and pin in irq number */
                gpio_irq_handlers[port][pin].arg
            );
        }
        
        /* Clear this bit from pending */
        pending &= ~BIT(pin);
    }
}

/*============================================================================
 * Port-specific Interrupt Handlers
 *============================================================================*/

void gpio_pb_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    gpio_irq_handler(GPIO_PORT_B);
}

void gpio_pc_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    gpio_irq_handler(GPIO_PORT_C);
}

void gpio_pd_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    gpio_irq_handler(GPIO_PORT_D);
}

void gpio_pe_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    gpio_irq_handler(GPIO_PORT_E);
}

void gpio_pf_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    gpio_irq_handler(GPIO_PORT_F);
}

void gpio_pg_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    gpio_irq_handler(GPIO_PORT_G);
}

/*============================================================================
 * GPIO Initialization
 *============================================================================*/

void gpio_init(void)
{
    /* Register port interrupt handlers */
    intc_register(INTC_GPIOB_NS, gpio_pb_irq_handler, NULL);
    intc_register(INTC_GPIOC_NS, gpio_pc_irq_handler, NULL);
    intc_register(INTC_GPIOD_NS, gpio_pd_irq_handler, NULL);
    intc_register(INTC_GPIOE_NS, gpio_pe_irq_handler, NULL);
    intc_register(INTC_GPIOF_NS, gpio_pf_irq_handler, NULL);
    intc_register(INTC_GPIOG_NS, gpio_pg_irq_handler, NULL);
    
    /* Enable port interrupts at the system level */
    intc_enable(INTC_GPIOB_NS);
    intc_enable(INTC_GPIOC_NS);
    intc_enable(INTC_GPIOD_NS);
    intc_enable(INTC_GPIOE_NS);
    intc_enable(INTC_GPIOF_NS);
    intc_enable(INTC_GPIOG_NS);
}
