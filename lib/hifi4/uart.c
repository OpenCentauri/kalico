/**
 * @file uart.c
 * @brief UART driver implementation for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * This implements a 16550-compatible UART driver with interrupt support.
 */

#include "hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* UART base addresses */
static const uint32_t uart_bases[6] = {
    UART0_BASE,
    UART1_BASE,
    UART2_BASE,
    UART3_BASE,
    UART4_BASE,
    UART5_BASE,
};

/* UART interrupt handlers */
static struct {
    irq_handler_t rx_handler;
    irq_handler_t tx_handler;
    void *arg;
} uart_handlers[4];

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline uint32_t uart_base(uart_id_t id)
{
    return uart_bases[id];
}

static inline void uart_write_reg(uart_id_t id, uint32_t offset, uint32_t value)
{
    REG32(uart_base(id) + offset) = value;
}

static inline uint32_t uart_read_reg(uart_id_t id, uint32_t offset)
{
    return REG32(uart_base(id) + offset);
}

/*============================================================================
 * UART Status Functions
 *============================================================================*/

bool uart_rx_ready(uart_id_t uart_id)
{
    return (uart_read_reg(uart_id, UART_LSR) & UART_LSR_DR) != 0;
}

bool uart_tx_ready(uart_id_t uart_id)
{
    return (uart_read_reg(uart_id, UART_LSR) & UART_LSR_THRE) != 0;
}

/*============================================================================
 * UART Blocking I/O
 *============================================================================*/

void uart_putc(uart_id_t uart_id, uint8_t data)
{
    /* Wait for transmit holding register to be empty */
    while (!uart_tx_ready(uart_id)) {
        /* Busy wait */
    }
    
    uart_write_reg(uart_id, UART_THR, data);
}

uint8_t uart_getc(uart_id_t uart_id)
{
    /* Wait for data to be available */
    while (!uart_rx_ready(uart_id)) {
        /* Busy wait */
    }
    
    return (uint8_t)uart_read_reg(uart_id, UART_RBR);
}

void uart_puts(uart_id_t uart_id, const char *str)
{
    while (*str) {
        if (*str == '\n') {
            uart_putc(uart_id, '\r');
        }
        uart_putc(uart_id, *str++);
    }
}

void uart_write(uart_id_t uart_id, const uint8_t *data, uint32_t len)
{
    while (len--) {
        uart_putc(uart_id, *data++);
    }
}

void uart_read(uart_id_t uart_id, uint8_t *data, uint32_t len)
{
    while (len--) {
        *data++ = uart_getc(uart_id);
    }
}

/*============================================================================
 * UART Interrupt Handling
 *============================================================================*/

static void uart_irq_handler_internal(uart_id_t uart_id)
{
    uint32_t iir = uart_read_reg(uart_id, UART_IIR);
    
    /* Check if there's actually an interrupt pending */
    if (iir & UART_IIR_NO_INT) {
        return;
    }
    
    uint32_t int_id = iir & UART_IIR_ID_MASK;
    
    switch (int_id) {
    case UART_IIR_RDI:  /* Receive data available */
    case UART_IIR_CTI:  /* Character timeout */
        if (uart_handlers[uart_id].rx_handler) {
            uart_handlers[uart_id].rx_handler(uart_id, uart_handlers[uart_id].arg);
        } else {
            /* Discard data if no handler */
            (void)uart_read_reg(uart_id, UART_RBR);
        }
        break;
        
    case UART_IIR_THRI: /* TX holding register empty */
        if (uart_handlers[uart_id].tx_handler) {
            uart_handlers[uart_id].tx_handler(uart_id, uart_handlers[uart_id].arg);
        }
        break;
        
    case UART_IIR_RLSI: /* Receiver line status */
        /* Clear by reading LSR */
        (void)uart_read_reg(uart_id, UART_LSR);
        break;
        
    case UART_IIR_MSI: /* Modem status */
        /* Clear by reading MSR */
        (void)uart_read_reg(uart_id, UART_MSR);
        break;
        
    case UART_IIR_BUSY: /* Busy detect */
        /* Clear by reading USR */
        (void)uart_read_reg(uart_id, UART_USR);
        break;
        
    default:
        break;
    }
}

/* Individual UART interrupt handlers */
static void uart0_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    uart_irq_handler_internal(UART_0);
}

static void uart1_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    uart_irq_handler_internal(UART_1);
}

static void uart2_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    uart_irq_handler_internal(UART_2);
}

static void uart3_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    uart_irq_handler_internal(UART_3);
}

static void uart4_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    uart_irq_handler_internal(UART_4);
}

static void uart5_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    uart_irq_handler_internal(UART_5);
}

static const irq_handler_t uart_irq_handlers[6] = {
    uart0_irq_handler,
    uart1_irq_handler,
    uart2_irq_handler,
    uart3_irq_handler,
    uart4_irq_handler,
    uart5_irq_handler,
};

static const uint32_t uart_irq_nums[6] = {
    INTC_UART0,
    INTC_UART1,
    INTC_UART2,
    INTC_UART3,
    INTC_UART4,
    INTC_UART5,
};

void uart_set_rx_callback(uart_id_t uart_id, irq_handler_t callback, void *arg)
{
    uart_handlers[uart_id].rx_handler = callback;
    uart_handlers[uart_id].arg = arg;
}

void uart_set_tx_callback(uart_id_t uart_id, irq_handler_t callback, void *arg)
{
    uart_handlers[uart_id].tx_handler = callback;
    uart_handlers[uart_id].arg = arg;
}

void uart_enable_rx_irq(uart_id_t uart_id)
{
    uint32_t ier = uart_read_reg(uart_id, UART_IER);
    ier |= UART_IER_RDI;
    uart_write_reg(uart_id, UART_IER, ier);
}

void uart_disable_rx_irq(uart_id_t uart_id)
{
    uint32_t ier = uart_read_reg(uart_id, UART_IER);
    ier &= ~UART_IER_RDI;
    uart_write_reg(uart_id, UART_IER, ier);
}

void uart_enable_tx_irq(uart_id_t uart_id)
{
    uint32_t ier = uart_read_reg(uart_id, UART_IER);
    ier |= UART_IER_THRI;
    uart_write_reg(uart_id, UART_IER, ier);
}

void uart_disable_tx_irq(uart_id_t uart_id)
{
    uint32_t ier = uart_read_reg(uart_id, UART_IER);
    ier &= ~UART_IER_THRI;
    uart_write_reg(uart_id, UART_IER, ier);
}

/*============================================================================
 * UART Initialization
 *============================================================================*/

int uart_init(uart_id_t uart_id, const uart_config_t *config, uint32_t clk_freq)
{
    // uint32_t base = uart_base(uart_id);
    uint32_t divisor;
    uint32_t lcr = 0;
    
    /* Validate parameters */
    if (uart_id > UART_5 || config == NULL) {
        return -1;
    }
    
    if (config->data_bits < 5 || config->data_bits > 8) {
        return -1;
    }
    
    if (config->stop_bits < 1 || config->stop_bits > 2) {
        return -1;
    }
    
    /* Enable clock and deassert reset */
    ccu_uart_enable(uart_id);

    /* Clear handlers */
    uart_handlers[uart_id].rx_handler = NULL;
    uart_handlers[uart_id].tx_handler = NULL;
    uart_handlers[uart_id].arg = NULL;
    
    /* Calculate divisor: divisor = clk_freq / (16 * baudrate) */
    divisor = clk_freq / (config->baudrate * 16);
    if (divisor == 0 || divisor > 0xFFFF) {
        return -1;
    }
    
    /* Disable all interrupts */
    uart_write_reg(uart_id, UART_IER, 0);
    
    /* Set DLAB to access divisor latch */
    uart_write_reg(uart_id, UART_LCR, UART_LCR_DLAB);
    
    /* Set divisor */
    uart_write_reg(uart_id, UART_DLL, divisor & 0xFF);
    uart_write_reg(uart_id, UART_DLH, (divisor >> 8) & 0xFF);
    
    /* Configure line control register */
    lcr = config->data_bits - 5; /* DLS: 0=5bits, 1=6bits, 2=7bits, 3=8bits */
    
    if (config->stop_bits == 2) {
        lcr |= UART_LCR_STOP;
    }
    
    switch (config->parity) {
    case UART_PARITY_ODD:
        lcr |= UART_LCR_PEN;
        break;
    case UART_PARITY_EVEN:
        lcr |= UART_LCR_PEN | UART_LCR_EPS;
        break;
    case UART_PARITY_NONE:
    default:
        break;
    }
    
    /* Clear DLAB and set line parameters */
    uart_write_reg(uart_id, UART_LCR, lcr);
    
    /* Enable and reset FIFOs */
    uart_write_reg(uart_id, UART_FCR, 
                   UART_FCR_FIFO_EN | UART_FCR_RXSR | UART_FCR_TXSR);
    
    /* Clear pending interrupts */
    (void)uart_read_reg(uart_id, UART_LSR);
    (void)uart_read_reg(uart_id, UART_RBR);
    (void)uart_read_reg(uart_id, UART_IIR);
    (void)uart_read_reg(uart_id, UART_MSR);

    /* Register the system IRQ handler */
    intc_register(uart_irq_nums[uart_id], uart_irq_handlers[uart_id], NULL);
    intc_enable(uart_irq_nums[uart_id]);
    
    return 0;
}

/*============================================================================
 * UART Non-Blocking Operations (for use in interrupt handlers)
 *============================================================================*/

/**
 * @brief Non-blocking transmit (use in TX interrupt handler)
 * @param uart_id UART identifier
 * @param data Byte to transmit
 * @return true if byte was transmitted, false if TX FIFO full
 */
bool uart_putc_nb(uart_id_t uart_id, uint8_t data)
{
    if (uart_tx_ready(uart_id)) {
        uart_write_reg(uart_id, UART_THR, data);
        return true;
    }
    return false;
}

/**
 * @brief Non-blocking receive (use in RX interrupt handler)
 * @param uart_id UART identifier
 * @param data Pointer to store received byte
 * @return true if byte was received, false if no data available
 */
bool uart_getc_nb(uart_id_t uart_id, uint8_t *data)
{
    if (uart_rx_ready(uart_id)) {
        *data = (uint8_t)uart_read_reg(uart_id, UART_RBR);
        return true;
    }
    return false;
}

/**
 * @brief Check and get line status errors
 * @param uart_id UART identifier
 * @return LSR error bits (OE, PE, FE, BI)
 */
uint8_t uart_get_errors(uart_id_t uart_id)
{
    uint32_t lsr = uart_read_reg(uart_id, UART_LSR);
    return (uint8_t)(lsr & (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI));
}
