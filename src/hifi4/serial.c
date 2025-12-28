// UART functions on hifi4.
//
// Copyright (C) 2025  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <hal.h>
#include "autoconf.h" // Include configuration header
#include "board/serial_irq.h" // serial_get_tx_byte
#include "sched.h" // DECL_INIT

// Dynamically select UART and IRQ based on configuration
#if CONFIG_HIFI4_SERIAL_UART0
    #define UARTx UART_0
#elif CONFIG_HIFI4_SERIAL_UART1
    #define UARTx UART_1
#elif CONFIG_HIFI4_SERIAL_UART2
    #define UARTx UART_2
#elif CONFIG_HIFI4_SERIAL_UART3
    #define UARTx UART_0
#endif

void
uart_rx_handler(uint32_t irq, void *arg)
{
    /* Read all available data from UART */
    while (uart_rx_ready(UARTx))
        serial_rx_byte(uart_getc(UARTx));
}

// Write tx bytes to the serial port
void
uart_tx_handler(uint32_t irq, void *arg)
{
    for (;;) {
        if (!uart_tx_ready(UARTx)) {
            // Output fifo full - enable tx irq
            uart_enable_tx_irq(UARTx);
            break;
        }
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret) {
            // No more data to send - disable tx irq
            uart_disable_tx_irq(UARTx);
            break;
        }
        uart_putc(UARTx, data);
    }
}

void
serial_init(void)
{
    uart_config_t uart_cfg = {
        .baudrate = CONFIG_SERIAL_BAUD,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = UART_PARITY_NONE,
    };
    uart_init(UARTx, &uart_cfg, CONFIG_CLOCK_FREQ);

    uart_set_rx_callback(UARTx, uart_rx_handler, NULL);
    uart_enable_rx_irq(UARTx);

    uart_set_tx_callback(UARTx, uart_tx_handler, NULL);
}
DECL_INIT(serial_init);

void
serial_enable_tx_irq(void)
{
    uart_tx_handler(UARTx, NULL);
}
