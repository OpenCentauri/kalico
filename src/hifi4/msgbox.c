// Msgbox functions on hifi4.
//
// Copyright (C) 2025  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <hal.h>
#include "autoconf.h" // Include configuration header
#include "board/serial_irq.h" // serial_get_tx_byte
#include "sched.h" // DECL_INIT

static void
msgbox_rx_callback(uint8_t channel, uint32_t message, void *arg)
{
    serial_rx_byte(message);
}

static void
msgbox_tx_callback(uint8_t channel, void *arg)
{
    for (;;) {
        if (!msgbox_tx_ready(channel)) {
            // Output fifo full - enable tx irq
            msgbox_enable_tx_irq(channel);
            break;
        }
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret) {
            // No more data to send - disable tx irq
            msgbox_disable_tx_irq(channel);
            break;
        }
        msgbox_send(channel, data);
    }
}

void
msgbox_hw_init(void)
{
    msgbox_init();
    msgbox_set_rx_callback(MSGBOX_ARM_TO_DSP_CH0, msgbox_rx_callback, NULL);
    msgbox_enable_rx_irq(MSGBOX_ARM_TO_DSP_CH0);

    msgbox_set_tx_callback(MSGBOX_DSP_TO_ARM_CH0, msgbox_tx_callback, NULL);
}
DECL_INIT(msgbox_hw_init);

void
serial_enable_tx_irq(void)
{
    msgbox_tx_callback(MSGBOX_DSP_TO_ARM_CH0, NULL);
}
