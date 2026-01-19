// Msgbox functions on hifi4.
//
// Copyright (C) 2025  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <hal.h>
#include "com.h" // sharespace_consume
#include "sched.h" // DECL_INIT

static uint16_t msgbox_new_msg[2];

static void
msgbox_rx_callback(uint8_t channel, uint32_t message, void *arg)
{
    msgbox_new_msg[0] = (uint16_t)((message << 16) >> 16);
    msgbox_new_msg[1] = (uint16_t)(message >> 16);
    sharespace_notify_consume();
}

void
msgbox_hw_init(void)
{
    msgbox_init();
    msgbox_set_rx_callback(MSGBOX_CHANNEL_3, msgbox_rx_callback, NULL);
    msgbox_enable_rx_irq(MSGBOX_CHANNEL_3);
}
DECL_INIT(msgbox_hw_init);

void msgbox_send_signal(uint32_t data)
{
    msgbox_send(MSGBOX_CHANNEL_3, data);
}
