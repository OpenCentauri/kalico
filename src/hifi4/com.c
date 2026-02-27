// Support for serial port over sharespace with msgbox interrupts
//
// Copyright (C) 2026  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "util.h" // memmove
#include "board/misc.h" // console_sendf
#include "board/pgm.h" // READP
#include "command.h" // DECL_CONSTANT
#include "sched.h" // sched_wake_tasks
#include "com.h" // msgbox_enable_tx_irq
#include "rpmsg.h"

#define RX_BUFFER_SIZE 192
#define TX_BUFFER_SIZE 192

static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
static uint8_t transmit_buf[TX_BUFFER_SIZE], transmit_pos;

DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);
DECL_CONSTANT("TRANSMIT_WINDOW", TX_BUFFER_SIZE);

// RPMsg endpoint — set by rpmsg_transport_init()
static struct rpmsg_endpoint *klipper_ept;

void
com_set_endpoint(struct rpmsg_endpoint *ept)
{
    klipper_ept = ept;
}

/****************************************************************
 * Message block reading
 ****************************************************************/

static struct task_wake rpmsg_consume_wake;

// Called from RPMsg endpoint callback (interrupt context)
void
rpmsg_notify_rx(const void *data, uint32_t len)
{
    // Copy incoming RPMsg payload into the receive buffer
    uint_fast8_t rpos = receive_pos;
    uint_fast8_t space = sizeof(receive_buf) - rpos;

    if (len > space)
        len = space;  // Drop excess if buffer is full

    if (len > 0) {
        memcpy(&receive_buf[rpos], data, len);
        receive_pos = rpos + len;
    }

    sched_wake_task(&rpmsg_consume_wake);
}

// Process any incoming commands
void
rpmsg_consume_task(void)
{
    if (!sched_check_wake(&rpmsg_consume_wake))
        return;

    uint_fast8_t rpos = receive_pos, pop_count;

    // Process a message block
    int_fast8_t ret = command_find_and_dispatch(receive_buf, rpos, &pop_count);
    if (ret) {
        // Move buffer
        uint_fast8_t needcopy = rpos - pop_count;
        if (needcopy)
            memmove(receive_buf, &receive_buf[pop_count], needcopy);
        rpos = needcopy;
    }
    receive_pos = rpos;

    if (rpos)
        sched_wake_task(&rpmsg_consume_wake);
}
DECL_TASK(rpmsg_consume_task);

/****************************************************************
 * Message block sending
 ****************************************************************/

static struct task_wake rpmsg_send_wake;

void
rpmsg_notify_send(void)
{
    sched_wake_task(&rpmsg_send_wake);
}

void
rpmsg_send_task(void)
{
    if (!sched_check_wake(&rpmsg_send_wake))
        return;

    uint_fast8_t tpos = transmit_pos;
    if (!tpos || !klipper_ept)
        return;

    // Don't attempt to send until Linux has connected
    if (klipper_ept->dst == RPMSG_ADDR_ANY)
        return;  // Drop data — Linux hasn't opened the tty yet

    // Send the entire pending buffer as one RPMsg message
    int ret = rpmsg_sendto(klipper_ept, transmit_buf, tpos);
    if (ret < 0) {
        // No TX buffer available — retry later
        sched_wake_task(&rpmsg_send_wake);
        return;
    }

    // All data sent successfully
    transmit_pos = 0;
}
DECL_TASK(rpmsg_send_task);

void
console_sendf(const struct command_encoder *ce, va_list args)
{
    // Verify space for message
    uint_fast8_t tpos = transmit_pos, max_size = READP(ce->max_size);
    if (tpos + max_size > sizeof(transmit_buf))
        // Not enough space for message
        return;

    // Generate message
    uint8_t *buf = &transmit_buf[tpos];
    uint_fast8_t msglen = command_encode_and_frame(buf, ce, args);

    // Start message transmit
    transmit_pos = tpos + msglen;
    rpmsg_notify_send();
}
