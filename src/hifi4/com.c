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
#include "sharespace.h" // sharespace_read
#include "log.h" // lprintf

#define RX_BUFFER_SIZE 192
#define TX_BUFFER_SIZE 128

static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
static uint8_t transmit_buf[TX_BUFFER_SIZE], transmit_pos;

DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);
DECL_CONSTANT("TRANSMIT_WINDOW", TX_BUFFER_SIZE);

/****************************************************************
 * Message block reading
 ****************************************************************/

static struct task_wake sharespace_consume_wake;

void
sharespace_notify_consume(void)
{
    sched_wake_task(&sharespace_consume_wake);
}

// Process any incoming commands
void
sharespace_consume_task(void)
{
    if (!sched_check_wake(&sharespace_consume_wake))
        return;
    uint_fast8_t rpos = receive_pos, pop_count;
    uint_fast8_t rmax = sizeof(receive_buf) - rpos;
    int_fast8_t ret = sharespace_read(&receive_buf[receive_pos], rmax);
    if (ret > 0) {
        rpos += ret;
        sharespace_notify_consume();
    }
    // Process a message block
    ret = command_find_and_dispatch(receive_buf, rpos, &pop_count);
    if (ret) {
        // Move buffer
        uint_fast8_t needcopy = rpos - pop_count;
        if (needcopy) {
            memmove(receive_buf, &receive_buf[pop_count], needcopy);
            sharespace_notify_consume();
        }
        rpos = needcopy;
    }
    receive_pos = rpos;
}
DECL_TASK(sharespace_consume_task);

/****************************************************************
 * Message block sending
 ****************************************************************/

static struct task_wake sharespace_send_wake;

void
sharespace_notify_send(void)
{
    sched_wake_task(&sharespace_send_wake);
}

void
sharespace_send_task(void)
{
    if (!sched_check_wake(&sharespace_send_wake))
        return;
    uint_fast8_t tpos = transmit_pos;
    if (!tpos)
        return;
    int_fast8_t ret = sharespace_write(transmit_buf, tpos);
    // lprintf("sharespace_send_task: ");
    // for (int i = tpos; i < tmax; i++)
    //     lprint_hex(data[i]);
    if (ret <= 0)
        return;
    uint_fast8_t needcopy = tpos - ret;
    if (needcopy) {
        memmove(transmit_buf, &transmit_buf[ret], needcopy);
        sharespace_notify_send();
    }
    transmit_pos = needcopy;
}
DECL_TASK(sharespace_send_task);

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
    sharespace_notify_send();
}
