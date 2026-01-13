// Generic interrupt based serial uart helper code
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memmove
#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "board/pgm.h" // READP
#include "command.h" // DECL_CONSTANT
#include "sched.h" // sched_wake_tasks
#include "com.h" // msgbox_enable_tx_irq
#include "sharespace.h" // sharespace_read
#include "log.h" // lprintf

#define RX_BUFFER_SIZE 192
#define TX_BUFFER_SIZE 96

static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
static uint8_t transmit_buf[TX_BUFFER_SIZE], transmit_pos, transmit_max;

DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);
DECL_CONSTANT("TRANSMIT_WINDOW", TX_BUFFER_SIZE);

// Rx interrupt - store read data
void
sharespace_consume(void)
{
    if (receive_pos >= sizeof(receive_buf))
        // Serial overflow - ignore it as crc error will force retransmit
        return;
    // Calculate max length of message we can recieve
    uint_fast8_t rmax = sizeof(receive_buf) - receive_pos;
    int_fast8_t ret = sharespace_read(&receive_buf[receive_pos], rmax);
    if (!ret)
        return;
    if (ret == 1 && receive_buf[receive_pos] == MESSAGE_SYNC)
    for (int i = receive_pos; i < receive_pos + ret; i++)
    {
        if (receive_buf[i] == MESSAGE_SYNC)
            sched_wake_tasks();
    }
    // lprintf("sharespace_consume: ");
    // for (int i = receive_pos; i < receive_pos + ret; i++)
    //     lprint_hex(receive_buf[i]);
    receive_pos += ret;
}

// Remove from the receive buffer the given number of bytes
static void
console_pop_input(uint_fast8_t len)
{
    uint_fast8_t copied = 0;
    for (;;) {
        uint_fast8_t rpos = readb(&receive_pos);
        uint_fast8_t needcopy = rpos - len;
        if (needcopy) {
            memmove(&receive_buf[copied], &receive_buf[copied + len]
                    , needcopy - copied);
            copied = needcopy;
            sched_wake_tasks();
        }
        irqstatus_t flag = irq_save();
        if (rpos != readb(&receive_pos)) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        receive_pos = needcopy;
        irq_restore(flag);
        break;
    }
}

// Process any incoming commands
void
console_task(void)
{
    uint_fast8_t rpos = readb(&receive_pos), pop_count;
    int_fast8_t ret = command_find_block(receive_buf, rpos, &pop_count);
    if (ret > 0)
    {
        lprintf("command_dispatch\n");
        command_dispatch(receive_buf, pop_count);
    }
    if (ret) {
        console_pop_input(pop_count);
        if (ret > 0)
        {
            lprintf("command_send_ack\n");
            command_send_ack();
        }
    }
}
DECL_TASK(console_task);

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
    uint_fast8_t tpos = readb(&transmit_pos), tmax = readb(&transmit_max);
    if (tpos >= tmax || !tmax)
        return;
    uint_fast8_t len = tmax - tpos;
    uint8_t *data = &transmit_buf[tpos];
    int_fast8_t ret = sharespace_write(data, len);
    // lprintf("sharespace_send_task: ");
    // for (int i = tpos; i < tmax; i++)
    //     lprint_hex(data[i]);
    if (ret <= 0)
        return;
    tpos += ret;
    if (tpos < tmax)
        sharespace_notify_send();
    writeb(&transmit_pos, tpos);
}
DECL_TASK(sharespace_send_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
    // Verify space for message
    uint_fast8_t tpos = readb(&transmit_pos), tmax = readb(&transmit_max);
    if (tpos >= tmax) {
        tpos = tmax = 0;
        writeb(&transmit_max, 0);
        writeb(&transmit_pos, 0);
    }
    uint_fast8_t max_size = READP(ce->max_size);
    if (tmax + max_size > sizeof(transmit_buf)) {
        if (tmax + max_size - tpos > sizeof(transmit_buf))
            // Not enough space for message
            return;
        // Disable TX irq and move buffer
        writeb(&transmit_max, 0);
        tpos = readb(&transmit_pos);
        tmax -= tpos;
        memmove(&transmit_buf[0], &transmit_buf[tpos], tmax);
        writeb(&transmit_pos, 0);
        writeb(&transmit_max, tmax);
        // msgbox_enable_tx_irq();
    }

    // Generate message
    uint8_t *buf = &transmit_buf[tmax];
    uint_fast8_t msglen = command_encode_and_frame(buf, ce, args);

    // Start message transmit
    writeb(&transmit_max, tmax + msglen);
    sharespace_notify_send();
    // msgbox_enable_tx_irq();
}
