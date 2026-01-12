// Support for standard serial port over USB emulation
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memmove
#include "autoconf.h" // CONFIG_USB_VENDOR_ID
#include "board/misc.h" // console_sendf
#include "board/pgm.h" // PROGMEM
#include "board/usb_cdc_ep.h" // USB_CDC_EP_BULK_IN
#include "byteorder.h" // cpu_to_le16
#include "command.h" // output
#include "generic/usbstd.h" // struct usb_device_descriptor
#include "generic/usbstd_cdc.h" // struct usb_cdc_header_descriptor
#include "sched.h" // sched_wake_task
#include "generic/usb_cdc.h" // usb_notify_ep0
#include "usb_bulk.h" // share_space_find_data_pack

// To debug a USB connection over UART, uncomment the two macros
// below, alter the board KConfig to "select USBSERIAL" on a serial
// UART build (so both USB and UART are enabled in a single build),
// compile the code using serial UART, add output() calls to the USB
// code as needed, deploy the new binary, and then connect via
// console.py using UART to see those output() messages.
//#define console_sendf(ce,va) console_sendf_usb(ce,va)
//#define command_find_and_dispatch(rb, rp, pc) ({*(pc) = rp; 1;})


/****************************************************************
 * Message block sending
 ****************************************************************/

static struct task_wake usb_bulk_in_wake;
static uint8_t transmit_buf[96], transmit_pos;

void
usb_notify_bulk_in(void)
{
    sched_wake_task(&usb_bulk_in_wake);
}

void
usb_bulk_in_task(void)
{
    if (!sched_check_wake(&usb_bulk_in_wake))
        return;
    uint_fast8_t tpos = transmit_pos;
    if (!tpos)
        return;
    uint_fast8_t max_tpos = (tpos > USB_CDC_EP_BULK_IN_SIZE ? USB_CDC_EP_BULK_IN_SIZE : tpos);
    int_fast8_t ret = usb_send_bulk_in(transmit_buf, max_tpos);  //-----ggg---6.9
    if (ret <= 0)
        return;
    uint_fast8_t needcopy = tpos - ret;
    if (needcopy) {
        memmove(transmit_buf, &transmit_buf[ret], needcopy);
        usb_notify_bulk_in();
    }
    transmit_pos = needcopy;
}
DECL_TASK(usb_bulk_in_task);

// Encode and transmit a "response" message
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
    usb_notify_bulk_in();
}


/****************************************************************
 * Message block reading
 ****************************************************************/

static struct task_wake usb_bulk_out_wake;
static uint8_t receive_buf[128], receive_pos;

void
usb_notify_bulk_out(void)
{
    sched_wake_task(&usb_bulk_out_wake);
}

void
usb_bulk_out_task(void)
{
    if (!sched_check_wake(&usb_bulk_out_wake))
        return;

    share_space_find_data_pack(); //-----read share space ,find data block-----

    // Read data
    uint_fast8_t rpos = receive_pos, pop_count;
    if (rpos + USB_CDC_EP_BULK_OUT_SIZE <= sizeof(receive_buf)) {
        int_fast8_t ret = usb_read_bulk_out(
            &receive_buf[rpos], USB_CDC_EP_BULK_OUT_SIZE);
        if (ret > 0) {
            rpos += ret;
            usb_notify_bulk_out();
        }
    } else {
        usb_notify_bulk_out();
    }
    // Process a message block
    int_fast8_t ret = command_find_and_dispatch(receive_buf, rpos, &pop_count);
    if (ret) {
        // Move buffer
        uint_fast8_t needcopy = rpos - pop_count;
        if (needcopy) {
            memmove(receive_buf, &receive_buf[pop_count], needcopy);
            usb_notify_bulk_out();
        }
        rpos = needcopy;
    }
    receive_pos = rpos;
}
DECL_TASK(usb_bulk_out_task);

/****************************************************************
 * USB endpoint 0 control message handling
 ****************************************************************/

// State tracking
enum {
    UX_READ = 1<<0, UX_SEND = 1<<1, UX_SEND_PROGMEM = 1<<2, UX_SEND_ZLP = 1<<3
};

static void *usb_xfer_data;
static uint8_t usb_xfer_size, usb_xfer_flags;

// Transfer data on the usb endpoint 0
static void
usb_do_xfer(void *data, uint_fast8_t size, uint_fast8_t flags)
{
}

static void
usb_state_ready(void)
{
}

// State tracking dispatch
static struct task_wake usb_ep0_wake;

void
usb_notify_ep0(void)
{
    sched_wake_task(&usb_ep0_wake);
}

void
usb_ep0_task(void)
{
    if (!sched_check_wake(&usb_ep0_wake))
        return;
    if (usb_xfer_flags)
        usb_do_xfer(usb_xfer_data, usb_xfer_size, usb_xfer_flags);
    else
        usb_state_ready();
}
DECL_TASK(usb_ep0_task);

void
usb_shutdown(void)
{
    usb_notify_bulk_in();
    usb_notify_bulk_out();
    usb_notify_ep0();
}
DECL_SHUTDOWN(usb_shutdown);
