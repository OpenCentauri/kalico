// RPMsg transport integration for HiFi4 DSP.
//
// This connects the MSGBOX hardware (kick mechanism) to the RPMsg
// virtio layer. It replaces the old sharespace protocol.
//
// Copyright (C) 2026  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <hal.h>
#include "rpmsg.h"
#include "com.h"
#include "log.h"
#include "sched.h" // DECL_INIT

// Forward declaration
void com_set_endpoint(struct rpmsg_endpoint *ept);

// ---------------------------------------------------------------------------
// Configuration — must match device tree and resource table
// ---------------------------------------------------------------------------

// MSGBOX channel used for RPMsg kicks.
// This must match the mboxes property in the device tree.
// The kernel driver uses channel 0 (RX) and channel 4 (TX).
// From the DSP's perspective:
//   - We receive kicks from Linux on a MSGBOX RX channel
//   - We send kicks to Linux on a MSGBOX TX channel
#define MSGBOX_RPMSG_RX_CHANNEL  MSGBOX_CHANNEL_0
#define MSGBOX_RPMSG_TX_CHANNEL  MSGBOX_CHANNEL_0

// ---------------------------------------------------------------------------
// Kick implementation (called by rpmsg.c)
// ---------------------------------------------------------------------------

/*
 * rpmsg_kick - notify Linux that we've updated a vring.
 *
 * This is called by rpmsg_send() and rpmsg_process() after modifying
 * the used ring. The vq_id tells Linux which virtqueue to check:
 *   0 = RX vring (we returned consumed buffers)
 *   1 = TX vring (we added new messages)
 */
void rpmsg_kick(uint32_t vq_id)
{
    msgbox_send(MSGBOX_RPMSG_TX_CHANNEL, vq_id);
}

// ---------------------------------------------------------------------------
// MSGBOX interrupt handler
// ---------------------------------------------------------------------------

/*
 * Called when Linux kicks us via MSGBOX.
 *
 * The message value is the virtqueue ID that Linux wants us to process.
 * We call rpmsg_process() to handle any incoming messages.
 */
static void
rpmsg_msgbox_rx_callback(uint8_t channel, uint32_t message, void *arg)
{
    (void)channel;
    (void)arg;
    (void)message;

    rpmsg_process();
}

// ---------------------------------------------------------------------------
// Example endpoint callback
// ---------------------------------------------------------------------------

// An endpoint for your application (e.g., Klipper communication)
static struct rpmsg_endpoint *klipper_ept;

/*
 * Called when Linux sends a message to our "rpmsg-klipper" endpoint.
 * Passes the data into Klipper's command receive pipeline.
 */
static void
klipper_rpmsg_cb(uint32_t src, const void *data, uint32_t len)
{
    rpmsg_notify_rx(data, len);
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void
rpmsg_transport_init(void)
{
    lprintf("rpmsg: initializing transport\n");

    // Initialize MSGBOX for kick notifications
    msgbox_init();

    // Initialize the RPMsg virtio layer
    rpmsg_init(VRING_TX_ADDR, VRING_RX_ADDR, RPMSG_NUM_BUFS);

    // Check if Linux already has the vrings populated.
    // On cold boot, tx_vring.avail->idx == 0 (no buffers yet).
    // After a soft reset, it's > 0 because Linux never tore down.
    int cold_boot = rpmsg_tx_avail_idx() == 0;

    if (cold_boot) {
        // Wait for Linux to finish virtio setup.
        // Linux populates the TX vring (vring0) with empty buffers when
        // virtio_rpmsg_bus probes. Until that happens, avail->idx == 0.
        hal_debug_print("rpmsg: waiting for Linux...\n");
        msgbox_recv_blocking(MSGBOX_RPMSG_RX_CHANNEL);
        hal_debug_print("rpmsg: Linux is ready\n");
    } else {
        hal_debug_print("rpmsg: warm restart, syncing with Linux\n");
        rpmsg_sync_indices();
    }

    // Create the endpoint BEFORE enabling IRQs
    klipper_ept = rpmsg_create_ept("rpmsg-tty", 1024, klipper_rpmsg_cb);
    if (!klipper_ept) {
        hal_debug_print("rpmsg: failed to create endpoint\n");
        return;
    }

    // Give com.c access to the endpoint for TX
    com_set_endpoint(klipper_ept);

    if (cold_boot) {
        // Send the initial NS announcement and process any pending messages.
        // Do this BEFORE enabling the MSGBOX IRQ so there's no race.
        rpmsg_process();
    } else {
        // Warm restart — set dst directly, Linux already knows us
        klipper_ept->dst = 0x400;  // The address Linux assigned
        // Suppress NS announcement
        rpmsg_clear_pending_announcements();
    }

    // NOW enable the interrupt handler — pending_announcements is already 0
    msgbox_set_rx_callback(MSGBOX_RPMSG_RX_CHANNEL,
                           rpmsg_msgbox_rx_callback, NULL);
    msgbox_enable_rx_irq(MSGBOX_RPMSG_RX_CHANNEL);

    hal_debug_print("rpmsg: transport ready\n");
}
DECL_INIT(rpmsg_transport_init);
