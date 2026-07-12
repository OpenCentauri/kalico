// Minimal RPMsg implementation for bare-metal HiFi4 DSP.
//
// Copyright (C) 2026  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __HIFI4_RPMSG_H
#define __HIFI4_RPMSG_H

#include <stdint.h>

// RPMsg protocol constants
#define RPMSG_NAME_SIZE         32
#define RPMSG_BUF_SIZE          512     // Total buffer size including header
#define RPMSG_DATA_SIZE         (RPMSG_BUF_SIZE - sizeof(struct rpmsg_hdr))

// Well-known RPMsg addresses
#define RPMSG_ADDR_ANY          0xFFFFFFFF
#define RPMSG_NS_ADDR           53      // Name service endpoint address

// Name service announcement flags
#define RPMSG_NS_CREATE         0
#define RPMSG_NS_DESTROY        1

// Shared-memory placement — must match resource table and device tree.
//
// All RPMsg shared memory (vrings + message buffers) lives in SRAM A1
// (32 KiB @ 0x00020000). SRAM A1 is visible to ARM and DSP at the same
// address and is uncached on both sides (DSP region 0 keeps the
// reset-default bypass attribute), so there are no coherency concerns.
//
// SRAM A1 layout (totals exactly 32 KiB):
//   vring0  @ 0x20000, 4 KiB   (Linux PAGE_ALIGNs vring_size(16, 256))
//   vring1  @ 0x21000, 4 KiB
//   buffers @ 0x22000, 20 KiB  (Linux allocates num*2*512 = 16 KiB)
//   trace   @ 0x27000, 4 KiB   (.trace_buf, placed by hifi4.ld)
//
// The Linux DTS reserved-memory nodes (vdev0vring0/1, vdev0buffer,
// dsp_trace) must match these addresses/sizes.

// Vring addresses
//
// IMPORTANT: From Linux's perspective (virtio_rpmsg_bus.c):
//   vring0 = rvq = Linux's receive queue, pre-filled with empty buffers
//            DSP's TX: take empty buffer, write message, return via used ring
//   vring1 = svq = Linux's send queue, Linux puts messages here
//            DSP's RX: read messages from available ring
//
#define RPMSG_VRING_ALIGN       256
#define RPMSG_NUM_BUFS          16      // Number of buffers per direction
#define VRING_TX_ADDR   0x00020000  // vring0: DSP → Linux
#define VRING_RX_ADDR   0x00021000  // vring1: Linux → DSP

/*
 * RPMsg message header — prepended to every message.
 *
 * This must match Linux's struct rpmsg_hdr exactly.
 */
struct rpmsg_hdr {
    uint32_t src;       // Source endpoint address
    uint32_t dst;       // Destination endpoint address
    uint32_t reserved;  // Reserved for future use
    uint16_t len;       // Length of payload (after this header)
    uint16_t flags;     // Message flags
} __attribute__((packed));

/*
 * RPMsg name service announcement message.
 *
 * Sent to RPMSG_NS_ADDR (53) to tell Linux about a new endpoint.
 * Linux creates /dev/rpmsgN (or binds an rpmsg driver) in response.
 */
struct rpmsg_ns_msg {
    char name[RPMSG_NAME_SIZE]; // Endpoint name (e.g., "rpmsg-klipper")
    uint32_t addr;              // Endpoint address on the DSP
    uint32_t flags;             // RPMSG_NS_CREATE or RPMSG_NS_DESTROY
} __attribute__((packed));

/*
 * Endpoint receive callback.
 *
 * Called when a message arrives for a registered endpoint.
 *   src:  source endpoint address (Linux side)
 *   data: pointer to payload (after rpmsg_hdr)
 *   len:  payload length
 */
typedef void (*rpmsg_rx_cb_t)(uint32_t src, const void *data, uint32_t len);

/*
 * RPMsg endpoint.
 */
struct rpmsg_endpoint {
    uint32_t addr;              // Local endpoint address
    uint32_t dst;               // Remote endpoint address (set after binding)
    char name[RPMSG_NAME_SIZE]; // Endpoint name
    rpmsg_rx_cb_t cb;           // Receive callback
    int active;                 // Whether this endpoint is registered
};

// Maximum number of endpoints the DSP can register
#define RPMSG_MAX_ENDPOINTS     4

/*
 * Initialize the RPMsg transport.
 *
 * Call this once at startup. It sets up the vrings and waits for
 * Linux to populate the available buffers.
 *
 *   tx_vring_addr: physical address of vring0 (our RX — Linux sends here)
 *   rx_vring_addr: physical address of vring1 (our TX — we send here)
 *   num_bufs:      number of buffers per vring (must match resource table)
 */
void rpmsg_init(uint32_t tx_vring_addr, uint32_t rx_vring_addr, uint32_t num_bufs);

/*
 * Create and announce an endpoint.
 *
 * Registers a local endpoint and sends a name service announcement
 * to Linux. Linux will create a corresponding rpmsg device.
 *
 *   name: endpoint name (e.g., "rpmsg-klipper")
 *   addr: local endpoint address (pick any unique number, e.g., 1024)
 *   cb:   callback invoked when messages arrive for this endpoint
 *
 * Returns: pointer to endpoint on success, NULL on failure
 */
struct rpmsg_endpoint *rpmsg_create_ept(const char *name, uint32_t addr,
                                        rpmsg_rx_cb_t cb);

/*
 * Send a message to a remote endpoint.
 *
 *   ept:  local endpoint (source)
 *   dst:  destination endpoint address on Linux side
 *   data: payload data
 *   len:  payload length (must be <= RPMSG_DATA_SIZE)
 *
 * Returns: 0 on success, negative on error
 */
int rpmsg_send(struct rpmsg_endpoint *ept, uint32_t dst,
               const void *data, uint32_t len);

/*
 * Send a message using the endpoint's stored destination.
 *
 * Convenience wrapper — uses ept->dst as the destination, which gets
 * set automatically when Linux first sends a message to this endpoint.
 */
int rpmsg_sendto(struct rpmsg_endpoint *ept, const void *data, uint32_t len);

/*
 * Process incoming messages.
 *
 * Call this from your main loop or MSGBOX interrupt handler.
 * It checks the RX vring for new messages and dispatches them
 * to the appropriate endpoint callbacks.
 *
 * Returns: number of messages processed
 */
int rpmsg_process(void);

/*
 * Kick Linux to notify that we've added buffers to a vring.
 *
 * This is called internally by rpmsg_send/rpmsg_process, but
 * you can provide the implementation since you already have
 * MSGBOX code. The vq_id is 0 for RX vring, 1 for TX vring.
 */
extern void rpmsg_kick(uint32_t vq_id);

/*
 * Get the current TX vring available index.
 *
 * Returns the avail->idx value from the TX vring, which indicates
 * how many empty buffers Linux has posted. Useful for detecting
 * whether the virtio transport has been initialized by the host
 * (returns 0 on a fresh cold boot before Linux populates buffers).
 *
 * Returns: current TX vring avail->idx value
 */
uint16_t rpmsg_tx_avail_idx(void);

/*
 * Resynchronize vring tracking indices after a warm restart.
 *
 * After a soft reset, the shared vring memory in SRAM A1 is still valid
 * but the DSP's local tracking indices were lost. This function
 * restores them from the vring state so the DSP stays in sync
 * with Linux:
 *   - RX: advances to avail->idx (skips any stale messages)
 *   - TX: resets to used->idx (reclaims recycled empty buffers)
 *
 * Call this once during warm restart init, before enabling the
 * MSGBOX interrupt handler.
 */
void rpmsg_sync_indices(void);

/*
 * Suppress any pending name service announcements.
 *
 * During a warm restart the rpmsg channel is already established
 * on the Linux side, so re-announcing endpoints is unnecessary
 * and would cause "channel already exists" errors. Call this
 * after recreating endpoints on the warm restart path.
 */
void rpmsg_clear_pending_announcements(void);

#endif // __HIFI4_RPMSG_H
