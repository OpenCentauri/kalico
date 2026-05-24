// Minimal RPMsg/virtio implementation for bare-metal HiFi4 DSP.
//
// This implements just enough of the virtio vring protocol and RPMsg
// framing to communicate with Linux's virtio_rpmsg_bus driver.
//
// Vring direction naming (from Linux host perspective):
//   vring0 = "svq" (send virtqueue) — Linux puts TX buffers, DSP consumes
//            DSP's RX path
//   vring1 = "rvq" (receive virtqueue) — DSP puts data, Linux consumes
//            DSP's TX path
//
// Buffer lifecycle:
//
//   RX (Linux → DSP):
//     1. Linux pre-allocates empty buffers and puts them on vring0.avail
//     2. Linux kicks DSP via MSGBOX
//     3. DSP takes a buffer from vring0.avail, reads the rpmsg message
//     4. DSP returns the buffer to vring0.used (so Linux can reuse it)
//     5. DSP kicks Linux via MSGBOX
//
//   TX (DSP → Linux):
//     1. Linux pre-allocates empty buffers and puts them on vring1.avail
//     2. DSP takes an empty buffer from vring1.avail
//     3. DSP writes an rpmsg message into the buffer
//     4. DSP puts the filled buffer on vring1.used
//     5. DSP kicks Linux via MSGBOX
//     6. Linux processes the message and recycles the buffer to vring1.avail
//
// Copyright (C) 2026  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "rpmsg.h"
#include "virtio_vring.h"
#include "log.h"
#include "util.h" // memcpy

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

// RX vring: we consume buffers that Linux put on avail
static struct vring rx_vring;
static uint16_t rx_last_avail_idx;

// TX vring: we take empty buffers from avail, fill them, put on used
static struct vring tx_vring;
static uint16_t tx_last_avail_idx;

// Registered endpoints
static struct rpmsg_endpoint endpoints[RPMSG_MAX_ENDPOINTS];

// Whether transport is initialized
static int rpmsg_ready;

// Whether we have endpoints waiting to be announced to Linux.
// The NS announcement is deferred because at boot, the DSP runs
// before Linux has populated the TX vring with available buffers.
static int pending_announcements;

// ---------------------------------------------------------------------------
// Memory barrier helpers
//
// On Xtensa/HiFi4, we need barriers to ensure the other processor sees
// our writes in the correct order. These are conservative — you may be
// able to relax them for your specific Xtensa config.
// ---------------------------------------------------------------------------

static inline void mb(void)
{
    __asm__ __volatile__("memw" ::: "memory");
}

// ---------------------------------------------------------------------------
// Vring helpers
// ---------------------------------------------------------------------------

/*
 * Get the next available descriptor index from a vring.
 * Returns -1 if no descriptors are available.
 */
static int vring_get_avail(struct vring *vr, uint16_t *last_avail_idx)
{
    mb();

    if (*last_avail_idx == vr->avail->idx)
        return -1;  // No new buffers

    uint16_t desc_idx = vr->avail->ring[*last_avail_idx % vr->num];
    (*last_avail_idx)++;

    return desc_idx;
}

/*
 * Put a descriptor index into the used ring.
 * This returns the buffer to the other side.
 */
static void vring_put_used(struct vring *vr, uint16_t desc_idx, uint32_t len)
{
    uint16_t used_idx = vr->used->idx % vr->num;

    vr->used->ring[used_idx].id = desc_idx;
    vr->used->ring[used_idx].len = len;

    mb();

    vr->used->idx++;

    mb();
}

// ---------------------------------------------------------------------------
// Endpoint management
// ---------------------------------------------------------------------------

static struct rpmsg_endpoint *find_endpoint(uint32_t addr)
{
    for (int i = 0; i < RPMSG_MAX_ENDPOINTS; i++) {
        if (endpoints[i].active && endpoints[i].addr == addr)
            return &endpoints[i];
    }
    return NULL;
}

// ---------------------------------------------------------------------------
// Name service announcement
// ---------------------------------------------------------------------------

/*
 * Send a name service announcement to Linux.
 * This tells Linux that we have an endpoint with the given name,
 * causing it to create a /dev/rpmsgN device (if using rpmsg_char)
 * or bind an rpmsg driver.
 */
static int rpmsg_ns_announce(struct rpmsg_endpoint *ept, uint32_t flags)
{
    struct rpmsg_ns_msg ns_msg;

    memset(&ns_msg, 0, sizeof(ns_msg));
    memcpy(ns_msg.name, ept->name, RPMSG_NAME_SIZE);
    ns_msg.addr = ept->addr;
    ns_msg.flags = flags;

    return rpmsg_send(ept, RPMSG_NS_ADDR, &ns_msg, sizeof(ns_msg));
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void rpmsg_init(uint32_t tx_vring_addr, uint32_t rx_vring_addr, uint32_t num_bufs)
{
    // TX vring (vring0): DSP takes empty buffers, fills with messages
    vring_init(&tx_vring, num_bufs, (void *)tx_vring_addr, RPMSG_VRING_ALIGN);
    tx_last_avail_idx = 0;
    
    // RX vring (vring1): DSP reads messages sent by Linux
    vring_init(&rx_vring, num_bufs, (void *)rx_vring_addr, RPMSG_VRING_ALIGN);
    rx_last_avail_idx = 0;
    
    // Clear endpoints
    memset(endpoints, 0, sizeof(endpoints));

    rpmsg_ready = 1;

    lprintf("rpmsg: initialized, tx_vring@0x%lx rx_vring@0x%lx num=%lu\n",
            (unsigned long)tx_vring_addr, (unsigned long)rx_vring_addr,
            (unsigned long)num_bufs);
}

struct rpmsg_endpoint *rpmsg_create_ept(const char *name, uint32_t addr,
                                        rpmsg_rx_cb_t cb)
{
    // Find a free slot
    struct rpmsg_endpoint *ept = NULL;
    for (int i = 0; i < RPMSG_MAX_ENDPOINTS; i++) {
        if (!endpoints[i].active) {
            ept = &endpoints[i];
            break;
        }
    }

    if (!ept) {
        lprintf("rpmsg: no free endpoint slots\n");
        return NULL;
    }

    memset(ept, 0, sizeof(*ept));
    ept->addr = addr;
    ept->dst = RPMSG_ADDR_ANY;
    ept->cb = cb;
    ept->active = 1;
    memcpy(ept->name, name, RPMSG_NAME_SIZE);

    lprintf("rpmsg: created endpoint \"%s\" addr=%lu\n",
            name, (unsigned long)addr);

    // Don't announce immediately — Linux hasn't populated the TX vring yet.
    // The announcement will be sent from rpmsg_process() when Linux first
    // kicks us, which means the TX buffers are ready.
    pending_announcements = 1;

    return ept;
}

int rpmsg_send(struct rpmsg_endpoint *ept, uint32_t dst,
               const void *data, uint32_t len)
{

    if (!rpmsg_ready)
        return -1;

    if (len > RPMSG_DATA_SIZE)
        return -2;

    // Get an empty buffer from the TX vring's available ring.
    // Linux pre-populates these when virtio_rpmsg_bus starts.
    int desc_idx = vring_get_avail(&tx_vring, &tx_last_avail_idx);
    if (desc_idx < 0) {
        lprintf("rpmsg: TX no available buffers\n");
        return -3;
    }

    struct vring_desc *desc = &tx_vring.desc[desc_idx];

    // The descriptor points to a buffer in shared memory.
    // Write our RPMsg message into it.
    struct rpmsg_hdr *hdr = (struct rpmsg_hdr *)(uint32_t)desc->addr;

    hdr->src = ept->addr;
    hdr->dst = dst;
    hdr->reserved = 0;
    hdr->len = len;
    hdr->flags = 0;

    memcpy((void *)hdr + sizeof(*hdr), data, len);

    mb();

    // Return the filled buffer via the used ring.
    // The total used length is header + payload.
    vring_put_used(&tx_vring, desc_idx, sizeof(*hdr) + len);

    // Kick Linux (vring0 = TX = notify_id 0)
    rpmsg_kick(0);

    return 0;
}

int rpmsg_sendto(struct rpmsg_endpoint *ept, const void *data, uint32_t len)
{
    if (ept->dst == RPMSG_ADDR_ANY) {
        lprintf("rpmsg: endpoint \"%s\" has no remote dst yet\n", ept->name);
        return -4;
    }
    return rpmsg_send(ept, ept->dst, data, len);
}

int rpmsg_process(void)
{
    if (!rpmsg_ready)
        return 0;

    int processed = 0;

    /*
     * Send any deferred name service announcements.
     *
     * At boot, the DSP starts before Linux has populated the TX vring
     * with available buffers. We defer the NS announcement until
     * rpmsg_process() is called (triggered by a MSGBOX kick from Linux),
     * which means the virtio device is ready and TX buffers are available.
     */
    if (pending_announcements) {
        for (int i = 0; i < RPMSG_MAX_ENDPOINTS; i++) {
            if (endpoints[i].active) {
                if (rpmsg_ns_announce(&endpoints[i], RPMSG_NS_CREATE) == 0) {
                    lprintf("rpmsg: announced \"%s\" to Linux\n",
                            endpoints[i].name);
                } else {
                    lprintf("rpmsg: failed to announce \"%s\"\n",
                            endpoints[i].name);
                }
            }
        }
        pending_announcements = 0;
    }

    // Process all available messages on the RX vring
    while (1) {
        int desc_idx = vring_get_avail(&rx_vring, &rx_last_avail_idx);
        if (desc_idx < 0)
            break;

        struct vring_desc *desc = &rx_vring.desc[desc_idx];
        struct rpmsg_hdr *hdr = (struct rpmsg_hdr *)(uint32_t)desc->addr;

        // Find the endpoint this message is addressed to
        struct rpmsg_endpoint *ept = find_endpoint(hdr->dst);
        if (ept) {
            // Update the remote destination address so rpmsg_sendto works
            if (ept->dst == RPMSG_ADDR_ANY)
                ept->dst = hdr->src;

            // Dispatch to the endpoint's callback
            if (ept->cb) {
                ept->cb(hdr->src, (void *)hdr + sizeof(*hdr), hdr->len);
            }
        } else {
            lprintf("rpmsg: no endpoint for dst=%lu (from src=%lu)\n",
                    (unsigned long)hdr->dst, (unsigned long)hdr->src);
        }

        // Return the buffer to Linux via the used ring
        vring_put_used(&rx_vring, desc_idx, RPMSG_BUF_SIZE);

        processed++;
    }

    // Kick Linux so it knows we returned buffers (vring1 = RX = notify_id 1)
    if (processed > 0)
        rpmsg_kick(1);

    return processed;
}

uint16_t rpmsg_tx_avail_idx(void)
{
    mb();
    return tx_vring.avail->idx;
}

void rpmsg_sync_indices(void)
{
    // Match Linux's current position — consume nothing,
    // just sync to where Linux is now
    mb();
    rx_last_avail_idx = rx_vring.avail->idx;
    tx_last_avail_idx = tx_vring.used->idx;
}

void rpmsg_clear_pending_announcements(void)
{
    pending_announcements = 0;
}
