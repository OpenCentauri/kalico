// Virtio vring data structures for bare-metal DSP firmware.
//
// These structures must match the Linux kernel's virtio_ring.h exactly.
// The vring is a lock-free producer/consumer ring buffer used by virtio
// to pass buffer descriptors between two processors.
//
// Copyright (C) 2026  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __HIFI4_VIRTIO_VRING_H
#define __HIFI4_VIRTIO_VRING_H

#include <stdint.h>

// Feature bits
#define VIRTIO_ID_RPMSG         7

// Vring descriptor flags
#define VRING_DESC_F_NEXT       1   // Buffer continues via 'next' field
#define VRING_DESC_F_WRITE      2   // Buffer is write-only (from device perspective)

// Used ring flag: don't kick the other side when adding to used ring
#define VRING_USED_F_NO_NOTIFY  1

// Available ring flag: don't kick us when adding to avail ring
#define VRING_AVAIL_F_NO_INTERRUPT  1

/*
 * Vring descriptor - describes a single buffer.
 *
 * The descriptor table is a flat array of these. Each entry points to
 * a buffer in shared memory. Descriptors can be chained via 'next'.
 */
struct vring_desc {
    uint64_t addr;      // Physical/bus address of the buffer
    uint32_t len;       // Length of the buffer in bytes
    uint16_t flags;     // VRING_DESC_F_* flags
    uint16_t next;      // Next descriptor index if VRING_DESC_F_NEXT set
} __attribute__((packed));

/*
 * Available ring - producer (sender) adds buffer indices here.
 *
 * The available ring tells the consumer which descriptors have buffers
 * ready to be processed. The producer adds descriptor indices to
 * ring[idx % num] and increments idx.
 */
struct vring_avail {
    uint16_t flags;     // VRING_AVAIL_F_* flags
    uint16_t idx;       // Next index the producer will write to
    uint16_t ring[];    // Array of descriptor indices (length = num)
} __attribute__((packed));

/*
 * Used ring element - one completed buffer.
 */
struct vring_used_elem {
    uint32_t id;        // Index of the descriptor chain head
    uint32_t len;       // Number of bytes written by the consumer
} __attribute__((packed));

/*
 * Used ring - consumer (receiver) adds completed buffer indices here.
 *
 * After the consumer processes a buffer from the available ring, it
 * puts the descriptor index into the used ring so the producer can
 * reclaim it.
 */
struct vring_used {
    uint16_t flags;     // VRING_USED_F_* flags
    uint16_t idx;       // Next index the consumer will write to
    struct vring_used_elem ring[];
} __attribute__((packed));

/*
 * Complete vring structure - ties together desc, avail, and used.
 */
struct vring {
    uint32_t num;                   // Number of descriptors
    struct vring_desc *desc;        // Descriptor table
    struct vring_avail *avail;      // Available ring
    struct vring_used *used;        // Used ring
};

/*
 * Calculate the byte size of the available ring.
 */
static inline uint32_t vring_avail_size(uint32_t num)
{
    return sizeof(struct vring_avail) + sizeof(uint16_t) * num
           + sizeof(uint16_t); // used_event
}

/*
 * Calculate the offset from the start of the vring to the used ring.
 * The used ring is aligned to the specified alignment boundary.
 */
static inline uint32_t vring_used_offset(uint32_t num, uint32_t align)
{
    uint32_t offset = sizeof(struct vring_desc) * num + vring_avail_size(num);
    return (offset + align - 1) & ~(align - 1);
}

/*
 * Initialize a vring structure from a base address.
 *
 * Layout in memory:
 *   [descriptor table][available ring][padding][used ring]
 */
static inline void vring_init(struct vring *vr, uint32_t num,
                              void *base, uint32_t align)
{
    vr->num = num;
    vr->desc = (struct vring_desc *)base;
    vr->avail = (struct vring_avail *)((uint8_t *)base +
                 num * sizeof(struct vring_desc));
    vr->used = (struct vring_used *)((uint8_t *)base +
                vring_used_offset(num, align));
}

#endif // __HIFI4_VIRTIO_VRING_H
