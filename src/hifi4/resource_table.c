#include <stdint.h>
#include <stddef.h>
#include "log.h"
#include "virtio_vring.h"
#include "rpmsg.h"

/*
 * These definitions match what Linux expects.
 * Normally from <linux/remoteproc.h> but we define them standalone
 * for the DSP side.
 */
#define RSC_CARVEOUT    0
#define RSC_TRACE       2
#define RSC_VDEV        3

struct fw_rsc_vdev_vring {
    uint32_t da;        /* device address of vring (0 = auto) */
    uint32_t align;
    uint32_t num;       /* number of buffers */
    uint32_t notifyid;
    uint32_t reserved;
};

struct fw_rsc_vdev {
    uint32_t type;      /* RSC_VDEV */
    uint32_t id;        /* VIRTIO_ID_RPMSG */
    uint32_t notifyid;
    uint32_t dfeatures;
    uint32_t gfeatures;
    uint32_t config_len;
    uint8_t  status;
    uint8_t  num_of_vrings;
    uint8_t  reserved[2];
    struct fw_rsc_vdev_vring vring[2];
};

struct fw_rsc_trace {
    uint32_t type;          /* RSC_TRACE */
    uint32_t da;            /* device address of trace_buffer */
    uint32_t len;
    uint32_t reserved;
    char     name[32];
};

struct resource_table {
    uint32_t ver;
    uint32_t num;
    uint32_t reserved[2];
    uint32_t offset[2];

    /* Entry 0: RPMsg vdev */
    struct fw_rsc_vdev rpmsg_vdev;

    /* Entry 1: trace buffer */
    struct fw_rsc_trace trace;
};

extern char trace_buffer[];

/*
 * Place in a dedicated section so the ELF loader can find it.
 */
const struct resource_table __attribute__((section(".resource_table")))
resource_table = {
    .ver = 1,
    .num = 2,
    .offset = {
        offsetof(struct resource_table, rpmsg_vdev),
        offsetof(struct resource_table, trace),
    },
    .rpmsg_vdev = {
        .type        = RSC_VDEV,
        .id          = VIRTIO_ID_RPMSG,
        .notifyid    = 0,
        .dfeatures   = 1,   // VIRTIO_RPMSG_F_NS
        .gfeatures   = 0,
        .config_len  = 0,
        .status      = 0,
        .num_of_vrings = 2,
        .vring = {
            [0] = { .da = VRING_TX_ADDR, .align = RPMSG_VRING_ALIGN,
                    .num = RPMSG_NUM_BUFS, .notifyid = 0 },
            [1] = { .da = VRING_RX_ADDR, .align = RPMSG_VRING_ALIGN,
                    .num = RPMSG_NUM_BUFS, .notifyid = 1 },
        },
    },
    .trace = {
        .type = RSC_TRACE,
        .da   = (uint32_t)&trace_buffer,
        .len  = TRACE_BUF_SIZE,
        .name = "dsp_trace",
    },
};
