#include "sharespace.h"
#include "basecmd.h"
#include <platform.h>
#include <hal.h>
#include "msgbox.h"
#include "util.h"
#include "sched.h" // DECL_INIT
#include "log.h"

// Pointers to the shared memory regions.
static volatile uint8_t* dsp_reads_from_arm = NULL; // ARM writes here, DSP reads from here.
static volatile uint8_t* dsp_writes_to_arm = NULL;  // DSP writes here, ARM reads from here.

// Head pointers for the two buffers.
static volatile MsgHead* arm_head_ptr = NULL;   // Head for the ARM->DSP buffer.
static volatile MsgHead* dsp_head_ptr = NULL;   // Head for the DSP->ARM buffer.

// Global instance to store the discovered shared space parameters
static struct dts_sharespace_t dts_sharespace;

// platform_head is defined as a macro in platform.h

// This is a placeholder for the hardware-specific function that will trigger
// an RPMsg interrupt to notify the host that new data is available.
extern void rpmsg_signal_host(uint32_t msg);

// Gets the initial shared memory configuration from the platform header.
static void sharespace_get_config(struct dts_sharespace_t *p_dts_sharespace) {
    volatile struct spare_rtos_head_t *pstr = platform_head;
    volatile struct dts_msg_t *pdts = &pstr->rtos_img_hdr.dts_msg;
    dcache_region_invalidate((void*)pstr, sizeof(struct spare_rtos_head_t));
    hal_debug_print("Initial DTS Sharespace Configuration:\n");
    hal_debug_variable("  Status:             ", pdts->dts_sharespace.status);
    hal_debug_variable("  DSP Write Address:  ", pdts->dts_sharespace.dsp_write_addr);
    hal_debug_variable("  DSP Write Size:     ", pdts->dts_sharespace.dsp_write_size);
    hal_debug_variable("  ARM Write Address:  ", pdts->dts_sharespace.arm_write_addr);
    hal_debug_variable("  ARM Write Size:     ", pdts->dts_sharespace.arm_write_size);
    hal_debug_variable("  DSP Log Address:    ", pdts->dts_sharespace.dsp_log_addr);
    hal_debug_variable("  DSP Log Size:       ", pdts->dts_sharespace.dsp_log_size);
    while (1) {
        dcache_region_invalidate((void*)pstr, sizeof(struct spare_rtos_head_t));
        if (pdts->dts_sharespace.status == DTS_OPEN) {
            p_dts_sharespace->dsp_write_addr = pdts->dts_sharespace.dsp_write_addr;
            p_dts_sharespace->dsp_write_size = pdts->dts_sharespace.dsp_write_size;
            p_dts_sharespace->arm_write_addr = pdts->dts_sharespace.arm_write_addr;
            p_dts_sharespace->arm_write_size = pdts->dts_sharespace.arm_write_size;
            p_dts_sharespace->dsp_log_addr = pdts->dts_sharespace.dsp_log_addr;
            p_dts_sharespace->dsp_log_size = pdts->dts_sharespace.dsp_log_size;
            hal_debug_print("Final DTS Sharespace Configuration:\n");
            hal_debug_variable("  Status:             ", pdts->dts_sharespace.status);
            hal_debug_variable("  DSP Write Address:  ", pdts->dts_sharespace.dsp_write_addr);
            hal_debug_variable("  DSP Write Size:     ", pdts->dts_sharespace.dsp_write_size);
            hal_debug_variable("  ARM Write Address:  ", pdts->dts_sharespace.arm_write_addr);
            hal_debug_variable("  ARM Write Size:     ", pdts->dts_sharespace.arm_write_size);
            hal_debug_variable("  DSP Log Address:    ", pdts->dts_sharespace.dsp_log_addr);
            hal_debug_variable("  DSP Log Size:       ", pdts->dts_sharespace.dsp_log_size);
            return;
        }
    }
}

// Fake init function to call from DSP boot-up to check on some stuff
void sharespace_fake_init(void) {
    sharespace_get_config(&dts_sharespace);
    // hal_debug_print("DTS Sharespace Configuration:\n");
    // hal_debug_variable("  DSP Write Address:  ", dts_sharespace.dsp_write_addr);
    // hal_debug_variable("  DSP Write Size:     ", dts_sharespace.dsp_write_size);
    // hal_debug_variable("  ARM Write Address:  ", dts_sharespace.arm_write_addr);
    // hal_debug_variable("  ARM Write Size:     ", dts_sharespace.arm_write_size);
    // hal_debug_variable("  DSP Log Address:    ", dts_sharespace.dsp_log_addr);
    // hal_debug_variable("  DSP Log Size:       ", dts_sharespace.dsp_log_size);
    dsp_reads_from_arm = (volatile uint8_t*)dts_sharespace.arm_write_addr;
    dsp_writes_to_arm = (volatile uint8_t*)dts_sharespace.dsp_write_addr;
    // hal_debug_print("Initialized Shared Memory Pointers:\n");
    // hal_debug_variable("  DSP Reads from ARM: ", (uint32_t)dsp_reads_from_arm);
    // hal_debug_variable("  DSP Writes to ARM:  ", (uint32_t)dsp_writes_to_arm);
    arm_head_ptr = (volatile MsgHead*)(dsp_reads_from_arm + SHARE_SPACE_HEAD_OFFSET);
    dsp_head_ptr = (volatile MsgHead*)(dsp_writes_to_arm + SHARE_SPACE_HEAD_OFFSET);
    // hal_debug_print("Initialized Message Head Pointers:\n");
    // hal_debug_variable("  ARM Head Pointer:   ", (uint32_t)arm_head_ptr);
    // hal_debug_variable("  DSP Head Pointer:   ", (uint32_t)dsp_head_ptr);
    /*char[] msg="This is a test of the emergency buffer initialization system. This is only a test!";
    memcpy((void*)(dsp_writes_to_arm + MIN_ADDR), msg, strlen(msg)+1);
    printf("Initializing DSP head buffer (%p) to string (%d bytes):\n%s\n", (void*)dsp_head_ptr, strlen(msg)+1, msg);*/
    // hal_debug_print("DONE DSP FAKEINIT!\n\n");
    return;
}

// Waits for the ARM core to initialize its side of the shared memory.
static void sharespace_reinit(MsgHead *p_arm_head) {
    // hal_debug_print("sharespace_reinit: Waiting for ARM initialization...\n");
    delay_us(10000); // sleep 10 seconds before looping
    while (1) {
        // hal_debug_variable("sharespace_reinit: Reading ARM head from ", (uint32_t)arm_head_ptr);
        dcache_region_invalidate((void*)arm_head_ptr, sizeof(MsgHead));
        memcpy(p_arm_head, (const void*)arm_head_ptr, sizeof(MsgHead));
        // hal_debug_variable("sharespace_reinit: arm_head.init_state = ", p_arm_head->init_state);
        // hal_debug_variable("sharespace_reinit: arm_head.write_addr = ", p_arm_head->write_addr);
        // hal_debug_variable("sharespace_reinit: arm_head.read_addr  = ", p_arm_head->read_addr);

        if ((p_arm_head->init_state == 1 || p_arm_head->init_state == 2) &&
            p_arm_head->write_addr != 0xa5a5a5a5 &&
            p_arm_head->read_addr != 0xa5a5a5a5) {

            if (p_arm_head->init_state == 2) {
                p_arm_head->init_state = 1;
                memcpy((void*)arm_head_ptr, p_arm_head, sizeof(MsgHead));
                dcache_region_writeback((void*)arm_head_ptr, sizeof(MsgHead));
            }

            hal_debug_variable("sharespace_reinit: arm_head.init_state = ", p_arm_head->init_state);
            hal_debug_variable("sharespace_reinit: arm_head.write_addr = ", p_arm_head->write_addr);
            hal_debug_variable("sharespace_reinit: arm_head.read_addr  = ", p_arm_head->read_addr);
            hal_debug_print("sharespace_reinit: Sync complete.\n");
            return;
        }
        delay_us(2000); // sleep 2 seconds between iterations
    }
}

// Main initialization function for the shared memory communication.
void sharespace_init(void) {
    // hal_debug_print("sharespace_init: Starting initialization.\n");
    sharespace_get_config(&dts_sharespace);
    // sharespace_fake_init();

    // Temporarily point to the initial handshake location in the DTS-defined sharespace
    arm_head_ptr = (volatile MsgHead*)(dts_sharespace.arm_write_addr + SHARE_SPACE_HEAD_OFFSET);
    // hal_debug_variable("sharespace_init: arm_head_ptr initially set to ", (uint32_t)arm_head_ptr);

    sharespace_clear();
    // hal_debug_print("sharespace_init: Got config from DTS.\n");
    // hal_debug_variable("sharespace_init: dsp_write_addr = ", dts_sharespace.dsp_write_addr);
    // hal_debug_variable("sharespace_init: arm_write_addr = ", dts_sharespace.arm_write_addr);

    // hal_debug_print("sharespace_init: Calling sharespace_reinit() to get kbuf addresses.\n");
    MsgHead initial_arm_head;
    sharespace_reinit(&initial_arm_head);
    // hal_debug_print("sharespace_init: sharespace_reinit() returned.\n");

    // hal_debug_variable("sharespace_init: Got kbuf addresses from ARM: read_addr  = ", initial_arm_head.read_addr);
    // hal_debug_variable("sharespace_init: Got kbuf addresses from ARM: write_addr = ", initial_arm_head.write_addr);

    // Now that we have the kbuf addresses, update pointers to point to the kbuf memory regions.
    // ARM's write_addr is the buffer for ARM->DSP communication.
    // ARM's read_addr is the buffer for DSP->ARM communication.
    dsp_writes_to_arm = (volatile uint8_t*)initial_arm_head.read_addr;
    dsp_reads_from_arm = (volatile uint8_t*)initial_arm_head.write_addr;

    // Update head pointers to point to the new locations within kbuf
    arm_head_ptr = (volatile MsgHead*)(dsp_reads_from_arm + SHARE_SPACE_HEAD_OFFSET);
    dsp_head_ptr = (volatile MsgHead*)(dsp_writes_to_arm + SHARE_SPACE_HEAD_OFFSET);

    hal_debug_print("sharespace_init: Pointers updated to kbuf regions.\n");
    hal_debug_variable("sharespace_init: dsp_reads_from_arm (ARM->DSP buffer) is now at ", (uint32_t)dsp_reads_from_arm);
    hal_debug_variable("sharespace_init: dsp_writes_to_arm  (DSP->ARM buffer) is now at ", (uint32_t)dsp_writes_to_arm);
    hal_debug_variable("sharespace_init: arm_head_ptr is now at ", (uint32_t)arm_head_ptr);
    hal_debug_variable("sharespace_init: dsp_head_ptr is now at ", (uint32_t)dsp_head_ptr);

    MsgHead dsp_head = {
        .read_addr = MIN_ADDR,
        .write_addr = MIN_ADDR,
        .init_state = 1
    };
    // hal_debug_variable("sharespace_init: Initializing DSP head: read_addr  = ", dsp_head.read_addr);
    // hal_debug_variable("sharespace_init: Initializing DSP head: write_addr = ", dsp_head.write_addr);
    // hal_debug_variable("sharespace_init: Initializing DSP head: init_state = ", dsp_head.init_state);

    memcpy((void*)dsp_head_ptr, &dsp_head, sizeof(MsgHead));
    // hal_debug_variable("sharespace_init: Wrote DSP head to ", (uint32_t)dsp_head_ptr);
    dcache_region_writeback((void*)dsp_head_ptr, sizeof(MsgHead));
    // hal_debug_variable("sharespace_init: Flushed cache for DSP head region at ", (uint32_t)dsp_head_ptr);
    // hal_debug_variable("    size ", sizeof(MsgHead));

    msgbox_send_signal((dsp_head.write_addr<<16) + dsp_head.read_addr);

    MsgHead arm_head;
    hal_debug_print("sharespace_init: Waiting for ARM to acknowledge with init_state=1.\n");
    while (1) {
        dcache_region_invalidate((void*)arm_head_ptr, sizeof(MsgHead));
        memcpy(&arm_head, (const void*)arm_head_ptr, sizeof(MsgHead));
        // hal_debug_variable("sharespace_init: Polling ARM head (from kbuf): init_state = ", arm_head.init_state);
        if (arm_head.init_state == 1) {
            // hal_debug_variable("sharespace_init: ARM acknowledged. read_addr  = ", arm_head.read_addr);
            // hal_debug_variable("sharespace_init: ARM acknowledged. write_addr = ", arm_head.write_addr);
            break;
        }
    }
    hal_debug_print("sharespace_init: Initialization complete.\n");
}
DECL_INIT(sharespace_init);

// Invalidates the ARM's message head in shared memory.
void sharespace_clear(void) {
    memset((void*)arm_head_ptr, 0xa5, sizeof(MsgHead));
    dcache_region_writeback((void*)arm_head_ptr, sizeof(MsgHead));

    // MsgHead arm_head;
    // memcpy(&arm_head, (const void*)arm_head_ptr, sizeof(MsgHead));
    // arm_head.init_state = 2;
    // // hal_debug_variable("sharespace_clear: Initializing ARM head: read_addr  = ", arm_head.read_addr);
    // // hal_debug_variable("sharespace_clear: Initializing ARM head: write_addr = ", arm_head.write_addr);
    // // hal_debug_variable("sharespace_clear: Initializing ARM head: init_state = ", arm_head.init_state);
    // memcpy((void*)arm_head_ptr, &arm_head, sizeof(MsgHead));
    // // hal_debug_variable("sharespace_clear: Wrote ARM head to ", (uint32_t)arm_head_ptr);
}

int sharespace_write(const void* data, int len) {
    MsgHead dsp_head, arm_head;
    
    // Invalidate ARM head cache to get fresh data
    dcache_region_invalidate((void*)arm_head_ptr, sizeof(MsgHead));
    
    memcpy(&dsp_head, (const void*)dsp_head_ptr, sizeof(MsgHead));
    memcpy(&arm_head, (const void*)arm_head_ptr, sizeof(MsgHead));

    uint32_t host_read_addr = arm_head.read_addr; // ARM's read position in DSP->ARM buffer
    uint32_t local_write_addr = dsp_head.write_addr;

    // Test using arm read addr from msg
    // host_read_addr = msgbox_new_msg[0];

    // hal_debug_variable("sharespace_write: host_read_addr   = ", host_read_addr);
    // hal_debug_variable("sharespace_write: local_write_addr = ", local_write_addr);
    // hal_debug_variable("sharespace_write: len = ", len);

    int free_size;
    if (host_read_addr <= local_write_addr) {
        free_size = MAX_ADDR - MIN_ADDR - (local_write_addr - host_read_addr);
    } else {
        free_size = host_read_addr - local_write_addr;
    }

    // hal_debug_variable("sharespace_write: free_size = ", free_size);

    if (free_size <= len) {
        lprintf("sharespace_write: buffer full, cannot write\n");
        return -1;
    }

    const uint8_t* src = (const uint8_t*)data;
    if (local_write_addr + len <= MAX_ADDR) {
        memcpy((void*)(dsp_writes_to_arm + local_write_addr), src, len);
        local_write_addr += len;
        if (local_write_addr >= MAX_ADDR) {
            local_write_addr = MIN_ADDR;
        }
    } else {
        int len1 = MAX_ADDR - local_write_addr;
        memcpy((void*)(dsp_writes_to_arm + local_write_addr), src, len1);
        int len2 = len - len1;
        memcpy((void*)(dsp_writes_to_arm + MIN_ADDR), src + len1, len2);
        local_write_addr = MIN_ADDR + len2;
    }

    dsp_head.write_addr = local_write_addr;
    memcpy((void*)dsp_head_ptr, &dsp_head, sizeof(MsgHead));
    
    // Flush written data and head to memory
    dcache_region_writeback((void*)(dsp_writes_to_arm + MIN_ADDR), MAX_ADDR - MIN_ADDR);
    dcache_region_writeback((void*)dsp_head_ptr, sizeof(MsgHead));

    // hal_debug_variable("sharespace_write: new write_addr = ", dsp_head.write_addr);

    msgbox_send_signal((dsp_head.write_addr<<16) + dsp_head.read_addr);

    return len;
}

int sharespace_read(void* out_buffer, int max_len) {
    MsgHead dsp_head, arm_head;
    
    // Invalidate ARM head to read fresh data from ARM
    dcache_region_invalidate((void*)arm_head_ptr, sizeof(MsgHead));
    
    memcpy(&arm_head, (const void*)arm_head_ptr, sizeof(MsgHead));
    memcpy(&dsp_head, (const void*)dsp_head_ptr, sizeof(MsgHead));

    uint32_t host_write_addr = arm_head.write_addr; // ARM's write position in ARM->DSP buffer
    uint32_t local_read_addr = dsp_head.read_addr;   // DSP's read position in ARM->DSP buffer

    // hal_debug_variable("sharespace_read: host_write_addr = ", host_write_addr);
    // hal_debug_variable("sharespace_read: local_read_addr = ", local_read_addr);

    if (local_read_addr == host_write_addr) {
        // lprintf("sharespace_read: no data available\n");
        return 0;
    }

    int msg_size;
    if (local_read_addr < host_write_addr) {
        msg_size = host_write_addr - local_read_addr;
    } else { 
        msg_size = (MAX_ADDR - local_read_addr) + (host_write_addr - MIN_ADDR);
    }

    // hal_debug_variable("sharespace_read: msg_size = ", msg_size);
    // hal_debug_variable("    max_len = ", max_len);

    // Invalidate data cache to read fresh data from ARM
    dcache_region_invalidate((void*)(dsp_reads_from_arm + MIN_ADDR), MAX_ADDR - MIN_ADDR);

    int bytes_to_copy = (msg_size < max_len) ? msg_size : max_len;
    uint8_t* dest = (uint8_t*)out_buffer;

    if (local_read_addr + bytes_to_copy <= MAX_ADDR) {
        memcpy(dest, (const void*)(dsp_reads_from_arm + local_read_addr), bytes_to_copy);
        local_read_addr += bytes_to_copy;
        if (local_read_addr >= MAX_ADDR) {
            local_read_addr = MIN_ADDR;
        }
    } else {
        int len1 = MAX_ADDR - local_read_addr;
        memcpy(dest, (const void*)(dsp_reads_from_arm + local_read_addr), len1);
        int len2 = bytes_to_copy - len1;
        memcpy(dest + len1, (const void*)(dsp_reads_from_arm + MIN_ADDR), len2);
        local_read_addr = MIN_ADDR + len2;
    }

    dsp_head.read_addr = local_read_addr;
    memcpy((void*)dsp_head_ptr, &dsp_head, sizeof(MsgHead));
    dcache_region_writeback((void*)dsp_head_ptr, sizeof(MsgHead));

    // hal_debug_variable("sharespace_read: number of read bytes ", bytes_to_copy);
    // hal_debug_variable("sharespace_read: new read_addr = ", dsp_head.read_addr);

    // msgbox_send_signal((dsp_head.write_addr<<16) + dsp_head.read_addr);

    return bytes_to_copy;
}
