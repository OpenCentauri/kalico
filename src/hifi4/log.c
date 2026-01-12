#include "log.h"
#include <platform.h>
#include <stdio.h>
#include <stdarg.h>
#include "sched.h" // DECL_INIT
#include "util.h" // vsnprintf

static volatile uint8_t* log_buffer = NULL;
static uint32_t log_buffer_size = 0;
static volatile uint32_t* log_write_ptr = NULL;

// void log_fake_init(void) {
//     volatile struct spare_rtos_head_t *pstr = platform_head;
//     volatile struct dts_msg_t *pdts = &pstr->rtos_img_hdr.dts_msg;
//     if (pdts->dts_sharespace.status == DTS_OPEN) {
//         log_buffer = (volatile uint8_t*)pdts->dts_sharespace.dsp_log_addr;
//         log_buffer_size = pdts->dts_sharespace.dsp_log_size;
//     }
//     printf("DSP Log Sharespace Configuration:\n");
//     printf("  DSP Log Address:   0x%x\n", (unsigned int)log_buffer);
//     printf("  DSP Log Size:      0x%x\n", log_buffer_size);
//     /*if (log_buffer && log_buffer_size > 4) {
//         log_write_ptr = (volatile uint32_t*)log_buffer;
//         log_clear();
//     }*/
//     printf("DONE DSP LOG FAKEINIT!\n");
// }

void log_init(void) {
    volatile struct spare_rtos_head_t *pstr = platform_head;
    volatile struct dts_msg_t *pdts = &pstr->rtos_img_hdr.dts_msg;
    if (pdts->dts_sharespace.status == DTS_OPEN) {
        log_buffer = (volatile uint8_t*)pdts->dts_sharespace.dsp_log_addr;
        log_buffer_size = pdts->dts_sharespace.dsp_log_size;
    }

    if (log_buffer && log_buffer_size > 4) {
        log_write_ptr = (volatile uint32_t*)log_buffer;
        log_clear();
    }
    lprintf("DSP logging kbuf initialized!\n");
}
DECL_INIT(log_init);

void log_clear(void) {
    if (log_write_ptr) {
        *log_write_ptr = 4; // Start writing after the write pointer itself.
        memset((void*)(log_buffer + 4), 0, log_buffer_size - 4);
    }
}

void lprintf(const char *str) {
    if (!log_write_ptr) {
        return;
    }

    const char *str_tmp = str;
    int len = 0;
    while (*str_tmp++) {
        len++;
    }

    uint32_t current_write_pos = *log_write_ptr;

    if (current_write_pos + len + 1 >= log_buffer_size) {
        current_write_pos = 4;
    }

    memcpy((void*)(log_buffer + current_write_pos), str, len);
    *(log_buffer + current_write_pos + len) = '\0';

    *log_write_ptr = current_write_pos + len + 1;
}
