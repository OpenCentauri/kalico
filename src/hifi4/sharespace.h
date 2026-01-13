#ifndef __HIF4_SHARESPACE_H
#define __HIF4_SHARESPACE_H

#include <stdint.h>

// Total size of one-way buffer region.
#define BUFFER_SIZE 4096

#define SYS_IDLE_STATE           0xcbda0001
#define DSP_READ_STATE           0xcbda0002
#define DSP_WRITE_STATE          0xcbda0003
#define ARM_READ_STATE           0xcbda0004
#define ARM_WRITE_STATE          0xcbda0005
#define SHARESPACE_READ          0
#define SHARESPACE_WRITE         1

extern uint16_t sharespace_arm_addr[2];
extern uint16_t sharespace_dsp_addr[2];

// Define the structure for the message head, matching the ARM side.
// The order of fields is critical for memory layout compatibility.
typedef struct __attribute__((packed)) {
    volatile uint32_t read_addr;
    volatile uint32_t write_addr;
    volatile uint32_t init_state;
} MsgHead;

// The offset to the MsgHead at the end of the buffer region.
#define SHARE_SPACE_HEAD_OFFSET (BUFFER_SIZE - sizeof(MsgHead))

// The data buffer does not start at 0. It starts after where the head would be if it were at the beginning.
#define MIN_ADDR sizeof(MsgHead)

// The data buffer ends just before the actual MsgHead at the end of the region.
#define MAX_ADDR (BUFFER_SIZE - sizeof(MsgHead))

void sharespace_fake_init(void);
void sharespace_init(void);
void sharespace_clear(void);
int sharespace_write(const void* data, int len);
int sharespace_read(void* out_buffer, int max_len);

#endif // sharespace.h
