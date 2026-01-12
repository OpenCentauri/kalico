/**
 * @file msgbox.c
 * @brief Message Box driver for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * Implements inter-processor communication between ARM and DSP cores.
 * 
 * Architecture:
 * - DSP MSGBOX base: 0x01701000
 * - ARM MSGBOX base: 0x03003000
 * - Both access the same shared FIFOs from different perspectives
 * 
 * For DSP <-> ARM communication (N=0, remote CPU = ARM):
 * - There are 4 channels (P=0-3)
 * - Each channel has an 8-deep FIFO of 32-bit messages
 * - user1 (DSP) is transmitter, user0 (ARM) is receiver on same channel
 * 
 * Register layout (for N=0, ARM):
 * - 0x0020: RD_IRQ_EN   - RX interrupt enable (DSP receives from ARM)
 * - 0x0024: RD_IRQ_STATUS - RX interrupt status
 * - 0x0030: WR_IRQ_EN   - TX interrupt enable (DSP sends to ARM)
 * - 0x0034: WR_IRQ_STATUS - TX interrupt status
 * - 0x0040: DEBUG_REG
 * - 0x0050+P*4: FIFO_STATUS[P] - FIFO full flag
 * - 0x0060+P*4: MSG_STATUS[P]  - Number of messages in FIFO
 * - 0x0070+P*4: MSG[P]         - Message read/write register
 */

#include "hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

static struct {
    msgbox_rx_callback_t rx_callback;
    msgbox_tx_callback_t tx_callback;
    void *arg;
} msgbox_handlers[MSGBOX_NUM_CHANNELS];

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline void msgbox_write_reg(bool remote, uint32_t offset, uint32_t value)
{
    if (remote)
        REG32(MSGBOX_ARM_BASE + offset) = value;
    else
        REG32(MSGBOX_DSP_BASE + offset) = value;
}

static inline uint32_t msgbox_read_reg(bool remote, uint32_t offset)
{
    if (remote)
        return REG32(MSGBOX_ARM_BASE + offset);
    else
        return REG32(MSGBOX_DSP_BASE + offset);
}

/**
 * @brief Internal IRQ handler for MSGBOX
 * 
 * Handles both RX (message received from ARM) and TX (FIFO has space) interrupts.
 */
static void msgbox_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    
    /* Check RX interrupts (messages received from ARM) */
    uint32_t rd_status = msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_RD_IRQ_STATUS(MSGBOX_RECEIVE));
    
    for (uint8_t ch = 0; ch < MSGBOX_NUM_CHANNELS; ch++) {
        if (rd_status & MSGBOX_RD_IRQ_CH(ch)) {
            /* Clear interrupt by writing 1 */
            msgbox_write_reg(MSGBOX_LOCAL, MSGBOX_RD_IRQ_STATUS(MSGBOX_RECEIVE), MSGBOX_RD_IRQ_CH(ch));
            
            /* Read all available messages from this channel */
            uint32_t msg_count = msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG_STATUS(MSGBOX_RECEIVE, ch)) & MSGBOX_MSG_NUM_MASK;
            
            while (msg_count > 0) {
                uint32_t message = msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG(MSGBOX_RECEIVE, ch));
                
                /* Call user callback if registered */
                if (msgbox_handlers[ch].rx_callback) {
                    msgbox_handlers[ch].rx_callback(ch, message, msgbox_handlers[ch].arg);
                }
                
                msg_count = msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG_STATUS(MSGBOX_RECEIVE, ch)) & MSGBOX_MSG_NUM_MASK;
            }
        }
    }
    
    /* Check TX interrupts (FIFO has space, can send more) */
    uint32_t wr_status = msgbox_read_reg(MSGBOX_REMOTE, MSGBOX_WR_IRQ_STATUS(MSGBOX_TRANSMIT));
    
    for (uint8_t ch = 0; ch < MSGBOX_NUM_CHANNELS; ch++) {
        if (wr_status & MSGBOX_WR_IRQ_CH(ch)) {
            /* Clear interrupt by writing 1 */
            msgbox_write_reg(MSGBOX_REMOTE, MSGBOX_WR_IRQ_STATUS(MSGBOX_TRANSMIT), MSGBOX_WR_IRQ_CH(ch));
            
            /* Call user callback if registered */
            if (msgbox_handlers[ch].tx_callback) {
                msgbox_handlers[ch].tx_callback(ch, msgbox_handlers[ch].arg);
            }
        }
    }
}

/*============================================================================
 * Public Functions
 *============================================================================*/

void msgbox_init(void)
{
    /* Enable clock and deassert reset */
    ccu_msgbox_enable();
    
    /* Clear all handlers */
    for (int i = 0; i < MSGBOX_NUM_CHANNELS; i++) {
        msgbox_handlers[i].rx_callback = NULL;
        msgbox_handlers[i].tx_callback = NULL;
        msgbox_handlers[i].arg = NULL;
    }
    
    /* Disable all RX and TX interrupts */
    msgbox_write_reg(MSGBOX_LOCAL, MSGBOX_RD_IRQ_EN(MSGBOX_RECEIVE), 0);
    msgbox_write_reg(MSGBOX_REMOTE, MSGBOX_WR_IRQ_EN(MSGBOX_TRANSMIT), 0);
    
    /* Clear any pending interrupts */
    msgbox_write_reg(MSGBOX_LOCAL, MSGBOX_RD_IRQ_STATUS(MSGBOX_RECEIVE), 0xFFFFFFFF);
    msgbox_write_reg(MSGBOX_REMOTE, MSGBOX_WR_IRQ_STATUS(MSGBOX_TRANSMIT), 0xFFFFFFFF);
    
    /* Register system IRQ handler */
    irq_register(IRQ_MSGBOX, msgbox_irq_handler, NULL);
    irq_enable_interrupt(IRQ_MSGBOX);
}

int msgbox_send(uint8_t channel, uint32_t message)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return -1;
    }
    
    /* Check if FIFO is full */
    if (msgbox_read_reg(MSGBOX_REMOTE, MSGBOX_FIFO_STATUS(MSGBOX_TRANSMIT, channel)) & MSGBOX_FIFO_FULL_FLAG) {
        return -1; /* FIFO full */
    }
    
    /* Write message to FIFO */
    msgbox_write_reg(MSGBOX_REMOTE, MSGBOX_MSG(MSGBOX_TRANSMIT, channel), message);
    
    return 0;
}

void msgbox_send_blocking(uint8_t channel, uint32_t message)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    /* Wait until FIFO has space (not full) */
    while (msgbox_read_reg(MSGBOX_REMOTE, MSGBOX_FIFO_STATUS(MSGBOX_TRANSMIT, channel)) & MSGBOX_FIFO_FULL_FLAG) {
        /* Busy wait */
    }
    
    /* Write message */
    msgbox_write_reg(MSGBOX_REMOTE, MSGBOX_MSG(MSGBOX_TRANSMIT, channel), message);
}

int msgbox_recv(uint8_t channel, uint32_t *message)
{
    if (channel >= MSGBOX_NUM_CHANNELS || message == NULL) {
        return -1;
    }
    
    /* Check if FIFO has data */
    uint32_t msg_count = msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG_STATUS(MSGBOX_RECEIVE, channel)) & MSGBOX_MSG_NUM_MASK;
    if (msg_count == 0) {
        return -1; /* FIFO empty */
    }
    
    /* Read message from FIFO */
    *message = msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG(MSGBOX_RECEIVE, channel));
    
    return 0;
}

uint32_t msgbox_recv_blocking(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return 0;
    }
    
    /* Wait until FIFO has data */
    while ((msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG_STATUS(MSGBOX_RECEIVE, channel)) & MSGBOX_MSG_NUM_MASK) == 0) {
        /* Busy wait */
    }
    
    return msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG(MSGBOX_RECEIVE, channel));
}

uint8_t msgbox_rx_pending(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return 0;
    }
    
    return msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG_STATUS(MSGBOX_RECEIVE, channel)) & MSGBOX_MSG_NUM_MASK;
}

bool msgbox_tx_ready(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return false;
    }
    
    /* Ready if FIFO is not full */
    return (msgbox_read_reg(MSGBOX_REMOTE, MSGBOX_FIFO_STATUS(MSGBOX_TRANSMIT, channel)) & MSGBOX_FIFO_FULL_FLAG) == 0;
}

void msgbox_set_rx_callback(uint8_t channel, msgbox_rx_callback_t callback, void *arg)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    msgbox_handlers[channel].rx_callback = callback;
    msgbox_handlers[channel].arg = arg;
}

void msgbox_set_tx_callback(uint8_t channel, msgbox_tx_callback_t callback, void *arg)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    msgbox_handlers[channel].tx_callback = callback;
    msgbox_handlers[channel].arg = arg;
}

void msgbox_enable_rx_irq(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    uint32_t en = msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_RD_IRQ_EN(MSGBOX_RECEIVE));
    en |= MSGBOX_RD_IRQ_CH(channel);
    msgbox_write_reg(MSGBOX_LOCAL, MSGBOX_RD_IRQ_EN(MSGBOX_RECEIVE), en);
}

void msgbox_disable_rx_irq(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    uint32_t en = msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_RD_IRQ_EN(MSGBOX_RECEIVE));
    en &= ~MSGBOX_RD_IRQ_CH(channel);
    msgbox_write_reg(MSGBOX_LOCAL, MSGBOX_RD_IRQ_EN(MSGBOX_RECEIVE), en);
}

void msgbox_enable_tx_irq(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    uint32_t en = msgbox_read_reg(MSGBOX_REMOTE, MSGBOX_WR_IRQ_EN(MSGBOX_TRANSMIT));
    en |= MSGBOX_WR_IRQ_CH(channel);
    msgbox_write_reg(MSGBOX_REMOTE, MSGBOX_WR_IRQ_EN(MSGBOX_TRANSMIT), en);
}

void msgbox_disable_tx_irq(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    uint32_t en = msgbox_read_reg(MSGBOX_REMOTE, MSGBOX_WR_IRQ_EN(MSGBOX_TRANSMIT));
    en &= ~MSGBOX_WR_IRQ_CH(channel);
    msgbox_write_reg(MSGBOX_REMOTE, MSGBOX_WR_IRQ_EN(MSGBOX_TRANSMIT), en);
}

void msgbox_flush_rx(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    /* Read and discard all messages in the FIFO */
    while ((msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG_STATUS(MSGBOX_RECEIVE, channel)) & MSGBOX_MSG_NUM_MASK) > 0) {
        (void)msgbox_read_reg(MSGBOX_LOCAL, MSGBOX_MSG(MSGBOX_RECEIVE, channel));
    }
    
    /* Clear any pending RX interrupt */
    msgbox_write_reg(MSGBOX_LOCAL, MSGBOX_RD_IRQ_STATUS(MSGBOX_RECEIVE), MSGBOX_RD_IRQ_CH(channel));
}