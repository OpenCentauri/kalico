/**
 * @file msgbox.c
 * @brief Message Box driver for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * Implements inter-processor communication between ARM and DSP cores.
 * 
 * The MSGBOX has 8 unidirectional channels:
 * - Even channels (0,2,4,6): ARM -> DSP (DSP receives)
 * - Odd channels (1,3,5,7): DSP -> ARM (DSP transmits)
 * 
 * Each channel has a 4-deep FIFO for 32-bit messages.
 * 
 * Interrupt types:
 * - RX interrupt: Fires when a message is received (FIFO not empty)
 * - TX interrupt: Fires when FIFO has space (remote side read a message)
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

static inline void msgbox_write_reg(uint32_t offset, uint32_t value)
{
    REG32(MSGBOX_BASE + offset) = value;
}

static inline uint32_t msgbox_read_reg(uint32_t offset)
{
    return REG32(MSGBOX_BASE + offset);
}

/**
 * @brief Internal IRQ handler for MSGBOX
 */
static void msgbox_irq_handler(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    
    /* Read interrupt status for DSP (user 1) */
    uint32_t status = msgbox_read_reg(MSGBOX_IRQ_STATUS(MSGBOX_USER_DSP));
    
    /* Process each channel */
    for (uint8_t ch = 0; ch < MSGBOX_NUM_CHANNELS; ch++) {
        /* Check RX interrupt (message received) */
        if (status & MSGBOX_IRQ_RX_PEND(ch)) {
            /* Clear interrupt first */
            msgbox_write_reg(MSGBOX_IRQ_STATUS(MSGBOX_USER_DSP), MSGBOX_IRQ_RX_PEND(ch));
            
            /* Read all available messages from this channel */
            while (msgbox_read_reg(MSGBOX_FIFO_STATUS(ch)) & MSGBOX_FIFO_NOT_EMPTY) {
                uint32_t message = msgbox_read_reg(MSGBOX_MSG(ch));
                
                /* Call user callback if registered */
                if (msgbox_handlers[ch].rx_callback) {
                    msgbox_handlers[ch].rx_callback(ch, message, msgbox_handlers[ch].arg);
                }
            }
        }
        
        /* Check TX interrupt (FIFO has space) */
        if (status & MSGBOX_IRQ_TX_PEND(ch)) {
            /* Clear interrupt */
            msgbox_write_reg(MSGBOX_IRQ_STATUS(MSGBOX_USER_DSP), MSGBOX_IRQ_TX_PEND(ch));
            
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
    
    /* Disable all interrupts for DSP */
    msgbox_write_reg(MSGBOX_IRQ_EN(MSGBOX_USER_DSP), 0);
    
    /* Clear any pending interrupts */
    msgbox_write_reg(MSGBOX_IRQ_STATUS(MSGBOX_USER_DSP), 0xFFFFFFFF);
    
    /* Register system IRQ handler */
    irq_register(IRQ_MSGBOX, msgbox_irq_handler, NULL);
    irq_enable_interrupt(IRQ_MSGBOX);
}

int msgbox_send(uint8_t channel, uint32_t message)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return -1;
    }
    
    /* Check if FIFO has space */
    if (!(msgbox_read_reg(MSGBOX_FIFO_STATUS(channel)) & MSGBOX_FIFO_NOT_FULL)) {
        return -1; /* FIFO full */
    }
    
    /* Write message to FIFO */
    msgbox_write_reg(MSGBOX_MSG(channel), message);
    
    return 0;
}

void msgbox_send_blocking(uint8_t channel, uint32_t message)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    /* Wait until FIFO has space */
    while (!(msgbox_read_reg(MSGBOX_FIFO_STATUS(channel)) & MSGBOX_FIFO_NOT_FULL)) {
        /* Busy wait */
    }
    
    /* Write message */
    msgbox_write_reg(MSGBOX_MSG(channel), message);
}

int msgbox_recv(uint8_t channel, uint32_t *message)
{
    if (channel >= MSGBOX_NUM_CHANNELS || message == NULL) {
        return -1;
    }
    
    /* Check if FIFO has data */
    if (!(msgbox_read_reg(MSGBOX_FIFO_STATUS(channel)) & MSGBOX_FIFO_NOT_EMPTY)) {
        return -1; /* FIFO empty */
    }
    
    /* Read message from FIFO */
    *message = msgbox_read_reg(MSGBOX_MSG(channel));
    
    return 0;
}

uint32_t msgbox_recv_blocking(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return 0;
    }
    
    /* Wait until FIFO has data */
    while (!(msgbox_read_reg(MSGBOX_FIFO_STATUS(channel)) & MSGBOX_FIFO_NOT_EMPTY)) {
        /* Busy wait */
    }
    
    return msgbox_read_reg(MSGBOX_MSG(channel));
}

uint8_t msgbox_rx_pending(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return 0;
    }
    
    uint32_t status = msgbox_read_reg(MSGBOX_FIFO_STATUS(channel));
    return (status & MSGBOX_FIFO_MSG_NUM_MASK) >> MSGBOX_FIFO_MSG_NUM_SHIFT;
}

bool msgbox_tx_ready(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return false;
    }
    
    return (msgbox_read_reg(MSGBOX_FIFO_STATUS(channel)) & MSGBOX_FIFO_NOT_FULL) != 0;
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
    
    uint32_t en = msgbox_read_reg(MSGBOX_IRQ_EN(MSGBOX_USER_DSP));
    en |= MSGBOX_IRQ_RX_PEND(channel);
    msgbox_write_reg(MSGBOX_IRQ_EN(MSGBOX_USER_DSP), en);
}

void msgbox_disable_rx_irq(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    uint32_t en = msgbox_read_reg(MSGBOX_IRQ_EN(MSGBOX_USER_DSP));
    en &= ~MSGBOX_IRQ_RX_PEND(channel);
    msgbox_write_reg(MSGBOX_IRQ_EN(MSGBOX_USER_DSP), en);
}

void msgbox_enable_tx_irq(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    uint32_t en = msgbox_read_reg(MSGBOX_IRQ_EN(MSGBOX_USER_DSP));
    en |= MSGBOX_IRQ_TX_PEND(channel);
    msgbox_write_reg(MSGBOX_IRQ_EN(MSGBOX_USER_DSP), en);
}

void msgbox_disable_tx_irq(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    uint32_t en = msgbox_read_reg(MSGBOX_IRQ_EN(MSGBOX_USER_DSP));
    en &= ~MSGBOX_IRQ_TX_PEND(channel);
    msgbox_write_reg(MSGBOX_IRQ_EN(MSGBOX_USER_DSP), en);
}

void msgbox_flush_rx(uint8_t channel)
{
    if (channel >= MSGBOX_NUM_CHANNELS) {
        return;
    }
    
    /* Read and discard all messages in the FIFO */
    while (msgbox_read_reg(MSGBOX_FIFO_STATUS(channel)) & MSGBOX_FIFO_NOT_EMPTY) {
        (void)msgbox_read_reg(MSGBOX_MSG(channel));
    }
    
    /* Clear any pending RX interrupt */
    msgbox_write_reg(MSGBOX_IRQ_STATUS(MSGBOX_USER_DSP), MSGBOX_IRQ_RX_PEND(channel));
}
