/**
 * @file gpadc.c
 * @brief GPADC driver implementation for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * This implements the General Purpose ADC driver with interrupt support.
 * T113 has 1 channel, D1 has 2 channels, R329/T507 have 4 channels.
 */

#include "hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* GPADC interrupt handlers */
static struct {
    irq_handler_t handler;
    void *arg;
} gpadc_handlers[GPADC_MAX_CHANNELS];

/* Track which channels are enabled */
static uint32_t enabled_channels = 0;

/* Current operating mode */
static gpadc_mode_t current_mode = GPADC_MODE_SINGLE;

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline void gpadc_write_reg(uint32_t offset, uint32_t value)
{
    REG32(GPADC_BASE + offset) = value;
}

static inline uint32_t gpadc_read_reg(uint32_t offset)
{
    return REG32(GPADC_BASE + offset);
}

/*============================================================================
 * GPADC Initialization
 *============================================================================*/

void gpadc_init(uint32_t sample_rate)
{
    uint32_t sr_con;
    uint32_t ctrl;
    
    /* Enable clock and deassert reset */
    ccu_gpadc_enable();
    
    /* 
     * Configure sample rate
     * The sample rate is determined by: fs_div and tacq
     * Actual sample rate = HOSC / ((fs_div + 1) * (tacq + 1))
     * HOSC is typically 24MHz
     * 
     * For simplicity, we'll use reasonable defaults:
     * fs_div = 4, tacq = 63 gives approximately 75 kHz base rate
     */
    uint32_t hosc_freq = 24000000; /* 24 MHz */
    uint32_t fs_div = 4;
    uint32_t tacq = 63;
    
    /* Adjust fs_div based on desired sample rate */
    if (sample_rate > 0) {
        /* Calculate: sample_rate = hosc_freq / ((fs_div + 1) * (tacq + 1))
         * fs_div = (hosc_freq / (sample_rate * (tacq + 1))) - 1 */
        uint32_t divisor = hosc_freq / (sample_rate * (tacq + 1));
        if (divisor > 0) {
            fs_div = divisor - 1;
        }
        if (fs_div > 0xFFFF) {
            fs_div = 0xFFFF;
        }
    }
    
    sr_con = (fs_div & 0xFFFF) | ((tacq & 0xFFFF) << 16);
    gpadc_write_reg(GPADC_SR_CON, sr_con);
    
    /* Configure control register:
     * - Enable auto-calibration
     * - Set single conversion mode initially
     */
    ctrl = GPADC_CTRL_ADC_AUTOCALI_EN | GPADC_CTRL_WORK_MODE_SINGLE;
    gpadc_write_reg(GPADC_CTRL, ctrl);
    
    /* Disable all channels initially */
    gpadc_write_reg(GPADC_CS_EN, 0);
    
    /* Clear any pending interrupts */
    gpadc_write_reg(GPADC_FIFO_INTS, 0xFFFFFFFF);
    gpadc_write_reg(GPADC_DATAL_INTS, 0xFFFFFFFF);
    gpadc_write_reg(GPADC_DATAH_INTS, 0xFFFFFFFF);
    gpadc_write_reg(GPADC_DATA_INTS, 0xFFFFFFFF);
    
    /* Disable all interrupts initially */
    gpadc_write_reg(GPADC_FIFO_INTC, 0);
    gpadc_write_reg(GPADC_DATAL_INTC, 0);
    gpadc_write_reg(GPADC_DATAH_INTC, 0);
    gpadc_write_reg(GPADC_DATA_INTC, 0);
    
    enabled_channels = 0;
    current_mode = GPADC_MODE_SINGLE;
}

/*============================================================================
 * Channel Enable/Disable
 *============================================================================*/

void gpadc_channel_enable(gpadc_channel_t channel)
{
    if (channel >= GPADC_MAX_CHANNELS) {
        return;
    }
    
    uint32_t cs_en = gpadc_read_reg(GPADC_CS_EN);
    cs_en |= GPADC_CS_EN_CH(channel);
    gpadc_write_reg(GPADC_CS_EN, cs_en);
    
    enabled_channels |= BIT(channel);
}

void gpadc_channel_disable(gpadc_channel_t channel)
{
    if (channel >= GPADC_MAX_CHANNELS) {
        return;
    }
    
    uint32_t cs_en = gpadc_read_reg(GPADC_CS_EN);
    cs_en &= ~GPADC_CS_EN_CH(channel);
    gpadc_write_reg(GPADC_CS_EN, cs_en);
    
    enabled_channels &= ~BIT(channel);
}

/*============================================================================
 * Single Conversion (Blocking)
 *============================================================================*/

uint16_t gpadc_read(gpadc_channel_t channel)
{
    uint32_t ctrl;
    uint32_t timeout = 100000;
    
    if (channel >= GPADC_MAX_CHANNELS) {
        return 0;
    }
    
    /* Make sure channel is enabled */
    gpadc_channel_enable(channel);
    
    /* Set single conversion mode */
    ctrl = gpadc_read_reg(GPADC_CTRL);
    ctrl &= ~GPADC_CTRL_WORK_MODE_MASK;
    ctrl |= GPADC_CTRL_WORK_MODE_SINGLE;
    ctrl |= GPADC_CTRL_ADC_EN;
    gpadc_write_reg(GPADC_CTRL, ctrl);
    
    /* Wait for data ready interrupt flag */
    while (timeout--) {
        uint32_t status = gpadc_read_reg(GPADC_DATA_INTS);
        if (status & GPADC_DATA_INTS_CH(channel)) {
            /* Clear interrupt flag */
            gpadc_write_reg(GPADC_DATA_INTS, GPADC_DATA_INTS_CH(channel));
            break;
        }
    }
    
    /* Read the data */
    uint32_t data = gpadc_read_reg(GPADC_CH0_DATA + channel * 4);
    
    /* Disable ADC */
    ctrl = gpadc_read_reg(GPADC_CTRL);
    ctrl &= ~GPADC_CTRL_ADC_EN;
    gpadc_write_reg(GPADC_CTRL, ctrl);
    
    return (uint16_t)(data & GPADC_MAX_VALUE);
}

uint32_t gpadc_read_mv(gpadc_channel_t channel)
{
    uint16_t raw = gpadc_read(channel);
    
    /* Convert to millivolts:
     * voltage_mv = raw * VREF_MV / MAX_VALUE */
    return (uint32_t)raw * GPADC_VREF_MV / GPADC_MAX_VALUE;
}

/*============================================================================
 * Continuous Conversion Mode
 *============================================================================*/

void gpadc_start_continuous(gpadc_channel_t channel)
{
    uint32_t ctrl;
    
    if (channel >= GPADC_MAX_CHANNELS) {
        return;
    }
    
    /* Enable the channel */
    gpadc_channel_enable(channel);
    
    /* Set continuous conversion mode */
    ctrl = gpadc_read_reg(GPADC_CTRL);
    ctrl &= ~GPADC_CTRL_WORK_MODE_MASK;
    ctrl |= GPADC_CTRL_WORK_MODE_CONT;
    ctrl |= GPADC_CTRL_ADC_EN;
    gpadc_write_reg(GPADC_CTRL, ctrl);
    
    current_mode = GPADC_MODE_CONTINUOUS;
}

void gpadc_stop_continuous(void)
{
    uint32_t ctrl = gpadc_read_reg(GPADC_CTRL);
    ctrl &= ~GPADC_CTRL_ADC_EN;
    gpadc_write_reg(GPADC_CTRL, ctrl);
    
    current_mode = GPADC_MODE_SINGLE;
}

uint16_t gpadc_read_data(gpadc_channel_t channel)
{
    if (channel >= GPADC_MAX_CHANNELS) {
        return 0;
    }
    
    uint32_t data = gpadc_read_reg(GPADC_CH0_DATA + channel * 4);
    return (uint16_t)(data & GPADC_MAX_VALUE);
}

/*============================================================================
 * Interrupt Handling
 *============================================================================*/

static void gpadc_irq_handler_internal(uint32_t irq, void *arg)
{
    (void)irq;
    (void)arg;
    
    uint32_t status = gpadc_read_reg(GPADC_DATA_INTS);
    uint32_t enabled = gpadc_read_reg(GPADC_DATA_INTC);
    
    /* Process pending interrupts */
    uint32_t pending = status & enabled;
    
    for (uint32_t ch = 0; ch < GPADC_MAX_CHANNELS; ch++) {
        if (pending & GPADC_DATA_INTS_CH(ch)) {
            /* Clear the interrupt flag */
            gpadc_write_reg(GPADC_DATA_INTS, GPADC_DATA_INTS_CH(ch));
            
            /* Call the handler if registered */
            if (gpadc_handlers[ch].handler) {
                gpadc_handlers[ch].handler(ch, gpadc_handlers[ch].arg);
            }
        }
    }
    
    /* Also check FIFO interrupts */
    uint32_t fifo_status = gpadc_read_reg(GPADC_FIFO_INTS);
    if (fifo_status & GPADC_FIFO_INTS_OVERRUN) {
        /* Clear overrun flag */
        gpadc_write_reg(GPADC_FIFO_INTS, GPADC_FIFO_INTS_OVERRUN);
    }
}

void gpadc_irq_enable(gpadc_channel_t channel, irq_handler_t handler, void *arg)
{
    if (channel >= GPADC_MAX_CHANNELS) {
        return;
    }
    
    /* Store handler */
    gpadc_handlers[channel].handler = handler;
    gpadc_handlers[channel].arg = arg;
    
    /* Register system IRQ handler (only once) */
    static bool irq_registered = false;
    if (!irq_registered) {
        irq_register(IRQ_GPADC, gpadc_irq_handler_internal, NULL);
        irq_enable_interrupt(IRQ_GPADC);
        irq_registered = true;
    }
    
    /* Enable data ready interrupt for this channel */
    uint32_t intc = gpadc_read_reg(GPADC_DATA_INTC);
    intc |= GPADC_DATA_INTC_CH(channel);
    gpadc_write_reg(GPADC_DATA_INTC, intc);
}

void gpadc_irq_disable(gpadc_channel_t channel)
{
    if (channel >= GPADC_MAX_CHANNELS) {
        return;
    }
    
    /* Disable data ready interrupt for this channel */
    uint32_t intc = gpadc_read_reg(GPADC_DATA_INTC);
    intc &= ~GPADC_DATA_INTC_CH(channel);
    gpadc_write_reg(GPADC_DATA_INTC, intc);
    
    /* Clear handler */
    gpadc_handlers[channel].handler = NULL;
    gpadc_handlers[channel].arg = NULL;
}

/*============================================================================
 * FIFO Operations (for burst/DMA mode)
 *============================================================================*/

/**
 * @brief Read data from FIFO
 * @return FIFO data (12-bit value + channel info)
 */
uint32_t gpadc_fifo_read(void)
{
    return gpadc_read_reg(GPADC_FIFO_DATA);
}

/**
 * @brief Check if FIFO has data
 * @return true if FIFO has data
 */
bool gpadc_fifo_has_data(void)
{
    uint32_t status = gpadc_read_reg(GPADC_FIFO_INTS);
    return (status & GPADC_FIFO_INTS_DATA) != 0;
}

/**
 * @brief Flush FIFO
 */
void gpadc_fifo_flush(void)
{
    uint32_t intc = gpadc_read_reg(GPADC_FIFO_INTC);
    intc |= GPADC_FIFO_INTC_FLUSH;
    gpadc_write_reg(GPADC_FIFO_INTC, intc);
    
    /* Clear flush bit */
    intc &= ~GPADC_FIFO_INTC_FLUSH;
    gpadc_write_reg(GPADC_FIFO_INTC, intc);
}

/*============================================================================
 * Comparison/Threshold Functions
 *============================================================================*/

/**
 * @brief Set comparison threshold for a channel
 * @param channel ADC channel
 * @param low_threshold Low threshold value (12-bit)
 * @param high_threshold High threshold value (12-bit)
 */
void gpadc_set_threshold(gpadc_channel_t channel, uint16_t low_threshold, 
                         uint16_t high_threshold)
{
    if (channel >= GPADC_MAX_CHANNELS) {
        return;
    }
    
    uint32_t cmp_data = ((uint32_t)high_threshold << 16) | (low_threshold & 0xFFFF);
    gpadc_write_reg(GPADC_CH0_CMP_DATA + channel * 4, cmp_data);
    
    /* Enable comparison for this channel */
    uint32_t cs_en = gpadc_read_reg(GPADC_CS_EN);
    cs_en |= GPADC_CS_EN_CMP(channel);
    gpadc_write_reg(GPADC_CS_EN, cs_en);
}

/**
 * @brief Enable low threshold interrupt
 * @param channel ADC channel
 */
void gpadc_low_threshold_irq_enable(gpadc_channel_t channel)
{
    if (channel >= GPADC_MAX_CHANNELS) {
        return;
    }
    
    uint32_t intc = gpadc_read_reg(GPADC_DATAL_INTC);
    intc |= BIT(channel);
    gpadc_write_reg(GPADC_DATAL_INTC, intc);
}

/**
 * @brief Enable high threshold interrupt
 * @param channel ADC channel
 */
void gpadc_high_threshold_irq_enable(gpadc_channel_t channel)
{
    if (channel >= GPADC_MAX_CHANNELS) {
        return;
    }
    
    uint32_t intc = gpadc_read_reg(GPADC_DATAH_INTC);
    intc |= BIT(channel);
    gpadc_write_reg(GPADC_DATAH_INTC, intc);
}
