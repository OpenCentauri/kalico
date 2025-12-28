/**
 * @file spi.c
 * @brief SPI driver for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 */

#include "hal.h"

/*============================================================================
 * Private Definitions
 *============================================================================*/

#define SPI_TIMEOUT     100000
#define SPI_FIFO_DEPTH  64

static const uint32_t spi_bases[2] = {
    SPI0_BASE,
    SPI1_BASE,
};

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline uint32_t spi_base(spi_id_t id)
{
    return spi_bases[id];
}

static inline void spi_write_reg(spi_id_t id, uint32_t offset, uint32_t value)
{
    REG32(spi_base(id) + offset) = value;
}

static inline uint32_t spi_read_reg(spi_id_t id, uint32_t offset)
{
    return REG32(spi_base(id) + offset);
}

static void spi_reset_fifos(spi_id_t id)
{
    uint32_t fcr = spi_read_reg(id, SPI_FCR);
    fcr |= SPI_FCR_RX_RST | SPI_FCR_TX_RST;
    spi_write_reg(id, SPI_FCR, fcr);
    
    /* Wait for reset to complete */
    uint32_t timeout = SPI_TIMEOUT;
    while (timeout--) {
        fcr = spi_read_reg(id, SPI_FCR);
        if (!(fcr & (SPI_FCR_RX_RST | SPI_FCR_TX_RST))) {
            break;
        }
    }
}

static uint32_t spi_get_rx_count(spi_id_t id)
{
    return spi_read_reg(id, SPI_FSR) & SPI_FSR_RX_CNT_MASK;
}

static uint32_t spi_get_tx_count(spi_id_t id)
{
    return (spi_read_reg(id, SPI_FSR) & SPI_FSR_TX_CNT_MASK) >> 16;
}

// static int spi_wait_tx_ready(spi_id_t id)
// {
//     uint32_t timeout = SPI_TIMEOUT;
//     while (timeout--) {
//         if (spi_get_tx_count(id) < SPI_FIFO_DEPTH) {
//             return 0;
//         }
//     }
//     return -1;
// }

static int spi_wait_transfer_complete(spi_id_t id)
{
    uint32_t timeout = SPI_TIMEOUT;
    while (timeout--) {
        uint32_t isr = spi_read_reg(id, SPI_ISR);
        if (isr & SPI_INT_TC) {
            /* Clear TC flag */
            spi_write_reg(id, SPI_ISR, SPI_INT_TC);
            return 0;
        }
    }
    return -1;
}

/*============================================================================
 * Public Functions
 *============================================================================*/

int spi_init(spi_id_t spi_id, const spi_config_t *config, uint32_t pclk_hz)
{
    if (spi_id > SPI_1 || config == NULL) {
        return -1;
    }
    
    /* Enable clock and deassert reset */
    ccu_spi_enable(spi_id, config->speed_hz, pclk_hz);
    
    /* Soft reset */
    spi_write_reg(spi_id, SPI_GCR, SPI_GCR_SRST);
    uint32_t timeout = SPI_TIMEOUT;
    while (timeout--) {
        if (!(spi_read_reg(spi_id, SPI_GCR) & SPI_GCR_SRST)) {
            break;
        }
    }
    
    /* Enable SPI in master mode */
    uint32_t gcr = SPI_GCR_EN | SPI_GCR_MODE_MASTER | SPI_GCR_TP_EN;
    spi_write_reg(spi_id, SPI_GCR, gcr);
    
    /* Configure transfer control */
    uint32_t tcr = 0;
    
    /* Clock polarity and phase */
    switch (config->mode) {
    case SPI_MODE_0:
        /* CPOL=0, CPHA=0 */
        break;
    case SPI_MODE_1:
        /* CPOL=0, CPHA=1 */
        tcr |= SPI_TCR_CPHA;
        break;
    case SPI_MODE_2:
        /* CPOL=1, CPHA=0 */
        tcr |= SPI_TCR_CPOL;
        break;
    case SPI_MODE_3:
        /* CPOL=1, CPHA=1 */
        tcr |= SPI_TCR_CPOL | SPI_TCR_CPHA;
        break;
    }
    
    /* CS active low */
    tcr |= SPI_TCR_SPOL;
    
    /* Software CS control */
    tcr |= SPI_TCR_SS_OWNER;
    tcr |= SPI_TCR_SS_LEVEL; /* Deassert by default */
    
    /* LSB/MSB first */
    if (config->lsb_first) {
        tcr |= SPI_TCR_FBS;
    }
    
    /* Discard hash burst - full duplex */
    /* tcr |= SPI_TCR_DHB; */ /* Set this for TX-only to save RX FIFO space */
    
    spi_write_reg(spi_id, SPI_TCR, tcr);
    
    /* Calculate clock divider
     * SPI clock = pclk / (2 * (n + 1))
     * n = (pclk / (2 * spi_clk)) - 1
     */
    uint32_t div = (pclk_hz / (2 * config->speed_hz));
    if (div > 0) div--;
    if (div > 0xFF) div = 0xFF;
    
    /* CDR2 uses linear divider */
    uint32_t ccr = (1 << 12) | (div & 0xFF); /* CDR2 mode, divider value */
    spi_write_reg(spi_id, SPI_CCR, ccr);
    
    /* Clear FIFOs */
    spi_reset_fifos(spi_id);
    
    /* Set FIFO trigger levels */
    uint32_t fcr = (SPI_FIFO_DEPTH / 2) | ((SPI_FIFO_DEPTH / 2) << 16);
    spi_write_reg(spi_id, SPI_FCR, fcr);
    
    /* Clear any pending interrupts */
    spi_write_reg(spi_id, SPI_ISR, 0xFFFFFFFF);
    
    /* Disable interrupts (we use polling) */
    spi_write_reg(spi_id, SPI_IER, 0);
    
    return 0;
}

void spi_cs_control(spi_id_t spi_id, uint8_t cs_num, bool active)
{
    uint32_t tcr = spi_read_reg(spi_id, SPI_TCR);
    
    /* Select chip select */
    tcr &= ~SPI_TCR_SS_MASK;
    tcr |= SPI_TCR_SS(cs_num & 0x03);
    
    /* Set level (active low, so active=true means low=0) */
    if (active) {
        tcr &= ~SPI_TCR_SS_LEVEL;
    } else {
        tcr |= SPI_TCR_SS_LEVEL;
    }
    
    spi_write_reg(spi_id, SPI_TCR, tcr);
}

int spi_write_read(spi_id_t spi_id, const uint8_t *tx_data, uint8_t *rx_data, uint32_t len)
{
    if (len == 0) return 0;
    
    /* Reset FIFOs */
    spi_reset_fifos(spi_id);
    
    /* Clear pending interrupts */
    spi_write_reg(spi_id, SPI_ISR, 0xFFFFFFFF);
    
    /* Set burst counters */
    spi_write_reg(spi_id, SPI_MBC, len);  /* Total bytes */
    spi_write_reg(spi_id, SPI_MTC, len);  /* TX bytes (for full duplex) */
    spi_write_reg(spi_id, SPI_BCC, len);  /* Single burst */
    
    /* Fill TX FIFO */
    uint32_t tx_count = 0;
    uint32_t rx_count = 0;
    
    while (tx_count < len && tx_count < SPI_FIFO_DEPTH) {
        uint8_t byte = tx_data ? tx_data[tx_count] : 0xFF;
        REG8(spi_base(spi_id) + SPI_TXD) = byte;
        tx_count++;
    }
    
    /* Start transfer */
    uint32_t tcr = spi_read_reg(spi_id, SPI_TCR);
    tcr |= SPI_TCR_XCH;
    spi_write_reg(spi_id, SPI_TCR, tcr);
    
    /* Continue filling TX and draining RX until done */
    while (rx_count < len) {
        /* Read any available RX data */
        while (rx_count < len && spi_get_rx_count(spi_id) > 0) {
            uint8_t byte = REG8(spi_base(spi_id) + SPI_RXD);
            if (rx_data) {
                rx_data[rx_count] = byte;
            }
            rx_count++;
        }
        
        /* Write more TX data if needed and space available */
        while (tx_count < len && spi_get_tx_count(spi_id) < SPI_FIFO_DEPTH) {
            uint8_t byte = tx_data ? tx_data[tx_count] : 0xFF;
            REG8(spi_base(spi_id) + SPI_TXD) = byte;
            tx_count++;
        }
    }
    
    /* Wait for transfer complete */
    if (spi_wait_transfer_complete(spi_id) < 0) {
        return -1;
    }
    
    return 0;
}

int spi_write(spi_id_t spi_id, const uint8_t *data, uint32_t len)
{
    return spi_write_read(spi_id, data, NULL, len);
}

int spi_read(spi_id_t spi_id, uint8_t *data, uint32_t len)
{
    return spi_write_read(spi_id, NULL, data, len);
}

uint8_t spi_write_read_byte(spi_id_t spi_id, uint8_t tx_byte)
{
    uint8_t rx_byte = 0;
    spi_write_read(spi_id, &tx_byte, &rx_byte, 1);
    return rx_byte;
}
