/**
 * @file i2c.c
 * @brief I2C/TWI driver for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 */

#include "hal.h"

/*============================================================================
 * Private Definitions
 *============================================================================*/

#define I2C_TIMEOUT     100000

/* I2C base addresses */
static const uint32_t i2c_bases[4] = {
    TWI0_BASE,
    TWI1_BASE,
    TWI2_BASE,
    TWI3_BASE,
};

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline uint32_t i2c_base(i2c_id_t id)
{
    return i2c_bases[id];
}

static inline void i2c_reg_write(i2c_id_t id, uint32_t offset, uint32_t value)
{
    REG32(i2c_base(id) + offset) = value;
}

static inline uint32_t i2c_reg_read(i2c_id_t id, uint32_t offset)
{
    return REG32(i2c_base(id) + offset);
}

static int i2c_wait_flag(i2c_id_t id)
{
    uint32_t timeout = I2C_TIMEOUT;
    
    while (timeout--) {
        if (i2c_reg_read(id, TWI_CNTR) & TWI_CNTR_INT_FLAG) {
            return 0;
        }
    }
    
    return -1; /* Timeout */
}

static void i2c_clear_flag(i2c_id_t id)
{
    uint32_t cntr = i2c_reg_read(id, TWI_CNTR);
    /* Writing 1 to INT_FLAG clears it, preserve other bits */
    cntr |= TWI_CNTR_INT_FLAG;
    i2c_reg_write(id, TWI_CNTR, cntr);
}

static int i2c_send_start(i2c_id_t id)
{
    uint32_t cntr = i2c_reg_read(id, TWI_CNTR);
    cntr |= TWI_CNTR_STA;
    cntr |= TWI_CNTR_INT_FLAG; /* Clear any pending flag */
    i2c_reg_write(id, TWI_CNTR, cntr);
    
    if (i2c_wait_flag(id) < 0) {
        return -1;
    }
    
    uint32_t status = i2c_reg_read(id, TWI_STAT);
    if (status != TWI_STAT_START && status != TWI_STAT_REP_START) {
        return -1;
    }
    
    return 0;
}

static int i2c_send_stop(i2c_id_t id)
{
    uint32_t cntr = i2c_reg_read(id, TWI_CNTR);
    cntr |= TWI_CNTR_STP;
    cntr |= TWI_CNTR_INT_FLAG;
    i2c_reg_write(id, TWI_CNTR, cntr);
    
    /* Wait for STOP to complete */
    uint32_t timeout = I2C_TIMEOUT;
    while (timeout--) {
        if (!(i2c_reg_read(id, TWI_CNTR) & TWI_CNTR_STP)) {
            break;
        }
    }
    
    return 0;
}

static int i2c_send_byte(i2c_id_t id, uint8_t byte)
{
    i2c_reg_write(id, TWI_DATA, byte);
    i2c_clear_flag(id);
    
    if (i2c_wait_flag(id) < 0) {
        return -1;
    }
    
    return 0;
}

static int i2c_recv_byte(i2c_id_t id, uint8_t *byte, bool ack)
{
    uint32_t cntr = i2c_reg_read(id, TWI_CNTR);
    
    if (ack) {
        cntr |= TWI_CNTR_ACK;
    } else {
        cntr &= ~TWI_CNTR_ACK;
    }
    
    i2c_reg_write(id, TWI_CNTR, cntr);
    i2c_clear_flag(id);
    
    if (i2c_wait_flag(id) < 0) {
        return -1;
    }
    
    *byte = (uint8_t)i2c_reg_read(id, TWI_DATA);
    return 0;
}

/*============================================================================
 * Public Functions
 *============================================================================*/

int i2c_init(i2c_id_t i2c_id, const i2c_config_t *config, uint32_t pclk_hz)
{
    if (i2c_id > I2C_3 || config == NULL) {
        return -1;
    }
    
    /* Enable clock and deassert reset */
    ccu_twi_enable(i2c_id);
    
    /* Software reset */
    i2c_reg_write(i2c_id, TWI_SRST, TWI_SRST_RESET);
    while (i2c_reg_read(i2c_id, TWI_SRST) & TWI_SRST_RESET) {
        /* Wait for reset to complete */
    }
    
    /* Calculate clock divider
     * I2C clock = pclk / (2^CLK_M * (CLK_N + 1) * 10)
     * For 100 kHz with 24 MHz: M=2, N=11 -> 24M / (4 * 12 * 10) = 50 kHz (close enough)
     * For 400 kHz with 24 MHz: M=1, N=2 -> 24M / (2 * 3 * 10) = 400 kHz
     */
    uint32_t clk_m = 0;
    uint32_t clk_n = 0;
    uint32_t target = config->speed_hz;
    
    /* Find best divider */
    for (clk_m = 0; clk_m < 8; clk_m++) {
        uint32_t div_m = 1 << clk_m;
        clk_n = (pclk_hz / (target * 10 * div_m)) - 1;
        if (clk_n <= 15) {
            break;
        }
    }
    
    if (clk_n > 15) clk_n = 15;
    
    uint32_t ccr = (clk_m << 3) | clk_n;
    i2c_reg_write(i2c_id, TWI_CCR, ccr);
    
    /* Enable bus */
    uint32_t cntr = TWI_CNTR_BUS_EN;
    i2c_reg_write(i2c_id, TWI_CNTR, cntr);
    
    return 0;
}

int i2c_write_addr(i2c_id_t i2c_id, uint8_t addr, const uint8_t *data, uint32_t len)
{
    int ret;
    
    /* Send START */
    ret = i2c_send_start(i2c_id);
    if (ret < 0) goto error;
    
    /* Send address + write bit */
    ret = i2c_send_byte(i2c_id, (addr << 1) | 0);
    if (ret < 0) goto error;
    
    uint32_t status = i2c_reg_read(i2c_id, TWI_STAT);
    if (status != TWI_STAT_ADDR_WR_ACK) {
        ret = -1;
        goto error;
    }
    
    /* Send data */
    for (uint32_t i = 0; i < len; i++) {
        ret = i2c_send_byte(i2c_id, data[i]);
        if (ret < 0) goto error;
        
        status = i2c_reg_read(i2c_id, TWI_STAT);
        if (status != TWI_STAT_DATA_WR_ACK) {
            ret = -1;
            goto error;
        }
    }
    
    i2c_send_stop(i2c_id);
    return 0;
    
error:
    i2c_send_stop(i2c_id);
    return ret;
}

int i2c_read_addr(i2c_id_t i2c_id, uint8_t addr, uint8_t *data, uint32_t len)
{
    int ret;
    
    /* Send START */
    ret = i2c_send_start(i2c_id);
    if (ret < 0) goto error;
    
    /* Send address + read bit */
    ret = i2c_send_byte(i2c_id, (addr << 1) | 1);
    if (ret < 0) goto error;
    
    uint32_t status = i2c_reg_read(i2c_id, TWI_STAT);
    if (status != TWI_STAT_ADDR_RD_ACK) {
        ret = -1;
        goto error;
    }
    
    /* Read data */
    for (uint32_t i = 0; i < len; i++) {
        bool ack = (i < len - 1); /* NACK on last byte */
        ret = i2c_recv_byte(i2c_id, &data[i], ack);
        if (ret < 0) goto error;
    }
    
    i2c_send_stop(i2c_id);
    return 0;
    
error:
    i2c_send_stop(i2c_id);
    return ret;
}

int i2c_write_read(i2c_id_t i2c_id, uint8_t addr,
                   const uint8_t *tx_data, uint32_t tx_len,
                   uint8_t *rx_data, uint32_t rx_len)
{
    int ret;
    
    /* Send START */
    ret = i2c_send_start(i2c_id);
    if (ret < 0) goto error;
    
    /* Send address + write bit */
    ret = i2c_send_byte(i2c_id, (addr << 1) | 0);
    if (ret < 0) goto error;
    
    uint32_t status = i2c_reg_read(i2c_id, TWI_STAT);
    if (status != TWI_STAT_ADDR_WR_ACK) {
        ret = -1;
        goto error;
    }
    
    /* Send data */
    for (uint32_t i = 0; i < tx_len; i++) {
        ret = i2c_send_byte(i2c_id, tx_data[i]);
        if (ret < 0) goto error;
        
        status = i2c_reg_read(i2c_id, TWI_STAT);
        if (status != TWI_STAT_DATA_WR_ACK) {
            ret = -1;
            goto error;
        }
    }
    
    /* Repeated START */
    ret = i2c_send_start(i2c_id);
    if (ret < 0) goto error;
    
    /* Send address + read bit */
    ret = i2c_send_byte(i2c_id, (addr << 1) | 1);
    if (ret < 0) goto error;
    
    status = i2c_reg_read(i2c_id, TWI_STAT);
    if (status != TWI_STAT_ADDR_RD_ACK) {
        ret = -1;
        goto error;
    }
    
    /* Read data */
    for (uint32_t i = 0; i < rx_len; i++) {
        bool ack = (i < rx_len - 1);
        ret = i2c_recv_byte(i2c_id, &rx_data[i], ack);
        if (ret < 0) goto error;
    }
    
    i2c_send_stop(i2c_id);
    return 0;
    
error:
    i2c_send_stop(i2c_id);
    return ret;
}

int i2c_write_reg(i2c_id_t i2c_id, uint8_t addr, uint8_t reg,
                  const uint8_t *data, uint32_t len)
{
    /* Combine register address and data into one write */
    int ret;
    
    ret = i2c_send_start(i2c_id);
    if (ret < 0) goto error;
    
    ret = i2c_send_byte(i2c_id, (addr << 1) | 0);
    if (ret < 0) goto error;
    
    uint32_t status = i2c_reg_read(i2c_id, TWI_STAT);
    if (status != TWI_STAT_ADDR_WR_ACK) {
        ret = -1;
        goto error;
    }
    
    /* Send register address */
    ret = i2c_send_byte(i2c_id, reg);
    if (ret < 0) goto error;
    
    status = i2c_reg_read(i2c_id, TWI_STAT);
    if (status != TWI_STAT_DATA_WR_ACK) {
        ret = -1;
        goto error;
    }
    
    /* Send data */
    for (uint32_t i = 0; i < len; i++) {
        ret = i2c_send_byte(i2c_id, data[i]);
        if (ret < 0) goto error;
        
        status = i2c_reg_read(i2c_id, TWI_STAT);
        if (status != TWI_STAT_DATA_WR_ACK) {
            ret = -1;
            goto error;
        }
    }
    
    i2c_send_stop(i2c_id);
    return 0;
    
error:
    i2c_send_stop(i2c_id);
    return ret;
}

int i2c_read_reg(i2c_id_t i2c_id, uint8_t addr, uint8_t reg,
                 uint8_t *data, uint32_t len)
{
    return i2c_write_read(i2c_id, addr, &reg, 1, data, len);
}
