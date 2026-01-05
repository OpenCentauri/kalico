/**
 * @file ccu.c
 * @brief Clock Control Unit driver for Allwinner R528/T113
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * Handles clock gating and reset control for peripherals.
 * Note: The DSP may have limited access to CCU registers depending
 * on system configuration. Some clocks may need to be enabled by
 * the ARM core before DSP startup.
 */

#include "hal.h"

/*============================================================================
 * Private Functions
 *============================================================================*/

static inline void ccu_write_reg(uint32_t offset, uint32_t value)
{
    REG32(CCU_BASE + offset) = value;
}

static inline uint32_t ccu_read_reg(uint32_t offset)
{
    return REG32(CCU_BASE + offset);
}

static inline void ccu_set_bits(uint32_t offset, uint32_t bits)
{
    uint32_t val = ccu_read_reg(offset);
    val |= bits;
    ccu_write_reg(offset, val);
}

static inline void ccu_clear_bits(uint32_t offset, uint32_t bits)
{
    uint32_t val = ccu_read_reg(offset);
    val &= ~bits;
    ccu_write_reg(offset, val);
}

/*============================================================================
 * UART Clock Control
 *============================================================================*/

void ccu_uart_enable(uint8_t uart_id)
{
    if (uart_id > 3) return;
    
    uint32_t gating_bit = BIT(uart_id);
    uint32_t reset_bit = BIT(16 + uart_id);
    
    /* Enable clock gating and deassert reset */
    ccu_set_bits(CCU_UART_BGR, gating_bit | reset_bit);
    
    /* Small delay for clock to stabilize */
    for (volatile int i = 0; i < 100; i++);
}

void ccu_uart_disable(uint8_t uart_id)
{
    if (uart_id > 3) return;
    
    uint32_t gating_bit = BIT(uart_id);
    uint32_t reset_bit = BIT(16 + uart_id);
    
    /* Assert reset and disable clock */
    ccu_clear_bits(CCU_UART_BGR, gating_bit | reset_bit);
}

/*============================================================================
 * I2C/TWI Clock Control
 *============================================================================*/

void ccu_twi_enable(uint8_t twi_id)
{
    if (twi_id > 3) return;
    
    uint32_t gating_bit = BIT(twi_id);
    uint32_t reset_bit = BIT(16 + twi_id);
    
    ccu_set_bits(CCU_TWI_BGR, gating_bit | reset_bit);
    
    for (volatile int i = 0; i < 100; i++);
}

void ccu_twi_disable(uint8_t twi_id)
{
    if (twi_id > 3) return;
    
    uint32_t gating_bit = BIT(twi_id);
    uint32_t reset_bit = BIT(16 + twi_id);
    
    ccu_clear_bits(CCU_TWI_BGR, gating_bit | reset_bit);
}

/*============================================================================
 * SPI Clock Control
 *============================================================================*/

void ccu_spi_enable(uint8_t spi_id, uint32_t clk_hz, uint32_t pclk_hz)
{
    if (spi_id > 1) return;
    
    uint32_t clk_reg = (spi_id == 0) ? CCU_SPI0_CLK : CCU_SPI1_CLK;
    uint32_t gating_bit = BIT(spi_id);
    uint32_t reset_bit = BIT(16 + spi_id);
    
    /* Configure SPI module clock
     * Clock = source / (N+1) / (M+1)
     * where N is 0-3 (factor 1,2,4,8) and M is 0-15
     */
    uint32_t clk_cfg = CCU_SPI_CLK_EN;
    
    if (pclk_hz <= 24000000) {
        /* Use HOSC (24 MHz) */
        clk_cfg |= CCU_SPI_CLK_SRC_HOSC;
        pclk_hz = 24000000;
    } else {
        /* Use PLL */
        clk_cfg |= CCU_SPI_CLK_SRC_PLL;
    }
    
    /* Find best divider */
    uint32_t div = (pclk_hz + clk_hz - 1) / clk_hz;
    uint8_t factor_n = 0;
    uint8_t factor_m = 0;
    
    /* Try to minimize N (coarse divider) and use M for fine tuning */
    for (factor_n = 0; factor_n < 4; factor_n++) {
        uint32_t n_div = 1 << factor_n;
        factor_m = (div / n_div);
        if (factor_m > 0) factor_m--;
        if (factor_m <= 15) break;
    }
    
    if (factor_m > 15) factor_m = 15;
    
    clk_cfg |= CCU_SPI_CLK_FACTOR_N(factor_n);
    clk_cfg |= CCU_SPI_CLK_FACTOR_M(factor_m);
    
    /* Write clock configuration */
    ccu_write_reg(clk_reg, clk_cfg);
    
    /* Enable bus gating and deassert reset */
    ccu_set_bits(CCU_SPI_BGR, gating_bit | reset_bit);
    
    for (volatile int i = 0; i < 100; i++);
}

void ccu_spi_disable(uint8_t spi_id)
{
    if (spi_id > 1) return;
    
    uint32_t clk_reg = (spi_id == 0) ? CCU_SPI0_CLK : CCU_SPI1_CLK;
    uint32_t gating_bit = BIT(spi_id);
    uint32_t reset_bit = BIT(16 + spi_id);
    
    /* Disable module clock */
    ccu_clear_bits(clk_reg, CCU_SPI_CLK_EN);
    
    /* Assert reset and disable bus clock */
    ccu_clear_bits(CCU_SPI_BGR, gating_bit | reset_bit);
}

/*============================================================================
 * GPADC Clock Control
 *============================================================================*/

void ccu_gpadc_enable(void)
{
    ccu_set_bits(CCU_GPADC_BGR, CCU_GPADC_GATING | CCU_GPADC_RST);
    
    for (volatile int i = 0; i < 100; i++);
}

void ccu_gpadc_disable(void)
{
    ccu_clear_bits(CCU_GPADC_BGR, CCU_GPADC_GATING | CCU_GPADC_RST);
}

/*============================================================================
 * PWM Clock Control
 *============================================================================*/

void ccu_pwm_enable(void)
{
    ccu_set_bits(CCU_PWM_BGR, CCU_PWM_GATING | CCU_PWM_RST);
    
    for (volatile int i = 0; i < 100; i++);
}

void ccu_pwm_disable(void)
{
    ccu_clear_bits(CCU_PWM_BGR, CCU_PWM_GATING | CCU_PWM_RST);
}

/*============================================================================
 * MSGBOX Clock Control
 *============================================================================*/

void ccu_msgbox_enable(void)
{
    ccu_set_bits(CCU_MSGBOX_BGR, CCU_MSGBOX_GATING | CCU_MSGBOX_RST);
    
    for (volatile int i = 0; i < 100; i++);
}

void ccu_msgbox_disable(void)
{
    ccu_clear_bits(CCU_MSGBOX_BGR, CCU_MSGBOX_GATING | CCU_MSGBOX_RST);
}

/*============================================================================
 * HSTIMER Clock Control
 *============================================================================*/

void ccu_hstimer_enable(void)
{
    ccu_set_bits(CCU_HSTIMER_BGR, CCU_HSTIMER_GATING | CCU_HSTIMER_RST);
    
    for (volatile int i = 0; i < 100; i++);
}

void ccu_hstimer_disable(void)
{
    ccu_clear_bits(CCU_HSTIMER_BGR, CCU_HSTIMER_GATING | CCU_HSTIMER_RST);
}
