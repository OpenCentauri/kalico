/**
 * @file hal.h
 * @brief Simple HAL for Allwinner R528/T113 HiFi4 DSP
 * 
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * A minimal baremetal HAL providing GPIO, UART, and ADC functionality
 * with interrupt support for the HiFi4 DSP core.
 * 
 * Register addresses are based on the R528/T113 User Manual.
 */

#ifndef R528_HIFI4_HAL_H
#define R528_HIFI4_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*============================================================================
 * Memory-Mapped I/O Helpers
 *============================================================================*/

#define REG32(addr)         (*(volatile uint32_t *)(addr))
#define REG8(addr)          (*(volatile uint8_t *)(addr))

#define BIT(n)              (1UL << (n))
#define GENMASK(h, l)       (((1UL << ((h) - (l) + 1)) - 1) << (l))

/*============================================================================
 * Base Addresses (R528/T113)
 *============================================================================*/

/* GPIO/PIO Controller - handles all GPIO ports */
#define PIO_BASE            0x02000000UL

/* UART Controllers */
#define UART0_BASE          0x02500000UL
#define UART1_BASE          0x02500400UL
#define UART2_BASE          0x02500800UL
#define UART3_BASE          0x02500C00UL

/* General Purpose ADC */
#define GPADC_BASE          0x02009000UL

/* Clock Control Unit */
#define CCU_BASE            0x02001000UL

/* Timer (32-bit down counters) */
#define TIMER_BASE          0x02050000UL

/* High-Speed Timer */
#define HSTIMER_BASE        0x02040000UL

/* Watchdog */
#define WDOG_BASE           0x02050020UL

/* I2C/TWI Controllers */
#define TWI0_BASE           0x02502000UL
#define TWI1_BASE           0x02502400UL
#define TWI2_BASE           0x02502800UL
#define TWI3_BASE           0x02502C00UL

/* SPI Controllers */
#define SPI0_BASE           0x04025000UL
#define SPI1_BASE           0x04026000UL

/* PWM Controller */
#define PWM_BASE            0x02000C00UL

/* Message Box (for ARM <-> DSP communication) */
#define MSGBOX_BASE         0x01701000UL    /* DSP MSGBOX base */
#define MSGBOX_ARM_BASE     0x03003000UL    /* ARM MSGBOX base (for reference) */

/*============================================================================
 * CCU (Clock Control Unit) Definitions
 *============================================================================*/

/* Bus Clock Gating Registers */
#define CCU_UART_BGR        0x090C      /* UART Bus Gating Reset */
#define CCU_TWI_BGR         0x091C      /* TWI/I2C Bus Gating Reset */
#define CCU_SPI0_CLK        0x0940      /* SPI0 Clock */
#define CCU_SPI1_CLK        0x0944      /* SPI1 Clock */
#define CCU_SPI_BGR         0x096C      /* SPI Bus Gating Reset */
#define CCU_GPADC_BGR       0x09EC      /* GPADC Bus Gating Reset */
#define CCU_THS_BGR         0x09FC      /* THS Bus Gating Reset */
#define CCU_PWM_BGR         0x0C0C      /* PWM Bus Gating Reset */
#define CCU_TIMER_BGR       0x0850      /* Timer Bus Gating Reset (found in some docs) */
#define CCU_MSGBOX_BGR      0x071C      /* MSGBOX Bus Gating Reset */
#define CCU_HSTIMER_BGR     0x073C      /* HSTIMER Bus Gating Reset */

/* For peripherals, bit 0 is usually gating, bit 16 is usually reset */
#define CCU_BGR_GATING(n)   BIT(n)      /* Clock gating enable */
#define CCU_BGR_RST(n)      BIT(16 + (n)) /* Reset deassert */

/* UART BGR bits */
#define CCU_UART0_GATING    BIT(0)
#define CCU_UART1_GATING    BIT(1)
#define CCU_UART2_GATING    BIT(2)
#define CCU_UART3_GATING    BIT(3)
#define CCU_UART0_RST       BIT(16)
#define CCU_UART1_RST       BIT(17)
#define CCU_UART2_RST       BIT(18)
#define CCU_UART3_RST       BIT(19)

/* TWI/I2C BGR bits */
#define CCU_TWI0_GATING     BIT(0)
#define CCU_TWI1_GATING     BIT(1)
#define CCU_TWI2_GATING     BIT(2)
#define CCU_TWI3_GATING     BIT(3)
#define CCU_TWI0_RST        BIT(16)
#define CCU_TWI1_RST        BIT(17)
#define CCU_TWI2_RST        BIT(18)
#define CCU_TWI3_RST        BIT(19)

/* SPI BGR bits */
#define CCU_SPI0_GATING     BIT(0)
#define CCU_SPI1_GATING     BIT(1)
#define CCU_SPI0_RST        BIT(16)
#define CCU_SPI1_RST        BIT(17)

/* SPI Clock Register bits */
#define CCU_SPI_CLK_EN          BIT(31)     /* Clock enable */
#define CCU_SPI_CLK_SRC_MASK    (0x7 << 24) /* Clock source */
#define CCU_SPI_CLK_SRC_HOSC    (0 << 24)   /* 24 MHz */
#define CCU_SPI_CLK_SRC_PLL     (1 << 24)   /* PLL_PERI(1X) */
#define CCU_SPI_CLK_FACTOR_N(n) ((n) << 8)  /* Factor N (0-3: /1, /2, /4, /8) */
#define CCU_SPI_CLK_FACTOR_M(m) ((m) << 0)  /* Factor M (0-15: /(M+1)) */

/* GPADC BGR bits */
#define CCU_GPADC_GATING    BIT(0)
#define CCU_GPADC_RST       BIT(16)

/* PWM BGR bits */
#define CCU_PWM_GATING      BIT(0)
#define CCU_PWM_RST         BIT(16)

/* MSGBOX BGR register and bits */
#define CCU_MSGBOX_GATING   BIT(0)
#define CCU_MSGBOX_RST      BIT(16)

/* HSTIMER BGR bits */
#define CCU_HSTIMER_GATING  BIT(0)
#define CCU_HSTIMER_RST     BIT(16)

/*============================================================================
 * GPIO Definitions
 *============================================================================*/

/* GPIO Port offsets from PIO_BASE */
#define GPIO_PORT_OFFSET(n)     (0x30 * (n))

/* GPIO Port Register offsets (relative to port base) */
#define GPIO_CFG0           0x00    /* Configure Register 0 (pins 0-7) */
#define GPIO_CFG1           0x04    /* Configure Register 1 (pins 8-15) */
#define GPIO_CFG2           0x08    /* Configure Register 2 (pins 16-23) */
#define GPIO_CFG3           0x0C    /* Configure Register 3 (pins 24-31) */
#define GPIO_DAT            0x10    /* Data Register */
#define GPIO_DRV0           0x14    /* Multi-Driving Register 0 */
#define GPIO_DRV1           0x18    /* Multi-Driving Register 1 */
#define GPIO_DRV2           0x1C    /* Multi-Driving Register 2 */
#define GPIO_DRV3           0x20    /* Multi-Driving Register 3 */
#define GPIO_PULL0          0x24    /* Pull Register 0 */
#define GPIO_PULL1          0x28    /* Pull Register 1 */

/* GPIO External Interrupt Register offsets (per port) */
#define GPIO_EINT_BASE(n)       (PIO_BASE + 0x200 + (n) * 0x20)
#define GPIO_EINT_CFG0          0x00    /* External Interrupt Configure 0 */
#define GPIO_EINT_CFG1          0x04    /* External Interrupt Configure 1 */
#define GPIO_EINT_CFG2          0x08    /* External Interrupt Configure 2 */
#define GPIO_EINT_CFG3          0x0C    /* External Interrupt Configure 3 */
#define GPIO_EINT_CTL           0x10    /* External Interrupt Control */
#define GPIO_EINT_STATUS        0x14    /* External Interrupt Status */
#define GPIO_EINT_DEB           0x18    /* External Interrupt Debounce */

/* GPIO ports */
typedef enum {
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2,
    GPIO_PORT_D = 3,
    GPIO_PORT_E = 4,
    GPIO_PORT_F = 5,
    GPIO_PORT_G = 6,
} gpio_port_t;

/* GPIO modes */
typedef enum {
    GPIO_MODE_INPUT     = 0,
    GPIO_MODE_OUTPUT    = 1,
    GPIO_MODE_FUNC2     = 2,    /* Alternate function 2 */
    GPIO_MODE_FUNC3     = 3,    /* Alternate function 3 */
    GPIO_MODE_FUNC4     = 4,    /* Alternate function 4 */
    GPIO_MODE_FUNC5     = 5,    /* Alternate function 5 */
    GPIO_MODE_EINT      = 6,    /* External interrupt */
    GPIO_MODE_DISABLED  = 7,
} gpio_mode_t;

/* GPIO pull configuration */
typedef enum {
    GPIO_PULL_NONE      = 0,
    GPIO_PULL_UP        = 1,
    GPIO_PULL_DOWN      = 2,
} gpio_pull_t;

/* GPIO interrupt trigger types */
typedef enum {
    GPIO_IRQ_RISING     = 0,
    GPIO_IRQ_FALLING    = 1,
    GPIO_IRQ_HIGH       = 2,
    GPIO_IRQ_LOW        = 3,
    GPIO_IRQ_BOTH       = 4,
} gpio_irq_trigger_t;

/* GPIO pin descriptor */
typedef struct {
    gpio_port_t port;
    uint8_t     pin;
} gpio_pin_t;

/*============================================================================
 * UART Definitions (16550-compatible)
 *============================================================================*/

/* UART Register offsets */
#define UART_RBR            0x00    /* Receive Buffer Register (read, DLAB=0) */
#define UART_THR            0x00    /* Transmit Holding Register (write, DLAB=0) */
#define UART_DLL            0x00    /* Divisor Latch Low (DLAB=1) */
#define UART_DLH            0x04    /* Divisor Latch High (DLAB=1) */
#define UART_IER            0x04    /* Interrupt Enable Register (DLAB=0) */
#define UART_IIR            0x08    /* Interrupt Identification Register (read) */
#define UART_FCR            0x08    /* FIFO Control Register (write) */
#define UART_LCR            0x0C    /* Line Control Register */
#define UART_MCR            0x10    /* Modem Control Register */
#define UART_LSR            0x14    /* Line Status Register */
#define UART_MSR            0x18    /* Modem Status Register */
#define UART_SCH            0x1C    /* Scratch Register */
#define UART_USR            0x7C    /* UART Status Register */
#define UART_TFL            0x80    /* Transmit FIFO Level */
#define UART_RFL            0x84    /* Receive FIFO Level */
#define UART_HALT           0xA4    /* Halt TX Register */

/* UART IER bits */
#define UART_IER_RDI        BIT(0)  /* Receive Data Available Interrupt */
#define UART_IER_THRI       BIT(1)  /* THR Empty Interrupt */
#define UART_IER_RLSI       BIT(2)  /* Receive Line Status Interrupt */
#define UART_IER_MSI        BIT(3)  /* Modem Status Interrupt */
#define UART_IER_PTIME      BIT(7)  /* Programmable THR Interrupt Mode */

/* UART IIR bits */
#define UART_IIR_NO_INT     BIT(0)  /* No interrupt pending */
#define UART_IIR_ID_MASK    0x0F    /* Interrupt ID mask */
#define UART_IIR_MSI        0x00    /* Modem status interrupt */
#define UART_IIR_THRI       0x02    /* THR empty interrupt */
#define UART_IIR_RDI        0x04    /* Receive data available */
#define UART_IIR_RLSI       0x06    /* Receive line status */
#define UART_IIR_BUSY       0x07    /* Busy detect */
#define UART_IIR_CTI        0x0C    /* Character timeout */

/* UART FCR bits */
#define UART_FCR_FIFO_EN    BIT(0)  /* FIFO Enable */
#define UART_FCR_RXSR       BIT(1)  /* Receiver FIFO Reset */
#define UART_FCR_TXSR       BIT(2)  /* Transmitter FIFO Reset */
#define UART_FCR_DMA_MODE   BIT(3)  /* DMA Mode Select */
#define UART_FCR_TFT_MASK   (3 << 4) /* TX FIFO Trigger Level */
#define UART_FCR_RFT_MASK   (3 << 6) /* RX FIFO Trigger Level */

/* UART LCR bits */
#define UART_LCR_DLS_MASK   0x03    /* Data Length Select */
#define UART_LCR_DLS_5      0x00    /* 5 data bits */
#define UART_LCR_DLS_6      0x01    /* 6 data bits */
#define UART_LCR_DLS_7      0x02    /* 7 data bits */
#define UART_LCR_DLS_8      0x03    /* 8 data bits */
#define UART_LCR_STOP       BIT(2)  /* Stop bit (0=1 stop, 1=2 stop) */
#define UART_LCR_PEN        BIT(3)  /* Parity Enable */
#define UART_LCR_EPS        BIT(4)  /* Even Parity Select */
#define UART_LCR_BC         BIT(6)  /* Break Control */
#define UART_LCR_DLAB       BIT(7)  /* Divisor Latch Access */

/* UART LSR bits */
#define UART_LSR_DR         BIT(0)  /* Data Ready */
#define UART_LSR_OE         BIT(1)  /* Overrun Error */
#define UART_LSR_PE         BIT(2)  /* Parity Error */
#define UART_LSR_FE         BIT(3)  /* Framing Error */
#define UART_LSR_BI         BIT(4)  /* Break Interrupt */
#define UART_LSR_THRE       BIT(5)  /* THR Empty */
#define UART_LSR_TEMT       BIT(6)  /* Transmitter Empty */
#define UART_LSR_FIFOERR    BIT(7)  /* FIFO Data Error */

/* UART USR bits */
#define UART_USR_BUSY       BIT(0)  /* UART Busy */
#define UART_USR_TFNF       BIT(1)  /* TX FIFO Not Full */
#define UART_USR_TFE        BIT(2)  /* TX FIFO Empty */
#define UART_USR_RFNE       BIT(3)  /* RX FIFO Not Empty */
#define UART_USR_RFF        BIT(4)  /* RX FIFO Full */

/* UART configuration */
typedef enum {
    UART_0 = 0,
    UART_1 = 1,
    UART_2 = 2,
    UART_3 = 3,
} uart_id_t;

typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_ODD  = 1,
    UART_PARITY_EVEN = 2,
} uart_parity_t;

typedef struct {
    uint32_t        baudrate;
    uint8_t         data_bits;  /* 5, 6, 7, or 8 */
    uint8_t         stop_bits;  /* 1 or 2 */
    uart_parity_t   parity;
} uart_config_t;

/*============================================================================
 * GPADC Definitions
 *============================================================================*/

/* GPADC Register offsets */
#define GPADC_SR_CON        0x00    /* Sample Rate Configure Register */
#define GPADC_CTRL          0x04    /* Control Register */
#define GPADC_CS_EN         0x08    /* Compare and Select Enable Register */
#define GPADC_FIFO_INTC     0x0C    /* FIFO Interrupt Control Register */
#define GPADC_FIFO_INTS     0x10    /* FIFO Interrupt Status Register */
#define GPADC_FIFO_DATA     0x14    /* FIFO Data Register */
#define GPADC_CB_DATA       0x18    /* Calibration Data Register */
#define GPADC_DATAL_INTC    0x20    /* Data Low Interrupt Config Register */
#define GPADC_DATAH_INTC    0x24    /* Data High Interrupt Config Register */
#define GPADC_DATA_INTC     0x28    /* Data Interrupt Config Register */
#define GPADC_DATAL_INTS    0x30    /* Data Low Interrupt Status Register */
#define GPADC_DATAH_INTS    0x34    /* Data High Interrupt Status Register */
#define GPADC_DATA_INTS     0x38    /* Data Interrupt Status Register */
#define GPADC_CH0_CMP_DATA  0x40    /* Channel 0 Compare Data Register */
#define GPADC_CH0_DATA      0x80    /* Channel 0 Data Register */

/* GPADC_SR_CON bits */
#define GPADC_SR_CON_FS_DIV_MASK    GENMASK(15, 0)
#define GPADC_SR_CON_TACQ_MASK      GENMASK(31, 16)

/* GPADC_CTRL bits */
#define GPADC_CTRL_ADC_EN           BIT(0)
#define GPADC_CTRL_ADC_AUTOCALI_EN  BIT(1)
#define GPADC_CTRL_WORK_MODE_MASK   GENMASK(5, 4)
#define GPADC_CTRL_WORK_MODE_SINGLE (0 << 4)
#define GPADC_CTRL_WORK_MODE_CONT   (2 << 4)
#define GPADC_CTRL_WORK_MODE_BURST  (3 << 4)
#define GPADC_CTRL_ADC_CALI_EN      BIT(17)

/* GPADC_CS_EN bits */
#define GPADC_CS_EN_CH(n)           BIT(n)
#define GPADC_CS_EN_CMP(n)          BIT((n) + 16)

/* GPADC_FIFO_INTC bits */
#define GPADC_FIFO_INTC_DATA_DRQ_EN BIT(16)
#define GPADC_FIFO_INTC_OVERRUN_EN  BIT(17)
#define GPADC_FIFO_INTC_DATA_IRQ_EN BIT(18)
#define GPADC_FIFO_INTC_FLUSH       BIT(21)

/* GPADC_FIFO_INTS bits */
#define GPADC_FIFO_INTS_OVERRUN     BIT(17)
#define GPADC_FIFO_INTS_DATA        BIT(18)

/* GPADC_DATA_INTC bits */
#define GPADC_DATA_INTC_CH(n)       BIT(n)

/* GPADC_DATA_INTS bits */
#define GPADC_DATA_INTS_CH(n)       BIT(n)

/* T113 has 1 channel, D1 has 2 channels, R329/T507 have 4 channels */
#define GPADC_MAX_CHANNELS  4

/* ADC resolution */
#define GPADC_RESOLUTION    12
#define GPADC_MAX_VALUE     ((1 << GPADC_RESOLUTION) - 1)

/* ADC reference voltage in mV (typically 1.8V) */
#define GPADC_VREF_MV       1800

typedef enum {
    GPADC_CHANNEL_0 = 0,
    GPADC_CHANNEL_1 = 1,
    GPADC_CHANNEL_2 = 2,
    GPADC_CHANNEL_3 = 3,
} gpadc_channel_t;

typedef enum {
    GPADC_MODE_SINGLE = 0,
    GPADC_MODE_CONTINUOUS = 1,
} gpadc_mode_t;

/*============================================================================
 * Interrupt Handling
 *============================================================================*/

/* HiFi4 DSP Interrupt Numbers (from DSP_CORE Interrupt Source table)
 * 
 * The HiFi4 DSP core has its own interrupt inputs. These are the interrupt
 * numbers as they appear to the DSP core.
 * 
 * Note: The DSP uses a 2-level interrupt scheme for many peripherals:
 * - Direct DSP core interrupts (e.g., TIMER0, TIMER1, HSTIMER0, HSTIMER1)
 * - DSP_INTC routed interrupts (most peripherals go through IRQ 17)
 * 
 * For INTC-routed interrupts, the handler for IRQ 17 (INTC) should check
 * DSP_INTC registers to determine which specific peripheral triggered it.
 */

/* Direct DSP Core interrupts (directly wired to core) */
#define IRQ_DSP_WDOG        13      /* DSP Watchdog */
#define IRQ_DSP_TZMA        14      /* DSP TZMA */
#define IRQ_HSTIMER0        15      /* High-speed timer 0 */
#define IRQ_HSTIMER1        16      /* High-speed timer 1 */
#define IRQ_INTC            17      /* External interrupt controller (DSP_INTC) */
#define IRQ_TIMER0          18      /* Timer 0 */
#define IRQ_TIMER1          22      /* Timer 1 */
#define IRQ_GPADC           20      /* GPADC (named GPA in table) */
#define IRQ_LRADC           21      /* LRADC */
#define IRQ_TPADC           22      /* TPADC */

/* DSP_INTC interrupt bit numbers (bits in DSP_INTC pending register)
 * These are dispatched when IRQ_INTC (17) fires.
 * Use with the INTC-specific handler.
 */
#define INTC_UART0          3
#define INTC_UART1          4
#define INTC_UART2          5
#define INTC_UART3          6
#define INTC_UART4          7
#define INTC_UART5          8
#define INTC_TWI0           10
#define INTC_TWI1           11
#define INTC_TWI2           12
#define INTC_TWI3           13
#define INTC_SPI0           16
#define INTC_SPI1           17
#define INTC_PWM            19
#define INTC_IR_TX          20
#define INTC_LEDC           21
#define INTC_CAN0           22
#define INTC_CAN1           23
#define INTC_USB0_DEV       30
#define INTC_USB0_EHCI      31
#define INTC_USB0_OHCI      32
#define INTC_USB1_EHCI      34
#define INTC_USB1_OHCI      35
#define INTC_SD0            41
#define INTC_SD1            42
#define INTC_SD2            43
#define INTC_GMAC           47
#define INTC_DMA_NS         51
#define INTC_DMA_S          52
#define INTC_CE_NS          53
#define INTC_CE_S           54
#define INTC_SPINLOCK       55
#define INTC_TIMER0         59
#define INTC_TIMER1         60
#define INTC_WATCHDOG       64      /* System watchdog (not DSP watchdog) */
#define INTC_IOMMU          65
#define INTC_VE             67
#define INTC_GPIOB_NS       70
#define INTC_GPIOB_S        71
#define INTC_GPIOC_NS       72
#define INTC_GPIOC_S        73
#define INTC_GPIOD_NS       74
#define INTC_GPIOD_S        75
#define INTC_GPIOE_NS       76
#define INTC_GPIOE_S        77
#define INTC_GPIOF_NS       78
#define INTC_GPIOF_S        79
#define INTC_GPIOG_NS       80
#define INTC_GPIOG_S        81
#define INTC_MSGBOX         86      /* CPUX_MSGBOX_DSP_W */

/* Maximum number of external interrupts */
#define MAX_IRQ_NUM         32      /* Direct DSP core interrupts */
#define MAX_INTC_NUM        128     /* DSP_INTC peripheral interrupts */

/* Interrupt handler callback type */
typedef void (*irq_handler_t)(uint32_t irq, void *arg);

/*============================================================================
 * Function Prototypes - Initialization
 *============================================================================*/

/**
 * @brief Initialize the HAL (clocks, interrupts, etc.)
 */
void hal_init(void);

/*============================================================================
 * Function Prototypes - Direct DSP Core Interrupts
 *============================================================================*/

/**
 * @brief Register handler for direct DSP core interrupt
 * @param irq_num IRQ number (IRQ_TIMER0, IRQ_HSTIMER0, etc.)
 * @param handler Handler function
 * @param arg User argument passed to handler
 */
void irq_register(uint32_t irq_num, irq_handler_t handler, void *arg);

/**
 * @brief Unregister direct DSP core interrupt handler
 */
void irq_unregister(uint32_t irq_num);

/**
 * @brief Enable direct DSP core interrupt
 */
void irq_enable_interrupt(uint32_t irq_num);

/**
 * @brief Disable direct DSP core interrupt
 */
void irq_disable_interrupt(uint32_t irq_num);

/**
 * @brief Enable global interrupts
 */
void irq_global_enable(void);

/**
 * @brief Disable global interrupts
 */
void irq_global_disable(void);

/*============================================================================
 * Function Prototypes - DSP_INTC Peripheral Interrupts
 *============================================================================*/

/**
 * @brief Register handler for DSP_INTC peripheral interrupt
 * @param intc_num INTC interrupt number (INTC_UART0, INTC_SPI0, etc.)
 * @param handler Handler function
 * @param arg User argument passed to handler
 * 
 * Note: These interrupts go through the DSP_INTC and trigger IRQ_INTC (17)
 */
void intc_register(uint32_t intc_num, irq_handler_t handler, void *arg);

/**
 * @brief Unregister DSP_INTC peripheral interrupt handler
 */
void intc_unregister(uint32_t intc_num);

/**
 * @brief Enable DSP_INTC peripheral interrupt
 * Also ensures IRQ_INTC is enabled at the core level.
 */
void intc_enable(uint32_t intc_num);

/**
 * @brief Disable DSP_INTC peripheral interrupt
 */
void intc_disable(uint32_t intc_num);

/*============================================================================
 * Function Prototypes - CCU (Clock Control)
 *============================================================================*/

/**
 * @brief Enable clock and deassert reset for UART
 * @param uart_id UART identifier (0-3)
 */
void ccu_uart_enable(uint8_t uart_id);

/**
 * @brief Disable clock for UART
 * @param uart_id UART identifier (0-3)
 */
void ccu_uart_disable(uint8_t uart_id);

/**
 * @brief Enable clock and deassert reset for I2C/TWI
 * @param twi_id TWI identifier (0-3)
 */
void ccu_twi_enable(uint8_t twi_id);

/**
 * @brief Disable clock for I2C/TWI
 * @param twi_id TWI identifier (0-3)
 */
void ccu_twi_disable(uint8_t twi_id);

/**
 * @brief Enable clock and deassert reset for SPI
 * @param spi_id SPI identifier (0-1)
 * @param clk_hz Desired SPI module clock frequency
 * @param pclk_hz Parent clock frequency (typically PLL_PERI or 24MHz)
 */
void ccu_spi_enable(uint8_t spi_id, uint32_t clk_hz, uint32_t pclk_hz);

/**
 * @brief Disable clock for SPI
 * @param spi_id SPI identifier (0-1)
 */
void ccu_spi_disable(uint8_t spi_id);

/**
 * @brief Enable clock and deassert reset for GPADC
 */
void ccu_gpadc_enable(void);

/**
 * @brief Disable clock for GPADC
 */
void ccu_gpadc_disable(void);

/**
 * @brief Enable clock and deassert reset for PWM
 */
void ccu_pwm_enable(void);

/**
 * @brief Disable clock for PWM
 */
void ccu_pwm_disable(void);

/**
 * @brief Enable clock and deassert reset for MSGBOX
 */
void ccu_msgbox_enable(void);

/**
 * @brief Disable clock for MSGBOX
 */
void ccu_msgbox_disable(void);

/**
 * @brief Enable clock and deassert reset for HSTIMER
 */
void ccu_hstimer_enable(void);

/**
 * @brief Disable clock for HSTIMER
 */
void ccu_hstimer_disable(void);

/*============================================================================
 * Function Prototypes - GPIO
 *============================================================================*/

/**
 * @brief Initialize GPIO (registers port-level IRQ handlers)
 */
void gpio_init(void);

/**
 * @brief Configure a GPIO pin's mode
 * @param pin GPIO pin descriptor
 * @param mode Desired mode (input, output, function, etc.)
 */
void gpio_set_mode(gpio_pin_t pin, gpio_mode_t mode);

/**
 * @brief Configure a GPIO pin's pull resistor
 * @param pin GPIO pin descriptor
 * @param pull Pull configuration
 */
void gpio_set_pull(gpio_pin_t pin, gpio_pull_t pull);

/**
 * @brief Set a GPIO output pin high or low
 * @param pin GPIO pin descriptor
 * @param value true = high, false = low
 */
void gpio_write(gpio_pin_t pin, bool value);

/**
 * @brief Read the current state of a GPIO pin
 * @param pin GPIO pin descriptor
 * @return true if high, false if low
 */
bool gpio_read(gpio_pin_t pin);

/**
 * @brief Toggle a GPIO output pin
 * @param pin GPIO pin descriptor
 */
void gpio_toggle(gpio_pin_t pin);

/**
 * @brief Configure GPIO external interrupt
 * @param pin GPIO pin descriptor
 * @param trigger Interrupt trigger type
 * @param handler Interrupt handler callback
 * @param arg User argument passed to handler
 */
void gpio_irq_configure(gpio_pin_t pin, gpio_irq_trigger_t trigger,
                        irq_handler_t handler, void *arg);

/**
 * @brief Enable GPIO external interrupt for a pin
 * @param pin GPIO pin descriptor
 */
void gpio_irq_enable(gpio_pin_t pin);

/**
 * @brief Disable GPIO external interrupt for a pin
 * @param pin GPIO pin descriptor
 */
void gpio_irq_disable(gpio_pin_t pin);

/**
 * @brief Clear GPIO interrupt pending flag
 * @param pin GPIO pin descriptor
 */
void gpio_irq_clear(gpio_pin_t pin);

/*============================================================================
 * Function Prototypes - UART
 *============================================================================*/

/**
 * @brief Initialize a UART peripheral
 * @param uart_id UART identifier (0-3)
 * @param config UART configuration
 * @param clk_freq Input clock frequency in Hz
 * @return 0 on success, negative error code on failure
 */
int uart_init(uart_id_t uart_id, const uart_config_t *config, uint32_t clk_freq);

/**
 * @brief Transmit a single byte (blocking)
 * @param uart_id UART identifier
 * @param data Byte to transmit
 */
void uart_putc(uart_id_t uart_id, uint8_t data);

/**
 * @brief Receive a single byte (blocking)
 * @param uart_id UART identifier
 * @return Received byte
 */
uint8_t uart_getc(uart_id_t uart_id);

/**
 * @brief Check if receive data is available
 * @param uart_id UART identifier
 * @return true if data available
 */
bool uart_rx_ready(uart_id_t uart_id);

/**
 * @brief Check if transmitter is ready for more data
 * @param uart_id UART identifier
 * @return true if ready to transmit
 */
bool uart_tx_ready(uart_id_t uart_id);

/**
 * @brief Transmit a string (blocking)
 * @param uart_id UART identifier
 * @param str Null-terminated string
 */
void uart_puts(uart_id_t uart_id, const char *str);

/**
 * @brief Transmit a buffer (blocking)
 * @param uart_id UART identifier
 * @param data Data buffer
 * @param len Number of bytes to transmit
 */
void uart_write(uart_id_t uart_id, const uint8_t *data, uint32_t len);

/**
 * @brief Receive into a buffer (blocking)
 * @param uart_id UART identifier
 * @param data Data buffer
 * @param len Number of bytes to receive
 */
void uart_read(uart_id_t uart_id, uint8_t *data, uint32_t len);

/**
 * @brief Register callback for received messages (interrupt-driven)
 * @param uart_id UART identifier
 * @param callback Function to call when message received
 * @param arg User argument passed to callback
 */
void uart_set_rx_callback(uart_id_t uart_id, irq_handler_t callback, void *arg);

/**
 * @brief Register callback for TX ready (FIFO has space)
 * @param uart_id UART identifier
 * @param callback Function to call when FIFO has space
 * @param arg User argument passed to callback
 * 
 * Note: TX interrupt fires when FIFO transitions from full to not-full.
 * Useful for implementing non-blocking bulk transfers.
 */
void uart_set_tx_callback(uart_id_t uart_id, irq_handler_t callback, void *arg);

/**
 * @brief Enable receive interrupt
 * @param uart_id UART identifier
 */
void uart_enable_rx_irq(uart_id_t uart_id);

/**
 * @brief Disable receive interrupt
 * @param uart_id UART identifier
 */
void uart_disable_rx_irq(uart_id_t uart_id);

/**
 * @brief Enable transmit interrupt
 * @param uart_id UART identifier
 * 
 * TX interrupt fires when the remote side reads from the FIFO,
 * making space available for new messages.
 */
void uart_enable_tx_irq(uart_id_t uart_id);

/**
 * @brief Disable transmit interrupt
 * @param uart_id UART identifier
 */
void uart_disable_tx_irq(uart_id_t uart_id);

/*============================================================================
 * Function Prototypes - GPADC
 *============================================================================*/

/**
 * @brief Initialize the GPADC
 * @param sample_rate Desired sample rate in Hz
 */
void gpadc_init(uint32_t sample_rate);

/**
 * @brief Enable an ADC channel
 * @param channel Channel to enable
 */
void gpadc_channel_enable(gpadc_channel_t channel);

/**
 * @brief Disable an ADC channel
 * @param channel Channel to disable
 */
void gpadc_channel_disable(gpadc_channel_t channel);

/**
 * @brief Read ADC value from a channel (blocking)
 * @param channel Channel to read
 * @return 12-bit ADC value (0-4095)
 */
uint16_t gpadc_read(gpadc_channel_t channel);

/**
 * @brief Read ADC value and convert to millivolts
 * @param channel Channel to read
 * @return Voltage in millivolts
 */
uint32_t gpadc_read_mv(gpadc_channel_t channel);

/**
 * @brief Start continuous conversion mode
 * @param channel Channel to convert
 */
void gpadc_start_continuous(gpadc_channel_t channel);

/**
 * @brief Stop continuous conversion mode
 */
void gpadc_stop_continuous(void);

/**
 * @brief Enable ADC data ready interrupt
 * @param channel Channel to monitor
 * @param handler Interrupt handler callback
 * @param arg User argument passed to handler
 */
void gpadc_irq_enable(gpadc_channel_t channel, irq_handler_t handler, void *arg);

/**
 * @brief Disable ADC data ready interrupt
 * @param channel Channel to disable interrupt for
 */
void gpadc_irq_disable(gpadc_channel_t channel);

/**
 * @brief Read ADC data from interrupt context (non-blocking)
 * @param channel Channel to read
 * @return 12-bit ADC value
 */
uint16_t gpadc_read_data(gpadc_channel_t channel);

/*============================================================================
 * Timer Definitions (32-bit down counter)
 *============================================================================*/

/* Timer Register offsets */
#define TMR_IRQ_EN          0x00    /* Timer IRQ Enable */
#define TMR_IRQ_STA         0x04    /* Timer IRQ Status */
#define TMR0_CTRL           0x10    /* Timer 0 Control */
#define TMR0_INTV           0x14    /* Timer 0 Interval Value */
#define TMR0_CUR            0x18    /* Timer 0 Current Value */
#define TMR1_CTRL           0x20    /* Timer 1 Control */
#define TMR1_INTV           0x24    /* Timer 1 Interval Value */
#define TMR1_CUR            0x28    /* Timer 1 Current Value */

/* Timer Control bits */
#define TMR_CTRL_EN             BIT(0)      /* Timer Enable */
#define TMR_CTRL_RELOAD         BIT(1)      /* Reload mode */
#define TMR_CTRL_CLK_SRC_MASK   (0x3 << 2)  /* Clock source */
#define TMR_CTRL_CLK_LOSC       (0 << 2)    /* 32.768 kHz */
#define TMR_CTRL_CLK_OSC24M     (1 << 2)    /* 24 MHz */
#define TMR_CTRL_PRESCALE_MASK  (0x7 << 4)  /* Prescaler */
#define TMR_CTRL_PRESCALE(n)    ((n) << 4)  /* Divide by 2^n */
#define TMR_CTRL_MODE_SINGLE    BIT(7)      /* Single shot */

/* Timer IRQ bits */
#define TMR_IRQ_TMR0            BIT(0)
#define TMR_IRQ_TMR1            BIT(1)

typedef enum {
    TIMER_0 = 0,
    TIMER_1 = 1,
} timer_id_t;

typedef enum {
    TIMER_CLK_LOSC = 0,     /* 32.768 kHz */
    TIMER_CLK_OSC24M = 1,   /* 24 MHz */
} timer_clk_src_t;

/*============================================================================
 * High-Speed Timer Definitions
 *============================================================================*/

/* HSTimer Register offsets */
#define HSTMR_IRQ_EN        0x00    /* HSTimer IRQ Enable */
#define HSTMR_IRQ_STA       0x04    /* HSTimer IRQ Status */
#define HSTMR0_CTRL         0x20    /* HSTimer 0 Control */
#define HSTMR0_INTV_LO      0x24    /* HSTimer 0 Interval Low */
#define HSTMR0_INTV_HI      0x28    /* HSTimer 0 Interval High */
#define HSTMR0_CUR_LO       0x2C    /* HSTimer 0 Current Low */
#define HSTMR0_CUR_HI       0x30    /* HSTimer 0 Current High */
#define HSTMR1_CTRL         0x40    /* HSTimer 1 Control */
#define HSTMR1_INTV_LO      0x44    /* HSTimer 1 Interval Low */
#define HSTMR1_INTV_HI      0x48    /* HSTimer 1 Interval High */
#define HSTMR1_CUR_LO       0x4C    /* HSTimer 1 Current Low */
#define HSTMR1_CUR_HI       0x50    /* HSTimer 1 Current High */

/* HSTimer Control bits */
#define HSTMR_CTRL_EN           BIT(0)      /* Enable */
#define HSTMR_CTRL_RELOAD       BIT(1)      /* Reload mode */
#define HSTMR_CTRL_CLK_PRE_MASK (0x7 << 4)  /* Prescaler */
#define HSTMR_CTRL_CLK_PRE(n)   ((n) << 4)  /* Divide by 2^n */
#define HSTMR_CTRL_MODE_SINGLE  0
#define HSTMR_CTRL_MODE_CONT    BIT(7)

/* HSTimer IRQ bits */
#define HSTMR_IRQ_TMR0          BIT(0)
#define HSTMR_IRQ_TMR1          BIT(1)

typedef enum {
    HSTIMER_0 = 0,
    HSTIMER_1 = 1,
} hstimer_id_t;

/*============================================================================
 * Watchdog Definitions
 *============================================================================*/

/* Watchdog Register offsets (from WDOG_BASE) */
#define WDOG_IRQ_EN         0x00    /* Watchdog IRQ Enable */
#define WDOG_IRQ_STA        0x04    /* Watchdog IRQ Status */
#define WDOG_CTRL           0x10    /* Watchdog Control */
#define WDOG_CFG            0x14    /* Watchdog Configuration */
#define WDOG_MODE           0x18    /* Watchdog Mode */

/* Watchdog Control bits */
#define WDOG_CTRL_KEY           (0x0A57 << 1)   /* Access key */
#define WDOG_CTRL_RESTART       BIT(0)          /* Restart/kick watchdog */

/* Watchdog Configuration bits */
#define WDOG_CFG_CONFIG_MASK    0x03
#define WDOG_CFG_SYS_RESET      1   /* Generate system reset */
#define WDOG_CFG_IRQ_ONLY       2   /* Generate IRQ only */

/* Watchdog Mode bits */
#define WDOG_MODE_EN            BIT(0)
#define WDOG_MODE_INTV_MASK     (0xF << 4)
#define WDOG_MODE_INTV(n)       ((n) << 4)  /* Interval: (n+1) * 0.5s */

/*============================================================================
 * I2C/TWI Definitions
 *============================================================================*/

/* TWI Register offsets */
#define TWI_ADDR            0x00    /* Slave Address */
#define TWI_XADDR           0x04    /* Extended Slave Address */
#define TWI_DATA            0x08    /* Data */
#define TWI_CNTR            0x0C    /* Control */
#define TWI_STAT            0x10    /* Status */
#define TWI_CCR             0x14    /* Clock Control */
#define TWI_SRST            0x18    /* Software Reset */
#define TWI_EFR             0x1C    /* Enhanced Feature */
#define TWI_LCR             0x20    /* Line Control */

/* TWI Control bits */
#define TWI_CNTR_ACK            BIT(2)      /* Assert ACK */
#define TWI_CNTR_INT_FLAG       BIT(3)      /* Interrupt flag */
#define TWI_CNTR_STP            BIT(4)      /* Generate STOP */
#define TWI_CNTR_STA            BIT(5)      /* Generate START */
#define TWI_CNTR_BUS_EN         BIT(6)      /* Bus enable */
#define TWI_CNTR_INT_EN         BIT(7)      /* Interrupt enable */

/* TWI Status codes */
#define TWI_STAT_BUS_ERROR      0x00
#define TWI_STAT_START          0x08
#define TWI_STAT_REP_START      0x10
#define TWI_STAT_ADDR_WR_ACK    0x18
#define TWI_STAT_ADDR_WR_NACK   0x20
#define TWI_STAT_DATA_WR_ACK    0x28
#define TWI_STAT_DATA_WR_NACK   0x30
#define TWI_STAT_ARB_LOST       0x38
#define TWI_STAT_ADDR_RD_ACK    0x40
#define TWI_STAT_ADDR_RD_NACK   0x48
#define TWI_STAT_DATA_RD_ACK    0x50
#define TWI_STAT_DATA_RD_NACK   0x58
#define TWI_STAT_IDLE           0xF8

/* TWI Software Reset */
#define TWI_SRST_RESET          BIT(0)

typedef enum {
    I2C_0 = 0,
    I2C_1 = 1,
    I2C_2 = 2,
    I2C_3 = 3,
} i2c_id_t;

typedef struct {
    uint32_t speed_hz;      /* Bus speed (100000 or 400000) */
} i2c_config_t;

/*============================================================================
 * SPI Definitions
 *============================================================================*/

/* SPI Register offsets */
#define SPI_GCR             0x04    /* Global Control */
#define SPI_TCR             0x08    /* Transfer Control */
#define SPI_IER             0x10    /* Interrupt Enable */
#define SPI_ISR             0x14    /* Interrupt Status */
#define SPI_FCR             0x18    /* FIFO Control */
#define SPI_FSR             0x1C    /* FIFO Status */
#define SPI_WCR             0x20    /* Wait Clock Counter */
#define SPI_CCR             0x24    /* Clock Control */
#define SPI_MBC             0x30    /* Master Burst Count */
#define SPI_MTC             0x34    /* Master Transmit Count */
#define SPI_BCC             0x38    /* Master Burst Control */
#define SPI_TXD             0x200   /* TX Data */
#define SPI_RXD             0x300   /* RX Data */

/* SPI Global Control bits */
#define SPI_GCR_EN              BIT(0)      /* SPI Enable */
#define SPI_GCR_MODE_MASTER     BIT(1)      /* Master mode */
#define SPI_GCR_TP_EN           BIT(7)      /* Transmit pause enable */
#define SPI_GCR_SRST            BIT(31)     /* Soft reset */

/* SPI Transfer Control bits */
#define SPI_TCR_CPHA            BIT(0)      /* Clock phase */
#define SPI_TCR_CPOL            BIT(1)      /* Clock polarity */
#define SPI_TCR_SPOL            BIT(2)      /* SS polarity */
#define SPI_TCR_SSCTL           BIT(3)      /* SS control */
#define SPI_TCR_SS_MASK         (0x3 << 4)  /* Chip select */
#define SPI_TCR_SS(n)           ((n) << 4)
#define SPI_TCR_SS_OWNER        BIT(6)      /* SS owner (1=SW) */
#define SPI_TCR_SS_LEVEL        BIT(7)      /* SS level when SW owned */
#define SPI_TCR_DHB             BIT(8)      /* Discard hash burst */
#define SPI_TCR_FBS             BIT(12)     /* First bit select (MSB/LSB) */
#define SPI_TCR_XCH             BIT(31)     /* Exchange burst */

/* SPI Interrupt Enable/Status bits */
#define SPI_INT_RX_RDY          BIT(0)      /* RX FIFO ready */
#define SPI_INT_RX_EMP          BIT(1)      /* RX FIFO empty */
#define SPI_INT_RX_FULL         BIT(2)      /* RX FIFO full */
#define SPI_INT_TX_RDY          BIT(4)      /* TX FIFO ready */
#define SPI_INT_TX_EMP          BIT(5)      /* TX FIFO empty */
#define SPI_INT_TX_FULL         BIT(6)      /* TX FIFO full */
#define SPI_INT_RX_OVF          BIT(8)      /* RX FIFO overflow */
#define SPI_INT_RX_UDF          BIT(9)      /* RX FIFO underflow */
#define SPI_INT_TX_OVF          BIT(10)     /* TX FIFO overflow */
#define SPI_INT_TX_UDF          BIT(11)     /* TX FIFO underflow */
#define SPI_INT_TC              BIT(12)     /* Transfer complete */
#define SPI_INT_SSI             BIT(13)     /* SS invalid */

/* SPI FIFO Control bits */
#define SPI_FCR_RX_RST          BIT(15)     /* Reset RX FIFO */
#define SPI_FCR_TX_RST          BIT(31)     /* Reset TX FIFO */
#define SPI_FCR_RX_TRIG_MASK    (0xFF << 0)
#define SPI_FCR_TX_TRIG_MASK    (0xFF << 16)

/* SPI FIFO Status */
#define SPI_FSR_RX_CNT_MASK     (0xFF << 0)
#define SPI_FSR_TX_CNT_MASK     (0xFF << 16)

typedef enum {
    SPI_0 = 0,
    SPI_1 = 1,
} spi_id_t;

typedef enum {
    SPI_MODE_0 = 0,     /* CPOL=0, CPHA=0 */
    SPI_MODE_1 = 1,     /* CPOL=0, CPHA=1 */
    SPI_MODE_2 = 2,     /* CPOL=1, CPHA=0 */
    SPI_MODE_3 = 3,     /* CPOL=1, CPHA=1 */
} spi_mode_t;

typedef struct {
    uint32_t    speed_hz;       /* Clock speed */
    spi_mode_t  mode;           /* SPI mode (0-3) */
    uint8_t     bits_per_word;  /* 8 typically */
    bool        lsb_first;      /* LSB first if true */
} spi_config_t;

/*============================================================================
 * PWM Definitions
 *============================================================================*/

/* PWM Register offsets */
#define PWM_PIER            0x00    /* PWM IRQ Enable */
#define PWM_PISR            0x04    /* PWM IRQ Status */
#define PWM_CIER            0x10    /* Capture IRQ Enable */
#define PWM_CISR            0x14    /* Capture IRQ Status */
#define PWM_PCCR(n)         (0x20 + (n) * 0x04)     /* PWM Clock Config */
#define PWM_PCGR            0x40    /* PWM Clock Gating */
#define PWM_PDZCR(n)        (0x60 + (n) * 0x04)     /* Dead Zone Control */
#define PWM_PER             0x80    /* PWM Enable */
#define PWM_PGR(n)          (0x90 + (n) * 0x04)     /* PWM Group */
#define PWM_CER             0xC0    /* Capture Enable */
#define PWM_PCR(n)          (0x100 + (n) * 0x20)    /* PWM Control */
#define PWM_PPR(n)          (0x104 + (n) * 0x20)    /* PWM Period */
#define PWM_PCNTR(n)        (0x108 + (n) * 0x20)    /* PWM Counter */
#define PWM_PPCNTR(n)       (0x10C + (n) * 0x20)    /* PWM Pulse Counter */
#define PWM_CCR(n)          (0x110 + (n) * 0x20)    /* Capture Control */
#define PWM_CRLR(n)         (0x114 + (n) * 0x20)    /* Capture Rise Lock */
#define PWM_CFLR(n)         (0x118 + (n) * 0x20)    /* Capture Fall Lock */

/* PWM Control bits */
#define PWM_PCR_PRESCAL_MASK    (0xFF << 0)     /* Prescaler (divide by n+1) */
#define PWM_PCR_ACT_STA         BIT(8)          /* Active state (0=low, 1=high) */
#define PWM_PCR_MODE            BIT(9)          /* Mode (0=cycle, 1=pulse) */
#define PWM_PCR_PUL_START       BIT(10)         /* Pulse start */
#define PWM_PCR_PERIOD_RDY      BIT(11)         /* Period ready */

/* PWM Period register */
#define PWM_PPR_ACT_CYCLES_MASK     (0xFFFF << 0)   /* Active cycles */
#define PWM_PPR_ENTIRE_CYCLES_MASK  (0xFFFF << 16)  /* Entire period cycles */

/* PWM Enable bits */
#define PWM_PER_EN(n)           BIT(n)

/* PWM Clock Config */
#define PWM_CLK_SRC_MASK        (0x3 << 7)
#define PWM_CLK_SRC_HOSC        (0 << 7)    /* 24 MHz */
#define PWM_CLK_SRC_APB1        (1 << 7)    /* APB1 clock */
#define PWM_CLK_DIV_MASK        0x0F
#define PWM_CLK_DIV(n)          (n)         /* Divide by 2^n */

#define PWM_MAX_CHANNELS        8

typedef enum {
    PWM_CH0 = 0,
    PWM_CH1 = 1,
    PWM_CH2 = 2,
    PWM_CH3 = 3,
    PWM_CH4 = 4,
    PWM_CH5 = 5,
    PWM_CH6 = 6,
    PWM_CH7 = 7,
} pwm_channel_t;

typedef struct {
    uint32_t    frequency_hz;   /* PWM frequency */
    uint16_t    duty_percent;   /* Duty cycle 0-100 (or use duty_raw) */
    bool        active_high;    /* Active high output */
} pwm_config_t;

/*============================================================================
 * Function Prototypes - Timer
 *============================================================================*/

/**
 * @brief Initialize a timer
 * @param timer_id Timer identifier (0 or 1)
 * @param clk_src Clock source
 * @param prescaler Prescaler value (0-7, divides by 2^n)
 */
void timer_init(timer_id_t timer_id, timer_clk_src_t clk_src, uint8_t prescaler);

/**
 * @brief Start timer in one-shot mode
 * @param timer_id Timer identifier
 * @param interval_us Interval in microseconds
 * @param handler Callback when timer expires (NULL for polling)
 * @param arg User argument for callback
 */
void timer_start_oneshot(timer_id_t timer_id, uint32_t interval_us,
                         irq_handler_t handler, void *arg);

/**
 * @brief Start timer in continuous/periodic mode
 * @param timer_id Timer identifier
 * @param interval_us Interval in microseconds
 * @param handler Callback on each period (NULL for polling)
 * @param arg User argument for callback
 */
void timer_start_periodic(timer_id_t timer_id, uint32_t interval_us,
                          irq_handler_t handler, void *arg);

/**
 * @brief Stop a timer
 * @param timer_id Timer identifier
 */
void timer_stop(timer_id_t timer_id);

/**
 * @brief Get current timer counter value
 * @param timer_id Timer identifier
 * @return Current counter value
 */
uint32_t timer_get_counter(timer_id_t timer_id);

/**
 * @brief Check if timer interrupt is pending
 * @param timer_id Timer identifier
 * @return true if interrupt pending
 */
bool timer_irq_pending(timer_id_t timer_id);

/**
 * @brief Clear timer interrupt
 * @param timer_id Timer identifier
 */
void timer_irq_clear(timer_id_t timer_id);

/*============================================================================
 * Function Prototypes - High-Speed Timer
 *============================================================================*/

/**
 * @brief Initialize high-speed timer
 * @param hstimer_id HSTimer identifier (0 or 1)
 * @param prescaler Prescaler value (0-7)
 */
void hstimer_init(hstimer_id_t hstimer_id, uint8_t prescaler);

/**
 * @brief Start high-speed timer (56-bit counter)
 * @param hstimer_id HSTimer identifier
 * @param interval_us Interval in microseconds (use _ns variant for nanoseconds)
 * @param handler Callback (NULL for polling)
 * @param arg User argument
 */
void hstimer_start_oneshot(hstimer_id_t hstimer_id, uint32_t interval_us,
                           irq_handler_t handler, void *arg);

/**
 * @brief Start high-speed timer in periodic mode
 */
void hstimer_start_periodic(hstimer_id_t hstimer_id, uint32_t interval_us,
                            irq_handler_t handler, void *arg);

/**
 * @brief Stop high-speed timer
 */
void hstimer_stop(hstimer_id_t hstimer_id);

/**
 * @brief Get current 56-bit counter value
 * @param hstimer_id HSTimer identifier
 * @param lo Output: lower 32 bits
 * @param hi Output: upper 24 bits
 */
void hstimer_get_counter(hstimer_id_t hstimer_id, uint32_t *lo, uint32_t *hi);

/*============================================================================
 * Function Prototypes - Watchdog
 *============================================================================*/

/**
 * @brief Initialize watchdog timer
 * @param timeout_ms Timeout in milliseconds (rounded to nearest 0.5s interval)
 * @param reset_on_timeout true to reset system, false for IRQ only
 */
void watchdog_init(uint32_t timeout_ms, bool reset_on_timeout);

/**
 * @brief Start the watchdog
 */
void watchdog_start(void);

/**
 * @brief Stop the watchdog
 */
void watchdog_stop(void);

/**
 * @brief Kick/feed the watchdog (reset countdown)
 */
void watchdog_kick(void);

/**
 * @brief Set watchdog interrupt handler (for IRQ-only mode)
 */
void watchdog_set_handler(irq_handler_t handler, void *arg);

/*============================================================================
 * Function Prototypes - I2C
 *============================================================================*/

/**
 * @brief Initialize I2C controller
 * @param i2c_id I2C identifier (0-3)
 * @param config I2C configuration
 * @param pclk_hz Peripheral clock frequency
 * @return 0 on success
 */
int i2c_init(i2c_id_t i2c_id, const i2c_config_t *config, uint32_t pclk_hz);

/**
 * @brief Write data to I2C device
 * @param i2c_id I2C identifier
 * @param addr 7-bit slave address
 * @param data Data to write
 * @param len Data length
 * @return 0 on success, negative on error
 */
int i2c_write_addr(i2c_id_t i2c_id, uint8_t addr, const uint8_t *data, uint32_t len);

/**
 * @brief Read data from I2C device
 * @param i2c_id I2C identifier
 * @param addr 7-bit slave address
 * @param data Buffer to read into
 * @param len Number of bytes to read
 * @return 0 on success, negative on error
 */
int i2c_read_addr(i2c_id_t i2c_id, uint8_t addr, uint8_t *data, uint32_t len);

/**
 * @brief Write then read (combined transaction)
 * @param i2c_id I2C identifier
 * @param addr 7-bit slave address
 * @param tx_data Data to write
 * @param tx_len Write length
 * @param rx_data Buffer to read into
 * @param rx_len Read length
 * @return 0 on success, negative on error
 */
int i2c_write_read(i2c_id_t i2c_id, uint8_t addr,
                   const uint8_t *tx_data, uint32_t tx_len,
                   uint8_t *rx_data, uint32_t rx_len);

/**
 * @brief Write to a register (common pattern)
 * @param i2c_id I2C identifier
 * @param addr 7-bit slave address
 * @param reg Register address
 * @param data Data to write
 * @param len Data length
 * @return 0 on success
 */
int i2c_write_reg(i2c_id_t i2c_id, uint8_t addr, uint8_t reg,
                  const uint8_t *data, uint32_t len);

/**
 * @brief Read from a register (common pattern)
 * @param i2c_id I2C identifier
 * @param addr 7-bit slave address
 * @param reg Register address
 * @param data Buffer to read into
 * @param len Number of bytes to read
 * @return 0 on success
 */
int i2c_read_reg(i2c_id_t i2c_id, uint8_t addr, uint8_t reg,
                 uint8_t *data, uint32_t len);

/*============================================================================
 * Function Prototypes - SPI
 *============================================================================*/

/**
 * @brief Initialize SPI controller
 * @param spi_id SPI identifier (0 or 1)
 * @param config SPI configuration
 * @param pclk_hz Peripheral clock frequency
 * @return 0 on success
 */
int spi_init(spi_id_t spi_id, const spi_config_t *config, uint32_t pclk_hz);

/**
 * @brief Set chip select level (for manual CS control)
 * @param spi_id SPI identifier
 * @param cs_num Chip select number (0-3)
 * @param active true = assert CS, false = deassert
 */
void spi_cs_control(spi_id_t spi_id, uint8_t cs_num, bool active);

/**
 * @brief Transfer data (full duplex)
 * @param spi_id SPI identifier
 * @param tx_data Data to transmit (NULL for receive only)
 * @param rx_data Buffer for received data (NULL for transmit only)
 * @param len Number of bytes
 * @return 0 on success
 */
int spi_write_read(spi_id_t spi_id, const uint8_t *tx_data, uint8_t *rx_data, uint32_t len);

/**
 * @brief Write data (transmit only)
 * @param spi_id SPI identifier
 * @param data Data to transmit
 * @param len Number of bytes
 * @return 0 on success
 */
int spi_write(spi_id_t spi_id, const uint8_t *data, uint32_t len);

/**
 * @brief Read data (receive only, sends zeros)
 * @param spi_id SPI identifier
 * @param data Buffer for received data
 * @param len Number of bytes
 * @return 0 on success
 */
int spi_read(spi_id_t spi_id, uint8_t *data, uint32_t len);

/**
 * @brief Transfer a single byte
 * @param spi_id SPI identifier
 * @param tx_byte Byte to send
 * @return Received byte
 */
uint8_t spi_write_read_byte(spi_id_t spi_id, uint8_t tx_byte);

/*============================================================================
 * Function Prototypes - PWM
 *============================================================================*/

/**
 * @brief Initialize PWM channel
 * @param channel PWM channel (0-7)
 * @param config PWM configuration
 * @param clk_hz Source clock frequency (typically 24 MHz)
 * @return 0 on success
 */
int pwm_init(pwm_channel_t channel, const pwm_config_t *config, uint32_t clk_hz);

/**
 * @brief Enable PWM output
 * @param channel PWM channel
 */
void pwm_enable(pwm_channel_t channel);

/**
 * @brief Disable PWM output
 * @param channel PWM channel
 */
void pwm_disable(pwm_channel_t channel);

/**
 * @brief Set PWM duty cycle
 * @param channel PWM channel
 * @param duty_percent Duty cycle 0-100
 */
void pwm_set_duty(pwm_channel_t channel, uint16_t duty_percent);

/**
 * @brief Set PWM duty cycle (raw value)
 * @param channel PWM channel
 * @param active_cycles Number of active cycles
 * @param total_cycles Total period cycles
 */
void pwm_set_duty_raw(pwm_channel_t channel, uint16_t active_cycles, uint16_t total_cycles);

/**
 * @brief Set PWM frequency
 * @param channel PWM channel
 * @param frequency_hz New frequency
 * @param clk_hz Source clock frequency
 */
void pwm_set_frequency(pwm_channel_t channel, uint32_t frequency_hz, uint32_t clk_hz);

/**
 * @brief Get current PWM counter value
 * @param channel PWM channel
 * @return Counter value
 */
uint16_t pwm_get_counter(pwm_channel_t channel);

/*============================================================================
 * MSGBOX (Mailbox) Definitions - ARM <-> DSP Communication
 *============================================================================*/

/*
 * The MSGBOX provides hardware FIFOs for inter-processor communication.
 * 
 * Architecture:
 * - ARM CPUX has its MSGBOX at 0x03003000
 * - DSP has its MSGBOX at 0x01701000
 * - Each MSGBOX accesses the SAME shared FIFOs from different perspectives
 * 
 * Register addressing:
 * - N = Remote CPU index (for DSP talking to ARM, N=0)
 * - P = Channel number (0-3 channels available)
 * 
 * Communication model:
 * - At the SAME channel P, user1 (transmitter) writes, user0 (receiver) reads
 * - From DSP's perspective: DSP is user1 (TX), ARM is user0 (RX)
 * - So DSP uses WR registers to send, RD registers to receive from ARM
 * 
 * FIFO: 8 messages deep, each message is 32 bits
 */

/* MSGBOX Register offsets
 * N = remote CPU index (0 for ARM<->DSP)
 * P = channel number (0-3)
 */
#define MSGBOX_RD_IRQ_EN(n)         (0x0020 + (n) * 0x100)
#define MSGBOX_RD_IRQ_STATUS(n)     (0x0024 + (n) * 0x100)
#define MSGBOX_WR_IRQ_EN(n)         (0x0030 + (n) * 0x100)
#define MSGBOX_WR_IRQ_STATUS(n)     (0x0034 + (n) * 0x100)
#define MSGBOX_DEBUG_REG(n)         (0x0040 + (n) * 0x100)
#define MSGBOX_FIFO_STATUS(n, p)    (0x0050 + (n) * 0x100 + (p) * 0x04)
#define MSGBOX_MSG_STATUS(n, p)     (0x0060 + (n) * 0x100 + (p) * 0x04)
#define MSGBOX_MSG(n, p)            (0x0070 + (n) * 0x100 + (p) * 0x04)
#define MSGBOX_WR_INT_THRESHOLD(n, p) (0x0080 + (n) * 0x100 + (p) * 0x04)

/* MSGBOX RD_IRQ_EN / RD_IRQ_STATUS bits (for receiving) */
#define MSGBOX_RD_IRQ_CH0           BIT(0)  /* Channel 0 RX interrupt */
#define MSGBOX_RD_IRQ_CH1           BIT(2)  /* Channel 1 RX interrupt */
#define MSGBOX_RD_IRQ_CH2           BIT(4)  /* Channel 2 RX interrupt */
#define MSGBOX_RD_IRQ_CH3           BIT(6)  /* Channel 3 RX interrupt */
#define MSGBOX_RD_IRQ_CH(p)         BIT((p) * 2)

/* MSGBOX WR_IRQ_EN / WR_IRQ_STATUS bits (for transmitting) */
#define MSGBOX_WR_IRQ_CH0           BIT(1)  /* Channel 0 TX interrupt */
#define MSGBOX_WR_IRQ_CH1           BIT(3)  /* Channel 1 TX interrupt */
#define MSGBOX_WR_IRQ_CH2           BIT(5)  /* Channel 2 TX interrupt */
#define MSGBOX_WR_IRQ_CH3           BIT(7)  /* Channel 3 TX interrupt */
#define MSGBOX_WR_IRQ_CH(p)         BIT((p) * 2 + 1)

/* MSGBOX FIFO Status bits (MSGBOX_FIFO_STATUS register) */
#define MSGBOX_FIFO_FULL_FLAG       BIT(0)  /* 1 = FIFO is full */

/* MSGBOX MSG Status bits (MSGBOX_MSG_STATUS register) */
#define MSGBOX_MSG_NUM_MASK         0x0F    /* Number of messages in FIFO (0-8) */

/* Remote CPU index for DSP <-> ARM communication */
#define MSGBOX_REMOTE_ARM           0       /* ARM is remote CPU 0 for DSP */

/* Channel numbers */
#define MSGBOX_CHANNEL_0            0
#define MSGBOX_CHANNEL_1            1
#define MSGBOX_CHANNEL_2            2
#define MSGBOX_CHANNEL_3            3
#define MSGBOX_NUM_CHANNELS         4

/* FIFO depth */
#define MSGBOX_FIFO_DEPTH           8       /* Each FIFO holds 8 messages */

/* Callback for received messages */
typedef void (*msgbox_rx_callback_t)(uint8_t channel, uint32_t message, void *arg);

/* Callback for TX ready (FIFO not full) */
typedef void (*msgbox_tx_callback_t)(uint8_t channel, void *arg);

/*============================================================================
 * Function Prototypes - MSGBOX
 *============================================================================*/

/**
 * @brief Initialize the MSGBOX hardware
 */
void msgbox_init(void);

/**
 * @brief Send a message to the ARM core
 * @param channel Channel to use (0-3)
 * @param message 32-bit message to send
 * @return 0 on success, -1 if FIFO full
 */
int msgbox_send(uint8_t channel, uint32_t message);

/**
 * @brief Send a message, blocking until space available
 * @param channel Channel to use
 * @param message 32-bit message to send
 */
void msgbox_send_blocking(uint8_t channel, uint32_t message);

/**
 * @brief Receive a message from ARM (non-blocking)
 * @param channel Channel to read from (0-3)
 * @param message Pointer to store received message
 * @return 0 on success, -1 if FIFO empty
 */
int msgbox_recv(uint8_t channel, uint32_t *message);

/**
 * @brief Receive a message, blocking until available
 * @param channel Channel to read from
 * @return Received message
 */
uint32_t msgbox_recv_blocking(uint8_t channel);

/**
 * @brief Check number of pending messages in receive FIFO
 * @param channel Channel to check
 * @return Number of messages in FIFO (0-8)
 */
uint8_t msgbox_rx_pending(uint8_t channel);

/**
 * @brief Check if channel can accept more messages
 * @param channel Channel to check
 * @return true if FIFO is not full
 */
bool msgbox_tx_ready(uint8_t channel);

/**
 * @brief Register callback for received messages (interrupt-driven)
 * @param channel Channel to monitor (0-3)
 * @param callback Function to call when message received
 * @param arg User argument passed to callback
 */
void msgbox_set_rx_callback(uint8_t channel, msgbox_rx_callback_t callback, void *arg);

/**
 * @brief Register callback for TX ready (FIFO not full)
 * @param channel Channel to monitor
 * @param callback Function to call when FIFO has space
 * @param arg User argument passed to callback
 */
void msgbox_set_tx_callback(uint8_t channel, msgbox_tx_callback_t callback, void *arg);

/**
 * @brief Enable receive interrupt for a channel
 * @param channel Channel to enable (0-3)
 */
void msgbox_enable_rx_irq(uint8_t channel);

/**
 * @brief Disable receive interrupt for a channel
 * @param channel Channel to disable
 */
void msgbox_disable_rx_irq(uint8_t channel);

/**
 * @brief Enable transmit interrupt for a channel
 * @param channel Channel to enable
 */
void msgbox_enable_tx_irq(uint8_t channel);

/**
 * @brief Disable transmit interrupt for a channel
 * @param channel Channel to disable
 */
void msgbox_disable_tx_irq(uint8_t channel);

/**
 * @brief Flush all pending messages from a receive channel
 * @param channel Channel to flush
 */
void msgbox_flush_rx(uint8_t channel);

/*============================================================================
 * Debug/Diagnostic Functions
 *============================================================================*/

/**
 * @brief Get current INTENABLE register value
 */
uint32_t hal_get_intenable(void);

/**
 * @brief Get current INTERRUPT register (pending interrupts)
 */
uint32_t hal_get_interrupt(void);

/**
 * @brief Get current PS register value
 */
uint32_t hal_get_ps(void);

/**
 * @brief Trigger a software interrupt for testing
 */
void hal_trigger_soft_interrupt(uint32_t irq_num);

/*============================================================================
 * Utility Macros
 *============================================================================*/

#define GPIO_PIN(po, pi) ((gpio_pin_t){.port = (po), .pin = (pi)})

/* Common delay function (implement based on your timing source) */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* R528_HIFI4_HAL_H */
