# Additional HIFI4 build rules

# Setup the toolchain
CROSS_PREFIX=xtensa-nxp_rt700_hifi4_zephyr-elf-

dirs-y += src/generic
dirs-y += src/hifi4
dirs-y += lib/hifi4

CFLAGS += -O0 -mlongcalls -mtext-section-literals -mabi=call0

# rtos-hal
CFLAGS += -I lib/hifi4

CFLAGS_klipper.elf += -Wl,--script src/hifi4/hifi4.ld
CFLAGS_klipper.elf += -nostdlib
CFLAGS_klipper.elf += -Wl,--gc-sections
CFLAGS_klipper.elf += -Wl,--abi-call0
CFLAGS_klipper.elf += -Wl,-Map,$(OUT)/dsp.map

ASFLAGS += -Wa,--abi-call0
ASFLAGS += -Ilib/hifi4

# Add source files
src-y += hifi4/main.c
src-y += hifi4/util.c
src-y += hifi4/gpio.c
src-y += hifi4/irq.c
src-y += hifi4/timer.c
src-$(CONFIG_SERIAL) += hifi4/serial.c
src-$(CONFIG_HIFI4_MSGBOX) += hifi4/msgbox.c
src-$(CONFIG_HIFI4_MSGBOX) += hifi4/usb_bulk.c
src-$(CONFIG_HIFI4_MSGBOX) += hifi4/usb_cdc.c
src-$(CONFIG_HIFI4_MSGBOX) += hifi4/sharespace.c

# rtos-hal
src-y += ../lib/hifi4/hal_core.c
src-y += ../lib/hifi4/gpio.c
src-y += ../lib/hifi4/gpadc.c
src-y += ../lib/hifi4/timer.c
src-y += ../lib/hifi4/hstimer.c
src-y += ../lib/hifi4/watchdog.c
src-y += ../lib/hifi4/i2c.c
src-y += ../lib/hifi4/spi.c
src-y += ../lib/hifi4/pwm.c
src-y += ../lib/hifi4/ccu.c
src-y += ../lib/hifi4/uart.c
src-$(CONFIG_HIFI4_MSGBOX) += ../lib/hifi4/msgbox.c

# generic
src-y += generic/timer_irq.c
src-y += generic/crc16_ccitt.c
src-y += generic/alloc.c
src-$(CONFIG_SERIAL) += generic/serial_irq.c

OBJS_klipper.elf += $(OUT)lib/hifi4/startup.o
OBJS_klipper.elf += $(OUT)lib/hifi4/vectors.o
OBJS_klipper.elf += $(OUT)lib/hifi4/oemhead.o
OBJS_klipper.elf += $(OUT)lib/hifi4/setjmp.o

$(OUT)%.o: %.S
	@echo "  Compiling $@"
	$(Q)$(CC) $(ASFLAGS) -c $(PWD)/$< -o $@
