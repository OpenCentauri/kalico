# Additional linux build rules
CROSS_PREFIX=arm-none-eabi-

CFLAGS-$(CONFIG_MACH_ALLWINNER_R528) += -mcpu=cortex-a7

dirs-y += src/aw_r528 src/generic

src-y += aw_r528/main.c aw_r528/timer.c aw_r528/console.c aw_r528/watchdog.c
src-y += aw_r528/pca9685.c aw_r528/spidev.c aw_r528/analog.c aw_r528/hard_pwm.c
src-y += aw_r528/i2c.c aw_r528/gpio.c generic/crc16_ccitt.c generic/alloc.c
src-y += aw_r528/sensor_ds18b20.c

CFLAGS_klipper.elf += -lutil -lrt -lpthread

flash: $(OUT)klipper.elf
	@echo "  Flashing"
	$(Q)sudo ./scripts/flash-linux.sh $(OUT)
