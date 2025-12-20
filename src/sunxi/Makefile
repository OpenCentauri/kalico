# Additional sunxi build rules

CROSS_PREFIX ?= ../toolchain/xtensa-hifi4-gcc/bin/xtensa-hifi4-elf-

dirs-y += src/sunxi src/generic

APP_NAME=klipper-dsp
APP := $(APP_NAME).elf

DFLAGS := -DXT_BOARD  -DXT_TIMER_INDEX=0 -DXT_USE_SWPRI -DSTANDALONE=1
DFLAGS += -DXTUTIL_NO_OVERRIDE
DFLAGS += -DMAIN_HAS_NOARGC
DFLAGS += -DPERFORMANCE_RUN=1 -DITERATIONS=23000

CFLAGS  := -Wa,--longcalls -static -O2  -Wall -mtext-section-literals  -fno-inlin
e-functions
CFLAGS  += -ffunction-sections -fdata-sections  -mlongcalls  $(DFLAGS) $(IFLAGS)
CFLAGS  += -std=c99 -Wextra -Wa,--no-generate-flix -mtarget-align
CFLAGS  += -Wno-unused-variable -Wno-unused-function

LDFLAGS := -nostdlib -Wl,--gc-sections -Wl,--defsym=__prefctl_default=0x144
LDFLAGS += -Wl,--defsym=__memctl_default_post=1
LDFLAGS += -Wl,--abi-windowed
LDFLAGS += -Wl,-Map,$(BUILDDIR)/dsp.map
LDFLAGS += -Wl,--script link.ld

LIBS =  -L ./lib/  -lxtutil  -lhandler-reset -lc -lgloss -lhal -lm -lgcc -lc




src-y += sunxi/main.c
#src-y += sunxi/main.c sunxi/gpio.c sunxi/timer.c sunxi/serial.c
#src-y += generic/crc16_ccitt.c generic/alloc.c
#src-y += generic/timer_irq.c generic/serial_irq.c
