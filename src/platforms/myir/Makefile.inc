CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc

CFLAGS += -DENABLE_DEBUG

# VPATH += platforms/common/stm32

SRC +=               \
	platform.c \
	debug.c

all:	blackmagic

host_clean:
	-$(Q)$(RM) blackmagic.bin blackmagic_dfu blackmagic_dfu.bin blackmagic_dfu.hex
