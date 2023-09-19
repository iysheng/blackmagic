CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc

CFLAGS += -DENABLE_DEBUG

ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nosys.specs
endif

# VPATH += platforms/common/stm32

SRC +=               \
	platform.c \
	gdb_if.c \
	traceswodecode.c \
	traceswo.c	\
	serialno.c	\
	timing.c	\
	timing_stm32.c	\

all:	blackmagic

host_clean:
	-$(Q)$(RM) blackmagic.bin blackmagic_dfu blackmagic_dfu.bin blackmagic_dfu.hex
