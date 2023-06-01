ifneq ($(V), 1)
MFLAGS += --no-print-dir
Q := @
endif

PC_HOSTED =
NO_LIBOPENCM3 =
# 只有在编译 host 平台时才会设置 NO_LIBOPENCM3 为 true
ifeq ($(PROBE_HOST), hosted)
	PC_HOSTED = true
	NO_LIBOPENCM3 = true
endif

all:
ifndef NO_LIBOPENCM3
	$(Q)if [ ! -f libopencm3/Makefile ]; then \
		echo "Initialising git submodules..." ;\
		git submodule init ;\
		git submodule update ;\
	fi
	# 如果需要 libopencm3 会走到这里,需要去这些目录中编译一些内容
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 lib/stm32/f1 lib/stm32/f4 lib/lm4f
endif
# 去 src 目录执行默认的编译
	$(Q)$(MAKE) $(MFLAGS) -C src

all_platforms:
	$(Q)$(MAKE) $(MFLAGS) -C src $@

clean:
ifndef NO_LIBOPENCM3
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 $@
endif
	$(Q)$(MAKE) $(MFLAGS) -C src $@

clang-tidy: SYSTEM_INCLUDE_PATHS=$(shell pkg-config --silence-errors --cflags libusb-1.0 libftdi1)
clang-tidy:
	$(Q)scripts/run-clang-tidy.py -s "$(PWD)" $(SYSTEM_INCLUDE_PATHS)

clang-format:
	$(Q)$(MAKE) $(MFLAGS) -C src $@

.PHONY: clean all_platforms clang-tidy clang-format
