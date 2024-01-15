### Taken from https://github.com/libopencm3/libopencm3-template
#
### REQUIRED ###
# OPENCM3_DIR - duh
# PROJECT - will be the basename of the output elf, eg usb-gadget0-stm32f4disco
# CFILES - basenames only, eg main.c blah.c
# DEVICE - the full device name, eg stm32f405ret6
#  _or_
# LDSCRIPT - full path, eg ../../examples/stm32/f4/stm32f4-discovery/stm32f4-discovery.ld
# OPENCM3_LIB - the basename, eg: opencm3_stm32f4
# OPENCM3_DEFS - the target define eg: -DSTM32F4
# ARCH_FLAGS - eg, -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
#    (ie, the full set of cpu arch flags, _none_ are defined in this file)
#
### OPTIONAL ###
# INCLUDES - include paths, if you want extra, eg: INCLUDES = shared libs/some/inc
# BUILD_DIR - defaults to bin, should set this if you are building multiarch
# OPT - full -O flag, defaults to -Os
# CSTD - defaults -std=c99
# CXXSTD - no default.

include $(OPENCM3_DIR)/mk/genlink-config.mk

BUILD_DIR ?= bin
OPT ?= -Os
CSTD ?= -std=c99

V ?= 0
ifeq ($(V),0)
 Q := @
 NULL	:= 2>/dev/null
endif

PREFIX ?= arm-none-eabi-
CC = $(PREFIX)gcc
LD = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE = $(PREFIX)size
STM32FLASH = $(shell which stm32flash)

OPENCM3_INC = $(OPENCM3_DIR)/include

TGT_IFLAGS += $(patsubst %,-I%,$(INCLUDES))
TGT_IFLAGS += $(patsubst %,-I%, . $(OPENCM3_INC) )

OBJS = $(CFILES:%.c=$(BUILD_DIR)/%.o)
GENERATED_BINS = $(PROJECT).elf $(PROJECT).bin $(PROJECT).map $(PROJECT).list $(PROJECT).lss

TGT_CPPFLAGS += -MD
TGT_CPPFLAGS += -Wall -Wundef
TGT_CPPFLAGS += $(TGT_IFLAGS) $(OPENCM3_DEFS)

TGT_CFLAGS += $(OPT) $(CSTD) -ggdb3
TGT_CFLAGS += $(ARCH_FLAGS)
TGT_CFLAGS += -fno-common
TGT_CFLAGS += -ffunction-sections -fdata-sections
TGT_CFLAGS += -Wextra -Wshadow -Wno-unused-variable -Wimplicit-function-declaration
TGT_CFLAGS += -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes

TGT_CXXFLAGS += $(OPT) $(CXXSTD) -ggdb3
TGT_CXXFLAGS += $(ARCH_FLAGS)
TGT_CXXFLAGS += -fno-common
TGT_CXXFLAGS += -ffunction-sections -fdata-sections
TGT_CXXFLAGS += -Wextra -Wshadow -Wredundant-decls  -Weffc++

TGT_LDFLAGS += -T$(LDSCRIPT) -L$(OPENCM3_DIR)/lib -nostartfiles
TGT_LDFLAGS += $(ARCH_FLAGS)
TGT_LDFLAGS += -specs=nano.specs
TGT_LDFLAGS += -Wl,--gc-sections
ifeq ($(V),99)
 TGT_LDFLAGS += -Wl,--print-gc-sections
endif

# Linker script generator fills this in for us.
ifeq (,$(DEVICE))
 LDLIBS += -l$(OPENCM3_LIB)
endif
LDLIBS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

.SUFFIXES:
.SUFFIXES: .c .h .o .cxx .elf .bin .list .lss

%: %,v
%: RCS/%,v
%: RCS/%

all: $(PROJECT).elf $(PROJECT).bin stat
flash: $(PROJECT).flash

# error if not using linker script generator
ifeq (,$(DEVICE))
$(LDSCRIPT):
ifeq (,$(wildcard $(LDSCRIPT)))
 $(error Unable to find specified linker script: $(LDSCRIPT))
endif
else
 # if linker script generator was used, make sure it's cleaned.
 GENERATED_BINS += $(LDSCRIPT)
endif

$(BUILD_DIR)/%.o: %.c
	@printf "  CC      $<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_CFLAGS) $(CFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $<

$(BUILD_DIR)/%.o: %.cxx
	@printf "  CXX     $<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_CXXFLAGS) $(CXXFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $<

$(PROJECT).elf: $(OBJS) $(LDSCRIPT) $(LIBDEPS)
	@printf "  LD      $@\n"
	$(Q)$(LD) $(TGT_LDFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $@

%.bin: %.elf
	@printf "  OBJCOPY $@\n"
	$(Q)$(OBJCOPY) -O binary $< $@

stat: $(PROJECT).bin
	@printf "\n  BINARY STATS:\n"
	$(Q)$(SIZE) --target=binary $<

%.lss: %.elf
	$(Q)$(OBJDUMP) -h -S $< > $@

%.list: %.elf
	$(Q)$(OBJDUMP) -S $< > $@

%.flash: %.bin
	@printf "  FLASH   $<\n"
	$(Q)$(STM32FLASH) -w $< -v -g 0x0 /dev/$(FLASH_PORT)

clean:
	$(Q)rm -rf $(BUILD_DIR) $(GENERATED_BINS)

.PHONY: all clean flash
-include $(OBJS:.o=.d)

include $(OPENCM3_DIR)/mk/genlink-rules.mk
