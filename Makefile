WORKDIR := /Users/caleb/Projects/Drone

# === Config ===
TARGET      := firmware
BUILD_DIR   := build
CORES       := CM4 CM7

# Shared source/include dirs
COMMON_SRC  := $(WORKDIR)/Common/Src
COMMON_INC  := $(WORKDIR)/Common/Inc

# Compiler toolchain
CC          := arm-none-eabi-gcc
LD          := arm-none-eabi-gcc
OBJCOPY     := arm-none-eabi-objcopy
CFLAGS_BASE := -Wall -Werror -g -O0 -ffunction-sections -fdata-sections -std=gnu11
LDFLAGS_BASE := -specs=nosys.specs -Wl,--gc-sections

# Core-specific config
CM4_MCU     := cortex-m4
CM4_FPU     := fpv4-sp-d16
CM4_FLOAT   := hard
CM4_SRC     := cm4/src
CM4_INC     := cm4/include
CM4_LD      := cm4/linker.ld

CM7_MCU     := cortex-m7
CM7_FPU     := fpv5-d16
CM7_FLOAT   := hard
CM7_SRC     := \
	$(WORKDIR)/CM7/Core/Inc \
	$(WORKDIR)/CM7/Core/Startup \
	$(WORKDIR)/CM7/Drivers/STM32H7xx_HAL_Driver \
	$(WORKDIR)/CM7/Middlewares/Third_Party/FreeRTOS \
CM7_INC     := cm7/include
CM7_LD      := cm7/linker.ld

# === Core-Specific build rules (generated per core) ===
define BUILD_CORE

SRC_$(1) := $$(shell find $$($(1)_SRC) -name '*.c')
OBJ_$(1) := $$(patsubst %.c,$(BUILD_DIR)/$(1)/%.o,$$(SRC_$(1)))
DEP_$(1) := $$(OBJ_$(1):%.o=%.d)
INC_$(1) := -I$$($(1)_INC)

CFLAGS_$(1) := $$(CFLAGS_BASE) -mcpu=$$($(1)_MCU) -mfpu=$$($(1)_FPU) -mfloat-abi=$$($(1)_FLOAT) -mthumb
LDFLAGS_$(1) := $$(LDFLAGS_BASE) -T$$($(1)_LD) -mcpu=$$($(1)_MCU) -mfpu=$$($(1)_FPU) -mfloat-abi=$$($(1)_FLOAT) -mthumb

ALL_ELFS += $(BUILD_DIR)/$(1)/$(TARGET).elf

$(BUILD_DIR)/$(1)/$(TARGET).elf: $$($(OBJ_$(1)))
	@mkdir -p $$(@D)
	$$(LD) $$^ -o $$@ $$(LDFLAGS_$(1))

$$(BUILD_DIR)/$(1)/%.o: %.c
	@mkdir -p $$(@D)
	$$(CC) $$(CFLAGS_$(1)) $$(INC_$(1)) -c $$< -o $$@

-include $$($(DEP_$(1)))

endef

$(foreach core,$(CORES),$(eval $(call BUILD_CORE,$(core))))

# === Targets ===

all: $(ALL_ELFS)

bin:
	$(foreach core,$(CORES),\
		$(OBJCOPY) -O binary $(BUILD_DIR)/$(core)/$(TARGET).elf $(BUILD_DIR)/$(core)/$(TARGET).bin; \
	)

flash: bin
	st-flash write $(BUILD_DIR)/CM7/$(TARGET).bin 0x08000000
	st-flash write $(BUILD_DIR)/CM4/$(TARGET).bin 0x08100000

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all bin flash clean



all:
	$(MAKE) -C ./CM4/Debug WORKDIR=$(WORKDIR)
	$(MAKE) -C ./CM7/Debug WORKDIR=$(WORKDIR)