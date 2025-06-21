WORKDIR := /home/cthomas/projects/Drone

# === Config ===
TARGET      := firmware
BUILD_DIR   := Build
CORES       := CM4 CM7

# Shared source/include dirs
COMMON_SRC  := $(WORKDIR)/Middlewares
COMMON_SRC  += $(WORKDIR)/Drivers
COMMON_SRC  += $(WORKDIR)/Common/Src

COMMON_INC  := 	$(WORKDIR)/Drivers/STM32H7xx_HAL_Driver/Inc
COMMON_INC  += 	$(WORKDIR)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy 						
COMMON_INC  += 	$(WORKDIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include 					
COMMON_INC  += 	$(WORKDIR)/Drivers/CMSIS/Include 										
COMMON_INC  += 	$(WORKDIR)/Common/Inc 													
COMMON_INC  += 	$(WORKDIR)/Middlewares/Third_Party/FreeRTOS/Source/include				
COMMON_INC  += 	$(WORKDIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2		
COMMON_INC  += 	$(WORKDIR)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

# Compiler toolchain
CC          := arm-none-eabi-gcc
AS          := arm-none-eabi-gcc
LD          := arm-none-eabi-gcc
OBJCOPY     := arm-none-eabi-objcopy
# -Wall -Werror
DEFINES 	:= DEBUG USE_HAL_DRIVER STM32H747xx USE_PWR_DIRECT_SMPS_SUPPLY
CFLAGS_BASE := -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections
CFLAGS_BASE += -Wall -fstack-usage -MMD -MP --specs=nano.specs

ASFLAGS_BASE := -x assembler-with-cpp
LDFLAGS_BASE := -specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs -Wl,--start-group -lc -lm -Wl,--end-group
# -Wl,-Map="Map.map"

define PRINT_SOURCES
print-$(1):
	@echo "Source files for $(1):"
	@echo "$$(INC_$(1))"
endef
# $(foreach core,$(CORES),$(eval $(call PRINT_SOURCES,$(core))))

SHARED_VARS := WORKDIR="$(WORKDIR)"
SHARED_VARS += TARGET="$(TARGET)"
SHARED_VARS += BUILD_DIR="$(BUILD_DIR)"
SHARED_VARS += COMMON_SRC="$(COMMON_SRC)"
SHARED_VARS += COMMON_INC="$(COMMON_INC)"
SHARED_VARS += DEFINES="$(DEFINES)"
SHARED_VARS += CFLAGS_BASE="$(CFLAGS_BASE)"
SHARED_VARS += ASFLAGS_BASE="$(ASFLAGS_BASE)"
SHARED_VARS += LDFLAGS_BASE="$(LDFLAGS_BASE)"
SHARED_VARS += CC="$(CC)" AS="$(AS)" LD="$(LD)" OBJCOPY="$(OBJCOPY)"

all: cm4 cm7

cm4:
	@echo "Calling cm4.mk"
	@$(MAKE) -f Scripts/CM4.mk $(SHARED_VARS)

cm7:
	@echo "Calling cm7.mk"
	@$(MAKE) -f Scripts/CM7.mk $(SHARED_VARS)

bin:
	$(foreach core,$(CORES),\
		$(OBJCOPY) -O binary $(BUILD_DIR)/$(core)/$(TARGET).elf $(BUILD_DIR)/$(core)/$(TARGET).bin; \
	)

flash: bin
	st-flash write $(BUILD_DIR)/CM7/$(TARGET).bin 0x08000000
	st-flash write $(BUILD_DIR)/CM4/$(TARGET).bin 0x08100000

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all bin flash clean cm4 cm7