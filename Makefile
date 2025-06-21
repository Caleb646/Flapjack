WORKDIR := /home/cthomas/projects/Drone

# === Config ===
TARGET      := firmware
BUILD_DIR   := Build
CORES       := CM4 CM7

# Shared source/include dirs
COMMON_SRC  := 								\
	$(WORKDIR)/Middlewares					\
	$(WORKDIR)/Drivers 						\
	$(WORKDIR)/Common/Src 					\	
	 					

COMMON_INC  := 																	\
	$(WORKDIR)/Drivers/STM32H7xx_HAL_Driver/Inc 								\
	$(WORKDIR)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy 							\
	$(WORKDIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include 						\
	$(WORKDIR)/Drivers/CMSIS/Include 											\
	$(WORKDIR)/Common/Inc 														\
	$(WORKDIR)/Middlewares/Third_Party/FreeRTOS/Source/include					\
	$(WORKDIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2			\
	$(WORKDIR)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F	\

# Compiler toolchain
CC          := arm-none-eabi-gcc
AS          := arm-none-eabi-gcc
LD          := arm-none-eabi-gcc
OBJCOPY     := arm-none-eabi-objcopy
# -Wall -Werror
CFLAGS_BASE := -std=gnu11 -g3 -O0 -ffunction-sections -fdata-sections
CFLAGS_BASE += -Wall -fstack-usage -MMD -MP --specs=nano.specs
CFLAGS_BASE += -DDEBUG -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY

ASFLAGS_BASE := -x assembler-with-cpp
LDFLAGS_BASE := -specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs -Wl,-Map="Map.map" -Wl,--start-group -lc -lm -Wl,--end-group

# Core-specific config
CM4_MCU     := cortex-m4
CM4_FPU     := fpv4-sp-d16
CM4_FLOAT   := hard

CM4_SRC     := 						\
	$(WORKDIR)/CM4/Core/Src 		\
	$(WORKDIR)/CM4/Core/Startup

CM4_INC     :=  					\
	$(WORKDIR)/CM4/Core/Inc 

CM4_LD      := $(WORKDIR)/CM4/CM4_Flash.ld

CM7_MCU     := cortex-m7
CM7_FPU     := fpv5-d16
CM7_FLOAT   := hard

CM7_SRC     := 						\
	$(WORKDIR)/CM7/Core/Src 		\
	$(WORKDIR)/CM7/Core/Startup

CM7_INC     := \
	$(WORKDIR)/CM7/Core/Inc

CM7_LD      := $(WORKDIR)/CM7/CM7_Flash.ld

# === Core-Specific build rules (generated per core) ===
# OBJ_C_$(1) := $$(patsubst $(WORKDIR)/%.c,$(BUILD_DIR)/$(1)/%.o,$$(SRC_C_$(1)))
# OBJ_S_$(1) := $$(patsubst $(WORKDIR)/%.s,$(BUILD_DIR)/$(1)/%.o,$$(SRC_S_$(1)))

# OBJ_C_$(1) := $(patsubst $(WORKDIR)/%, $(BUILD_DIR)/$(1)/%, $(SRC_C_$(1):%.c=%.o))
# OBJ_S_$(1) := $(patsubst $(WORKDIR)/%, $(BUILD_DIR)/$(1)/%, $(SRC_S_$(1):%.s=%.o))
# OBJ_$(1) := $$(OBJ_C_$(1)) $$(OBJ_S_$(1))
# DEP_$(1) := $$(OBJ_C_$(1):%.o=%.d)

# -o "Drone_CM4.elf" @"objects.list" $(USER_OBJS) $(LIBS) 
# -mcpu=cortex-m4 -T"C:\Users\caleb\Projects\Drone\CM4\CM4_Flash.ld" 
# --specs=nosys.specs -Wl,-Map="Drone_CM4.map" -Wl,
# --gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb 
# -Wl,--start-group -lc -lm -Wl,--end-group

ALL_ELFS :=

define BUILD_CORE

SRC_C_$(1) := \
	$$(shell find $$($(1)_SRC) -name '*.c') \
	$$(shell find $(COMMON_SRC) -name '*.c')

SRC_S_$(1) := \
	$$(shell find $$($(1)_SRC) -name '*.s') \
	$$(shell find $(COMMON_SRC) -name '*.s') 

OBJ_$(1) = $$(SRC_C_$(1):$(WORKDIR)/%.c=$(BUILD_DIR)/$(1)/%_c.o)
OBJ_$(1) += $$(SRC_S_$(1):$(WORKDIR)/%.s=$(BUILD_DIR)/$(1)/%_s.o)
DEP_$(1) = $$(OBJ_$(1):%.o=%.d)

INC_$(1) := $(addprefix -I, $$($(1)_INC) $(COMMON_INC))

CFLAGS_$(1) := $$(CFLAGS_BASE) -mcpu=$$($(1)_MCU) -mfpu=$$($(1)_FPU) -mfloat-abi=$$($(1)_FLOAT) -DCORE_$$($(1)) -mthumb
ASFLAGS_$(1) := $$(ASFLAGS_BASE) -mcpu=$$($(1)_MCU) -mfpu=$$($(1)_FPU) -mfloat-abi=$$($(1)_FLOAT) -mthumb
LDFLAGS_$(1) := $$(LDFLAGS_BASE) -T$$($(1)_LD) -mcpu=$$($(1)_MCU) -mfpu=$$($(1)_FPU) -mfloat-abi=$$($(1)_FLOAT) -mthumb

ALL_ELFS += $(BUILD_DIR)/$(1)/$(TARGET).elf

-include $$($(DEP_$(1)))

endef

# $$(BUILD_DIR)/$(1)/%_c.o: %.c
# 	mkdir -p $$(@D)
# 	$$(CC) $$(CFLAGS_$(1)) $$(INC_$(1)) -c $$< -o $$@

# $$(BUILD_DIR)/$(1)/%_s.o: %.s
# 	mkdir -p $$(@D)
# 	$$(AS) $$(ASFLAGS_$(1)) $$(INC_$(1)) -c $$< -o $$@

# $(BUILD_DIR)/$(1)/$(TARGET).elf: $$($(OBJ_$(1)))
# 	mkdir -p $$(@D)
# 	$$(LD) $$^ -o $$@ $$(LDFLAGS_$(1))

$(foreach core,$(CORES),$(info Generating rules for: $(core)))
$(foreach core,$(CORES),$(eval $(call BUILD_CORE,$(core))))

# @echo "$$(SRC_C_$(1))"
define PRINT_SOURCES
print-$(1):
	@echo "Source files for $(1):"
	@echo "$$(DEP_$(1))"
endef

# $(foreach core,$(CORES),$(eval $(call PRINT_SOURCES,$(core))))

# === Targets ===

$(BUILD_DIR)/CM4/%_c.o: %.c
	mkdir -p $(@D)
	$(CC) $(CFLAGS_CM4) $(INC_CM4) -c $< -o $@

$(BUILD_DIR)/CM4/%_s.o: %.s
	mkdir -p $(@D)
	$(AS) $(ASFLAGS_CM4) $(INC_CM4) -c $< -o $@

all: $(BUILD_DIR)/CM4/$(TARGET).elf $(OBJ_CM4)
	mkdir -p $(@D)
	$(LD) $^ -o $@ $(LDFLAGS_CM4)

# all: $(ALL_ELFS)
# all: $(BUILD_DIR)/CM4/$(TARGET).elf $(BUILD_DIR)/CM7/$(TARGET).elf

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



# all:
# 	$(MAKE) -C ./CM4/Debug WORKDIR=$(WORKDIR)
# 	$(MAKE) -C ./CM7/Debug WORKDIR=$(WORKDIR)