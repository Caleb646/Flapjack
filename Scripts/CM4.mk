CM4_MCU     := cortex-m4
CM4_FPU     := fpv4-sp-d16
CM4_FLOAT   := hard
CM4_SRC     := $(WORKDIR)/CM4/Core/Src
CM4_SRC     += $(WORKDIR)/CM4/Core/Startup
CM4_INC     := $(WORKDIR)/CM4/Core/Inc
CM4_LD      := $(WORKDIR)/CM4/CM4_Flash.ld

SRC_C_CM4 := $(shell find $(CM4_SRC) -name '*.c') 						
SRC_C_CM4 += $(shell find $(COMMON_SRC) -name '*.c')  						

SRC_S_CM4 := $(shell find $(CM4_SRC) -name '*.s')
SRC_S_CM4 += $(shell find $(COMMON_SRC) -name '*.s') 

OBJ_CM4 := $(SRC_C_CM4:$(WORKDIR)/%.c=$(BUILD_DIR)/CM4/%_c.o)
OBJ_CM4 += $(SRC_S_CM4:$(WORKDIR)/%.s=$(BUILD_DIR)/CM4/%_s.o)

INC_CM4 := $(addprefix -I, $(CM4_INC) $(COMMON_INC))
DEP_CM4 := $(OBJ_CM4:%.o=%.d)

DEFINES_CM4		:= 	$(foreach df, $(DEFINES), -D$(df)) -DCORE_CM4
CFLAGS_CM4 		:= 	$(CFLAGS_BASE) -mcpu=$(CM4_MCU) -mfpu=$(CM4_FPU) -mfloat-abi=$(CM4_FLOAT) $(DEFINES_CM4) -mthumb
ASFLAGS_CM4 	:= 	$(ASFLAGS_BASE) -mcpu=$(CM4_MCU) -mfpu=$(CM4_FPU) -mfloat-abi=$(CM4_FLOAT) $(DEFINES_CM4) -mthumb
LDFLAGS_CM4 	:= 	$(LDFLAGS_BASE) -T$(CM4_LD) -mcpu=$(CM4_MCU) -mfpu=$(CM4_FPU) -mfloat-abi=$(CM4_FLOAT) -mthumb

$(BUILD_DIR)/CM4/%_c.o: $(WORKDIR)/%.c
	@mkdir -p $(@D)
	@$(CC) $(CFLAGS_CM4) $(INC_CM4) -c $< -o $@

$(BUILD_DIR)/CM4/%_s.o: $(WORKDIR)/%.s
	@mkdir -p $(@D)
	@$(AS) $(ASFLAGS_CM4) $(INC_CM4) -c $< -o $@

$(BUILD_DIR)/CM4/$(TARGET).elf: $(OBJ_CM4)
	@mkdir -p $(@D)
	@$(LD) $^ -o $@ $(LDFLAGS_CM4)

all: $(BUILD_DIR)/CM4/$(TARGET).elf