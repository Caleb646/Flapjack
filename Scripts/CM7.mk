CM7_MCU     := cortex-m7
CM7_FPU     := fpv5-d16
CM7_FLOAT   := hard
CM7_SRC     := $(WORKDIR)/CM7/Core/Src
CM7_SRC     += $(WORKDIR)/CM7/Core/Startup
CM7_INC     := $(WORKDIR)/CM7/Core/Inc
CM7_LD      := $(WORKDIR)/CM7/CM7_Flash.ld

SRC_C_CM7 := $(shell find $(CM7_SRC) -name '*.c') 						
SRC_C_CM7 += $(shell find $(COMMON_SRC) -name '*.c')  						

SRC_S_CM7 := $(shell find $(CM7_SRC) -name '*.s')
SRC_S_CM7 += $(shell find $(COMMON_SRC) -name '*.s') 

OBJ_CM7 := $(SRC_C_CM7:$(WORKDIR)/%.c=$(BUILD_DIR)/CM7/%_c.o)
OBJ_CM7 += $(SRC_S_CM7:$(WORKDIR)/%.s=$(BUILD_DIR)/CM7/%_s.o)

INC_CM7 := $(addprefix -I, $(CM7_INC) $(COMMON_INC))
DEP_CM7 := $(OBJ_CM7:%.o=%.d)

DEFINES_CM7		:= 	$(foreach df, $(DEFINES), -D$(df)) -DCORE_CM7
CFLAGS_CM7 		:= 	$(CFLAGS_BASE) -mcpu=$(CM7_MCU) -mfpu=$(CM7_FPU) -mfloat-abi=$(CM7_FLOAT) $(DEFINES_CM7) -mthumb
ASFLAGS_CM7 	:= 	$(ASFLAGS_BASE) -mcpu=$(CM7_MCU) -mfpu=$(CM7_FPU) -mfloat-abi=$(CM7_FLOAT) $(DEFINES_CM7) -mthumb
LDFLAGS_CM7 	:= 	$(LDFLAGS_BASE) -T$(CM7_LD) -mcpu=$(CM7_MCU) -mfpu=$(CM7_FPU) -mfloat-abi=$(CM7_FLOAT) -mthumb

$(BUILD_DIR)/CM7/%_c.o: $(WORKDIR)/%.c
	@mkdir -p $(@D)
	@$(CC) $(CFLAGS_CM7) $(INC_CM7) -c $< -o $@

$(BUILD_DIR)/CM7/%_s.o: $(WORKDIR)/%.s
	@mkdir -p $(@D)
	@$(AS) $(ASFLAGS_CM7) $(INC_CM7) -c $< -o $@

$(BUILD_DIR)/CM7/$(TARGET).elf: $(OBJ_CM7)
	@mkdir -p $(@D)
	@$(LD) $^ -o $@ $(LDFLAGS_CM7)

all: $(BUILD_DIR)/CM7/$(TARGET).elf