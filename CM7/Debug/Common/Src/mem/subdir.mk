ifndef WORKDIR
$(error WORKDIR is not defined. Use 'make WORKDIR=/path/to/root')
endif

SRC := $(WORKDIR)/Common/Src/mem/ring_buff.c
OBJ := ./Common/Src/mem/ring_buff.o
DEP := ./Common/Src/mem/ring_buff.d

CC := arm-none-eabi-gcc
CFLAGS := \
  -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY \
  -c \
  -I$(WORKDIR)/Core/Inc \
  -I$(WORKDIR)/Drivers/STM32H7xx_HAL_Driver/Inc \
  -I$(WORKDIR)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy \
  -I$(WORKDIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
  -I$(WORKDIR)/Drivers/CMSIS/Include \
  -I$(WORKDIR)/Common/Inc \
  -I$(WORKDIR)/Middlewares/Third_Party/FreeRTOS/Source/include \
  -I$(WORKDIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
  -I$(WORKDIR)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
  -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity \
  -MMD -MP \
  -MF"$(@:%.o=%.d)" -MT"$@" \
  --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb

# Build rule
$(OBJ): $(SRC) Common/Src/mem/subdir.mk
	$(CC) $(CFLAGS) -o "$@" "$<"

# Clean rule
clean: clean-mem

clean-mem:
	-$(RM) ./Common/Src/mem/ring_buff.cyclo ./Common/Src/mem/ring_buff.d ./Common/Src/mem/ring_buff.o ./Common/Src/mem/ring_buff.su

.PHONY: clean clean-mem
