################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/caleb/Projects/Drone/Common/Src/mem/queue.c \
C:/Users/caleb/Projects/Drone/Common/Src/mem/ring_buff.c 

OBJS += \
./Common/Src/mem/queue.o \
./Common/Src/mem/ring_buff.o 

C_DEPS += \
./Common/Src/mem/queue.d \
./Common/Src/mem/ring_buff.d 


# Each subdirectory must supply rules for building sources it contributes
Common/Src/mem/queue.o: C:/Users/caleb/Projects/Drone/Common/Src/mem/queue.c Common/Src/mem/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/caleb/Projects/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/mem/ring_buff.o: C:/Users/caleb/Projects/Drone/Common/Src/mem/ring_buff.c Common/Src/mem/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/caleb/Projects/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Common-2f-Src-2f-mem

clean-Common-2f-Src-2f-mem:
	-$(RM) ./Common/Src/mem/queue.cyclo ./Common/Src/mem/queue.d ./Common/Src/mem/queue.o ./Common/Src/mem/queue.su ./Common/Src/mem/ring_buff.cyclo ./Common/Src/mem/ring_buff.d ./Common/Src/mem/ring_buff.o ./Common/Src/mem/ring_buff.su

.PHONY: clean-Common-2f-Src-2f-mem

