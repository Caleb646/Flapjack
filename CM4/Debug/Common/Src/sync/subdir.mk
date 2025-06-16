################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/sync/mailbox.c \
C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/sync/sync.c 

OBJS += \
./Common/Src/sync/mailbox.o \
./Common/Src/sync/sync.o 

C_DEPS += \
./Common/Src/sync/mailbox.d \
./Common/Src/sync/sync.d 


# Each subdirectory must supply rules for building sources it contributes
Common/Src/sync/mailbox.o: C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/sync/mailbox.c Common/Src/sync/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/sync/sync.o: C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/sync/sync.c Common/Src/sync/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Common-2f-Src-2f-sync

clean-Common-2f-Src-2f-sync:
	-$(RM) ./Common/Src/sync/mailbox.cyclo ./Common/Src/sync/mailbox.d ./Common/Src/sync/mailbox.o ./Common/Src/sync/mailbox.su ./Common/Src/sync/sync.cyclo ./Common/Src/sync/sync.d ./Common/Src/sync/sync.o ./Common/Src/sync/sync.su

.PHONY: clean-Common-2f-Src-2f-sync

