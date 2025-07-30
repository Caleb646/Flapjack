################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/coms/uart.c 

OBJS += \
./Common/Src/coms/uart.o 

C_DEPS += \
./Common/Src/coms/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Common/Src/coms/uart.o: C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/coms/uart.c Common/Src/coms/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Common-2f-Src-2f-coms

clean-Common-2f-Src-2f-coms:
	-$(RM) ./Common/Src/coms/uart.cyclo ./Common/Src/coms/uart.d ./Common/Src/coms/uart.o ./Common/Src/coms/uart.su

.PHONY: clean-Common-2f-Src-2f-coms

