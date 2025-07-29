################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/mc/actuators.c \
C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/mc/dshot.c \
C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/mc/filter.c \
C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/mc/pwm.c 

OBJS += \
./Common/Src/mc/actuators.o \
./Common/Src/mc/dshot.o \
./Common/Src/mc/filter.o \
./Common/Src/mc/pwm.o 

C_DEPS += \
./Common/Src/mc/actuators.d \
./Common/Src/mc/dshot.d \
./Common/Src/mc/filter.d \
./Common/Src/mc/pwm.d 


# Each subdirectory must supply rules for building sources it contributes
Common/Src/mc/actuators.o: C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/mc/actuators.c Common/Src/mc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/mc/dshot.o: C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/mc/dshot.c Common/Src/mc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/mc/filter.o: C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/mc/filter.c Common/Src/mc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/mc/pwm.o: C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Src/mc/pwm.c Common/Src/mc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/c.thomas/STM32CubeIDE/workspace_1.18.0/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Common-2f-Src-2f-mc

clean-Common-2f-Src-2f-mc:
	-$(RM) ./Common/Src/mc/actuators.cyclo ./Common/Src/mc/actuators.d ./Common/Src/mc/actuators.o ./Common/Src/mc/actuators.su ./Common/Src/mc/dshot.cyclo ./Common/Src/mc/dshot.d ./Common/Src/mc/dshot.o ./Common/Src/mc/dshot.su ./Common/Src/mc/filter.cyclo ./Common/Src/mc/filter.d ./Common/Src/mc/filter.o ./Common/Src/mc/filter.su ./Common/Src/mc/pwm.cyclo ./Common/Src/mc/pwm.d ./Common/Src/mc/pwm.o ./Common/Src/mc/pwm.su

.PHONY: clean-Common-2f-Src-2f-mc

