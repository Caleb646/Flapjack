################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/caleb/Projects/Drone/Common/Src/motion_control/actuators.c \
C:/Users/caleb/Projects/Drone/Common/Src/motion_control/filter.c 

OBJS += \
./Common/Src/motion_control/actuators.o \
./Common/Src/motion_control/filter.o 

C_DEPS += \
./Common/Src/motion_control/actuators.d \
./Common/Src/motion_control/filter.d 


# Each subdirectory must supply rules for building sources it contributes
Common/Src/motion_control/actuators.o: C:/Users/caleb/Projects/Drone/Common/Src/motion_control/actuators.c Common/Src/motion_control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/caleb/Projects/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/motion_control/filter.o: C:/Users/caleb/Projects/Drone/Common/Src/motion_control/filter.c Common/Src/motion_control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/caleb/Projects/Drone/Common/Inc" -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Common-2f-Src-2f-motion_control

clean-Common-2f-Src-2f-motion_control:
	-$(RM) ./Common/Src/motion_control/actuators.cyclo ./Common/Src/motion_control/actuators.d ./Common/Src/motion_control/actuators.o ./Common/Src/motion_control/actuators.su ./Common/Src/motion_control/filter.cyclo ./Common/Src/motion_control/filter.d ./Common/Src/motion_control/filter.o ./Common/Src/motion_control/filter.su

.PHONY: clean-Common-2f-Src-2f-motion_control

