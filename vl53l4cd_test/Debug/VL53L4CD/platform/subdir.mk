################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L4CD/platform/platform.c 

OBJS += \
./VL53L4CD/platform/platform.o 

C_DEPS += \
./VL53L4CD/platform/platform.d 


# Each subdirectory must supply rules for building sources it contributes
VL53L4CD/platform/%.o VL53L4CD/platform/%.su VL53L4CD/platform/%.cyclo: ../VL53L4CD/platform/%.c VL53L4CD/platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../VL53L4CD/core -I../VL53L4CD/platform -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32H7xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L4CD-2f-platform

clean-VL53L4CD-2f-platform:
	-$(RM) ./VL53L4CD/platform/platform.cyclo ./VL53L4CD/platform/platform.d ./VL53L4CD/platform/platform.o ./VL53L4CD/platform/platform.su

.PHONY: clean-VL53L4CD-2f-platform

