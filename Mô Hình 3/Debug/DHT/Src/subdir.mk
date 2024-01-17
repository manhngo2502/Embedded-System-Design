################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DHT/Src/DHT.c 

OBJS += \
./DHT/Src/DHT.o 

C_DEPS += \
./DHT/Src/DHT.d 


# Each subdirectory must supply rules for building sources it contributes
DHT/Src/%.o DHT/Src/%.su DHT/Src/%.cyclo: ../DHT/Src/%.c DHT/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu99 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/admin/STM32CubeIDE/workspace_1.14.0/Timing_driven/dht11/Inc" -I"C:/Users/admin/STM32CubeIDE/workspace_1.14.0/Timing_driven/lcd602_i2c/Inc" -I"C:/Users/admin/STM32CubeIDE/workspace_1.14.0/Timing_driven/DHT/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-DHT-2f-Src

clean-DHT-2f-Src:
	-$(RM) ./DHT/Src/DHT.cyclo ./DHT/Src/DHT.d ./DHT/Src/DHT.o ./DHT/Src/DHT.su

.PHONY: clean-DHT-2f-Src

