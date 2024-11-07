################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Application_Driver/loopback.c 

OBJS += \
./Application/Application_Driver/loopback.o 

C_DEPS += \
./Application/Application_Driver/loopback.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Application_Driver/%.o Application/Application_Driver/%.su: ../Application/Application_Driver/%.c Application/Application_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ILKER KESER/STM32CubeIDE/Bitirme_MODBUS/007Modbus/Application/Application_Lib" -I"C:/Users/ILKER KESER/STM32CubeIDE/Bitirme_MODBUS/007Modbus/Ethernet/Library_W5500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Application_Driver

clean-Application-2f-Application_Driver:
	-$(RM) ./Application/Application_Driver/loopback.d ./Application/Application_Driver/loopback.o ./Application/Application_Driver/loopback.su

.PHONY: clean-Application-2f-Application_Driver

