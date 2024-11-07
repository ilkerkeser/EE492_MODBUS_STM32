################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Ethernet/socket.c \
../Ethernet/w5500.c \
../Ethernet/wizchip_conf.c 

OBJS += \
./Ethernet/socket.o \
./Ethernet/w5500.o \
./Ethernet/wizchip_conf.o 

C_DEPS += \
./Ethernet/socket.d \
./Ethernet/w5500.d \
./Ethernet/wizchip_conf.d 


# Each subdirectory must supply rules for building sources it contributes
Ethernet/%.o Ethernet/%.su: ../Ethernet/%.c Ethernet/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Ethernet

clean-Ethernet:
	-$(RM) ./Ethernet/socket.d ./Ethernet/socket.o ./Ethernet/socket.su ./Ethernet/w5500.d ./Ethernet/w5500.o ./Ethernet/w5500.su ./Ethernet/wizchip_conf.d ./Ethernet/wizchip_conf.o ./Ethernet/wizchip_conf.su

.PHONY: clean-Ethernet

