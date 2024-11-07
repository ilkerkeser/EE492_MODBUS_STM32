################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Ethernet/W5500_Driver/socket.c \
../Ethernet/W5500_Driver/w5500.c \
../Ethernet/W5500_Driver/wizchip_conf.c 

OBJS += \
./Ethernet/W5500_Driver/socket.o \
./Ethernet/W5500_Driver/w5500.o \
./Ethernet/W5500_Driver/wizchip_conf.o 

C_DEPS += \
./Ethernet/W5500_Driver/socket.d \
./Ethernet/W5500_Driver/w5500.d \
./Ethernet/W5500_Driver/wizchip_conf.d 


# Each subdirectory must supply rules for building sources it contributes
Ethernet/W5500_Driver/%.o Ethernet/W5500_Driver/%.su: ../Ethernet/W5500_Driver/%.c Ethernet/W5500_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ILKER KESER/STM32CubeIDE/Bitirme_MODBUS/007Modbus/Ethernet/W5500_Driver" -I"C:/Users/ILKER KESER/STM32CubeIDE/Bitirme_MODBUS/007Modbus/Ethernet/Library_W5500" -I"C:/Users/ILKER KESER/STM32CubeIDE/Bitirme_MODBUS/007Modbus/Application/Application_Lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Ethernet-2f-W5500_Driver

clean-Ethernet-2f-W5500_Driver:
	-$(RM) ./Ethernet/W5500_Driver/socket.d ./Ethernet/W5500_Driver/socket.o ./Ethernet/W5500_Driver/socket.su ./Ethernet/W5500_Driver/w5500.d ./Ethernet/W5500_Driver/w5500.o ./Ethernet/W5500_Driver/w5500.su ./Ethernet/W5500_Driver/wizchip_conf.d ./Ethernet/W5500_Driver/wizchip_conf.o ./Ethernet/W5500_Driver/wizchip_conf.su

.PHONY: clean-Ethernet-2f-W5500_Driver

