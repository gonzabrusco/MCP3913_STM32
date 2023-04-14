################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCP3913/Src/MCP3913.c \
../Drivers/MCP3913/Src/MCP3913_port.c 

OBJS += \
./Drivers/MCP3913/Src/MCP3913.o \
./Drivers/MCP3913/Src/MCP3913_port.o 

C_DEPS += \
./Drivers/MCP3913/Src/MCP3913.d \
./Drivers/MCP3913/Src/MCP3913_port.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCP3913/Src/%.o Drivers/MCP3913/Src/%.su Drivers/MCP3913/Src/%.cyclo: ../Drivers/MCP3913/Src/%.c Drivers/MCP3913/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Gonzalo/Documents/Firmware/tmp/STM32/FinalPdMPcse/Drivers/MCP3913/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-MCP3913-2f-Src

clean-Drivers-2f-MCP3913-2f-Src:
	-$(RM) ./Drivers/MCP3913/Src/MCP3913.cyclo ./Drivers/MCP3913/Src/MCP3913.d ./Drivers/MCP3913/Src/MCP3913.o ./Drivers/MCP3913/Src/MCP3913.su ./Drivers/MCP3913/Src/MCP3913_port.cyclo ./Drivers/MCP3913/Src/MCP3913_port.d ./Drivers/MCP3913/Src/MCP3913_port.o ./Drivers/MCP3913/Src/MCP3913_port.su

.PHONY: clean-Drivers-2f-MCP3913-2f-Src

