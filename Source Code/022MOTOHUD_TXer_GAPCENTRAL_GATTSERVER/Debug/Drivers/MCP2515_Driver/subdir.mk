################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCP2515_Driver/mcp2515_driver.c 

OBJS += \
./Drivers/MCP2515_Driver/mcp2515_driver.o 

C_DEPS += \
./Drivers/MCP2515_Driver/mcp2515_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCP2515_Driver/%.o Drivers/MCP2515_Driver/%.su Drivers/MCP2515_Driver/%.cyclo: ../Drivers/MCP2515_Driver/%.c Drivers/MCP2515_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"D:/Embedded-C/MCUProjects/MCUProjects_HAL_BASED_NEW/022MOTOHUD_TXer_GAPCENTRAL_GATTSERVER/Drivers/MCP2515_Driver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MCP2515_Driver

clean-Drivers-2f-MCP2515_Driver:
	-$(RM) ./Drivers/MCP2515_Driver/mcp2515_driver.cyclo ./Drivers/MCP2515_Driver/mcp2515_driver.d ./Drivers/MCP2515_Driver/mcp2515_driver.o ./Drivers/MCP2515_Driver/mcp2515_driver.su

.PHONY: clean-Drivers-2f-MCP2515_Driver

