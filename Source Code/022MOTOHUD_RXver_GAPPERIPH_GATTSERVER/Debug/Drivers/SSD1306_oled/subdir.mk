################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SSD1306_oled/glyph_assets.c \
../Drivers/SSD1306_oled/ssd1306_ssd1315.c 

OBJS += \
./Drivers/SSD1306_oled/glyph_assets.o \
./Drivers/SSD1306_oled/ssd1306_ssd1315.o 

C_DEPS += \
./Drivers/SSD1306_oled/glyph_assets.d \
./Drivers/SSD1306_oled/ssd1306_ssd1315.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SSD1306_oled/%.o Drivers/SSD1306_oled/%.su Drivers/SSD1306_oled/%.cyclo: ../Drivers/SSD1306_oled/%.c Drivers/SSD1306_oled/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I"D:/Embedded-C/MCUProjects/MCUProjects_HAL_BASED_NEW/022MOTOHUD_RXver_GAPPERIPH_GATTCLIENT/Drivers/SSD1306_oled" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SSD1306_oled

clean-Drivers-2f-SSD1306_oled:
	-$(RM) ./Drivers/SSD1306_oled/glyph_assets.cyclo ./Drivers/SSD1306_oled/glyph_assets.d ./Drivers/SSD1306_oled/glyph_assets.o ./Drivers/SSD1306_oled/glyph_assets.su ./Drivers/SSD1306_oled/ssd1306_ssd1315.cyclo ./Drivers/SSD1306_oled/ssd1306_ssd1315.d ./Drivers/SSD1306_oled/ssd1306_ssd1315.o ./Drivers/SSD1306_oled/ssd1306_ssd1315.su

.PHONY: clean-Drivers-2f-SSD1306_oled

