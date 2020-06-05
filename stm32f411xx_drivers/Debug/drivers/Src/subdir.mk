################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f411xx_gpio_driver.c \
../drivers/Src/stm32f411xx_i2c_driver.c \
../drivers/Src/stm32f411xx_spi_driver.c 

OBJS += \
./drivers/Src/stm32f411xx_gpio_driver.o \
./drivers/Src/stm32f411xx_i2c_driver.o \
./drivers/Src/stm32f411xx_spi_driver.o 

C_DEPS += \
./drivers/Src/stm32f411xx_gpio_driver.d \
./drivers/Src/stm32f411xx_i2c_driver.d \
./drivers/Src/stm32f411xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32f411xx_gpio_driver.o: ../drivers/Src/stm32f411xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F411VETx -DSTM32 -DSTM32F4 -DDEBUG -c -I../Inc -I"X:/Users/Dan/STM32CubeIDE/workspace_1.3.0/stm32f411xx_drivers2/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f411xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f411xx_i2c_driver.o: ../drivers/Src/stm32f411xx_i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F411VETx -DSTM32 -DSTM32F4 -DDEBUG -c -I../Inc -I"X:/Users/Dan/STM32CubeIDE/workspace_1.3.0/stm32f411xx_drivers2/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f411xx_i2c_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f411xx_spi_driver.o: ../drivers/Src/stm32f411xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F411VETx -DSTM32 -DSTM32F4 -DDEBUG -c -I../Inc -I"X:/Users/Dan/STM32CubeIDE/workspace_1.3.0/stm32f411xx_drivers2/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f411xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

