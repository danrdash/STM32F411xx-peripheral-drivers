################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/011i2c_master_rx_testingIT.c 

OBJS += \
./Src/011i2c_master_rx_testingIT.o 

C_DEPS += \
./Src/011i2c_master_rx_testingIT.d 


# Each subdirectory must supply rules for building sources it contributes
Src/011i2c_master_rx_testingIT.o: ../Src/011i2c_master_rx_testingIT.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F411VETx -DSTM32 -DSTM32F4 -DDEBUG -c -I../Inc -I"X:/Users/Dan/STM32CubeIDE/workspace_1.3.0/stm32f411xx_drivers2/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/011i2c_master_rx_testingIT.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

