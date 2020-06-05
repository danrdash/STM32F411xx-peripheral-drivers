################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Startup/startup_stm32f411vetx.s 

OBJS += \
./Startup/startup_stm32f411vetx.o 


# Each subdirectory must supply rules for building sources it contributes
Startup/%.o: ../Startup/%.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I"X:/Users/Dan/STM32CubeIDE/workspace_1.3.0/stm32f411xx_drivers2/drivers/Inc" -x assembler-with-cpp --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

