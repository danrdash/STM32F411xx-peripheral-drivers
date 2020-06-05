################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include drivers/Src/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
stm32f411xx_drivers2.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stm32f411xx_drivers2.list \

OBJCOPY_BIN += \
stm32f411xx_drivers2.bin \


# All Target
all: stm32f411xx_drivers2.elf secondary-outputs

# Tool invocations
stm32f411xx_drivers2.elf: $(OBJS) $(USER_OBJS) X:\Users\Dan\STM32CubeIDE\workspace_1.3.0\stm32f411xx_drivers2\STM32F411VETX_FLASH.ld
	arm-none-eabi-gcc -o "stm32f411xx_drivers2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"X:\Users\Dan\STM32CubeIDE\workspace_1.3.0\stm32f411xx_drivers2\STM32F411VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stm32f411xx_drivers2.map" -Wl,--gc-sections -static -specs=rdimon.specs -lc -lrdimon --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stm32f411xx_drivers2.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stm32f411xx_drivers2.list"
	@echo 'Finished building: $@'
	@echo ' '

stm32f411xx_drivers2.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "stm32f411xx_drivers2.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
