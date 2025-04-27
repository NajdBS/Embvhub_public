################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Modbus_RTU_Slave_Master_With_IHM_EEPROM
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Modbus_RTU_Slave_Master_With_IHM_EEPROM.elf \

MAP_FILES += \
Modbus_RTU_Slave_Master_With_IHM_EEPROM.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Modbus_RTU_Slave_Master_With_IHM_EEPROM.list \


# All Target
all: main-build

# Main-build Target
main-build: Modbus_RTU_Slave_Master_With_IHM_EEPROM.elf secondary-outputs

# Tool invocations
Modbus_RTU_Slave_Master_With_IHM_EEPROM.elf Modbus_RTU_Slave_Master_With_IHM_EEPROM.map: $(OBJS) $(USER_OBJS) C:\Users\najd\Downloads\Compressed\MODBUS\ STM32F4_2\IHM_Slave_Master_EEPROM_Last\ 12-15-2024\V2\IHM_Slave_Master\STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Modbus_RTU_Slave_Master_With_IHM_EEPROM.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\najd\Downloads\Compressed\MODBUS STM32F4_2\IHM_Slave_Master_EEPROM_Last 12-15-2024\V2\IHM_Slave_Master\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Modbus_RTU_Slave_Master_With_IHM_EEPROM.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Modbus_RTU_Slave_Master_With_IHM_EEPROM.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Modbus_RTU_Slave_Master_With_IHM_EEPROM.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Modbus_RTU_Slave_Master_With_IHM_EEPROM.elf Modbus_RTU_Slave_Master_With_IHM_EEPROM.list Modbus_RTU_Slave_Master_With_IHM_EEPROM.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
