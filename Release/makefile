################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
GR_2015.lss \

FLASH_IMAGE += \
GR_2015.hex \

EEPROM_IMAGE += \
GR_2015.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: GR_2015.elf secondary-outputs

# Tool invocations
GR_2015.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,GR_2015.map -L./../../xmega_timer/Release -mmcu=atxmega128a1 -o "GR_2015.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

GR_2015.lss: GR_2015.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S GR_2015.elf  >"GR_2015.lss"
	@echo 'Finished building: $@'
	@echo ' '

GR_2015.hex: GR_2015.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex GR_2015.elf  "GR_2015.hex"
	@echo 'Finished building: $@'
	@echo ' '

GR_2015.eep: GR_2015.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex GR_2015.elf  "GR_2015.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: GR_2015.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atxmega128a1 GR_2015.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) GR_2015.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
