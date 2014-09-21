################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
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
hello_avr.lss \

FLASH_IMAGE += \
hello_avr.hex \

EEPROM_IMAGE += \
hello_avr.eep \

SIZEDUMMY += \
sizedummy \

AVRDUDEDUMMY += \
avrdudedummy \


# All Target
all: hello_avr.elf secondary-outputs

# Tool invocations
hello_avr.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,hello_avr.map -mmcu=atmega16 -o "hello_avr.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

hello_avr.lss: hello_avr.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S hello_avr.elf  >"hello_avr.lss"
	@echo 'Finished building: $@'
	@echo ' '

hello_avr.hex: hello_avr.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex hello_avr.elf  "hello_avr.hex"
	@echo 'Finished building: $@'
	@echo ' '

hello_avr.eep: hello_avr.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex hello_avr.elf  "hello_avr.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: hello_avr.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega16 hello_avr.elf
	@echo 'Finished building: $@'
	@echo ' '

avrdudedummy: hello_avr.elf
	@echo 'Invoking: AVRDude'
	/usr/bin/avrdude -pm16 -Uflash:w:hello_avr.hex:a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(AVRDUDEDUMMY)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) hello_avr.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY) $(AVRDUDEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
