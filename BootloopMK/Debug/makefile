################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := usbdm-rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup_Code/subdir.mk
-include Sources/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
MAP_FILE_OUTPUTS += \
BootloopMK.map \

ARCHIVES += \
libUSER_OBJS.a \

SIZE_FILE_OUTPUTS += \
BootloopMK.size \

SYMBOL_LIST_FILE_OUTPUTS += \
BootloopMK.sym \


# All Target
all: BootloopMK.elf

# Tool invocations
BootloopMK.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -g3 -O0 -Wl,--gc-sections -L"C:/Users/Peter/Documents/Development/Git/usbdm-firmware/BootloopMK/Project_Settings/Linker_Files" --specs=nano.specs -T "Linker-rom.ld" -Xlinker -Map="BootloopMK.map"  -o "BootloopMK.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

BootloopMK.map: BootloopMK.elf $(OBJS) $(USER_OBJS)

libUSER_OBJS.a: $(OBJS) $(USER_OBJS)
	@echo 'Invoking: ARM Archiver'
	arm-none-eabi-ar -r   "libUSER_OBJS.a" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building: $@'
	@echo ' '

BootloopMK.size: BootloopMK.elf
	@echo 'Invoking: ARM Sizer'
	arm-none-eabi-size "BootloopMK.elf" --format=sysv --radix=10   >"BootloopMK.size"
	@echo 'Finished building: $@'
	@echo ' '

BootloopMK.sym: BootloopMK.elf
	@echo 'Invoking: ARM Symbol Table'
	arm-none-eabi-nm "BootloopMK.elf" --demangle --defined-only --extern-only --numeric-sort   >"BootloopMK.sym"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_FILE_OUTPUTS)$(EXECUTABLES)$(LIST_FILE_OUTPUTS)$(OBJS)$(ARCHIVES)$(MAP_FILE_OUTPUTS)$(SYMBOL_LIST_FILE_OUTPUTS)$(C_DEPS) BootloopMK.elf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
