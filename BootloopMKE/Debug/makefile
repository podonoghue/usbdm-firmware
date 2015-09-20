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
BootloopMKE.map \

ARCHIVES += \
libUSER_OBJS.a \

SIZE_FILE_OUTPUTS += \
BootloopMKE.size \

SYMBOL_LIST_FILE_OUTPUTS += \
BootloopMKE.sym \


# All Target
all: BootloopMKE.elf

# Tool invocations
BootloopMKE.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -g3 -O0 -Wl,--gc-sections -L"C:/Users/Peter/Documents/Development/Git/usbdm-firmware/BootloopMKE/Project_Settings/Linker_Files" --specs=nano.specs -T "Linker-rom.ld" -Xlinker -Map="BootloopMKE.map"  -o "BootloopMKE.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

BootloopMKE.map: BootloopMKE.elf $(OBJS) $(USER_OBJS)

libUSER_OBJS.a: $(OBJS) $(USER_OBJS)
	@echo 'Invoking: ARM Archiver'
	arm-none-eabi-ar -r   "libUSER_OBJS.a" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building: $@'
	@echo ' '

BootloopMKE.size: BootloopMKE.elf
	@echo 'Invoking: ARM Sizer'
	arm-none-eabi-size "BootloopMKE.elf" --format=sysv --radix=10   >"BootloopMKE.size"
	@echo 'Finished building: $@'
	@echo ' '

BootloopMKE.sym: BootloopMKE.elf
	@echo 'Invoking: ARM Symbol Table'
	arm-none-eabi-nm "BootloopMKE.elf" --demangle --defined-only --extern-only --numeric-sort   >"BootloopMKE.sym"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_FILE_OUTPUTS)$(EXECUTABLES)$(LIST_FILE_OUTPUTS)$(OBJS)$(ARCHIVES)$(MAP_FILE_OUTPUTS)$(SYMBOL_LIST_FILE_OUTPUTS)$(C_DEPS) BootloopMKE.elf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
