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
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
MAP_FILE_OUTPUTS += \
PowerOnMassEraseKinetis.map \

ARCHIVES += \
libUSER_OBJS.a \

SIZE_FILE_OUTPUTS += \
PowerOnMassEraseKinetis.size \

SYMBOL_LIST_FILE_OUTPUTS += \
PowerOnMassEraseKinetis.sym \


# All Target
all: PowerOnMassEraseKinetis.elf secondary-outputs

# Tool invocations
PowerOnMassEraseKinetis.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -g3 -O0 -Wl,--gc-sections -L"C:/Users/podonoghue/Documents/Development/USBDM/usbdm-eclipse-makefiles-build/PowerOnMassEraseKinetis/Project_Settings/Linker_Files" --specs=nano.specs -T ""Linker-rom.ld"" -Xlinker -Map="PowerOnMassEraseKinetis.map"  -o "PowerOnMassEraseKinetis.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

PowerOnMassEraseKinetis.map: PowerOnMassEraseKinetis.elf $(OBJS) $(USER_OBJS)

libUSER_OBJS.a: $(OBJS) $(USER_OBJS)
	@echo 'Invoking: ARM Archiver'
	arm-none-eabi-ar -r   "libUSER_OBJS.a" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building: $@'
	@echo ' '

PowerOnMassEraseKinetis.size: PowerOnMassEraseKinetis.elf
	@echo 'Invoking: ARM Sizer'
	arm-none-eabi-size PowerOnMassEraseKinetis.elf --format=berkeley --radix=10   >"PowerOnMassEraseKinetis.size"
	@echo 'Finished building: $@'
	@echo ' '

PowerOnMassEraseKinetis.sym: PowerOnMassEraseKinetis.elf
	@echo 'Invoking: ARM Symbol Table'
	arm-none-eabi-nm PowerOnMassEraseKinetis.elf --demangle --defined-only --extern-only --numeric-sort   >"PowerOnMassEraseKinetis.sym"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_FILE_OUTPUTS)$(C_UPPER_DEPS)$(ARCHIVES)$(MAP_FILE_OUTPUTS)$(C_DEPS)$(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(LIST_FILE_OUTPUTS)$(OBJS)$(CXX_DEPS)$(SYMBOL_LIST_FILE_OUTPUTS)$(CPP_DEPS) PowerOnMassEraseKinetis.elf
	-@echo ' '

secondary-outputs: $(LIST_FILE_OUTPUTS) $(SIZE_FILE_OUTPUTS) $(SYMBOL_LIST_FILE_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
