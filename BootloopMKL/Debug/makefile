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
BootloopMKL.map \

ARCHIVES += \
libUSER_OBJS.a \

SIZE_FILE_OUTPUTS += \
BootloopMKL.size \

SYMBOL_LIST_FILE_OUTPUTS += \
BootloopMKL.sym \


# All Target
all: BootloopMKL.elf secondary-outputs

# Tool invocations
BootloopMKL.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m0plus -mthumb -g3 -O0 -Wl,--gc-sections -L"C:/Users/Peter/Documents/Development/Git/usbdm-firmware/BootloopMKL/Project_Settings/Linker_Files" --specs=nano.specs -T "Linker-rom.ld" -Xlinker -Map="BootloopMKL.map"  -o "BootloopMKL.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

BootloopMKL.map: BootloopMKL.elf $(OBJS) $(USER_OBJS)

libUSER_OBJS.a: $(OBJS) $(USER_OBJS)
	@echo 'Invoking: ARM Archiver'
	arm-none-eabi-ar -r   "libUSER_OBJS.a" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building: $@'
	@echo ' '

BootloopMKL.size: BootloopMKL.elf
	@echo 'Invoking: ARM Sizer'
	arm-none-eabi-size "BootloopMKL.elf" --format=sysv --radix=10   >"BootloopMKL.size"
	@echo 'Finished building: $@'
	@echo ' '

BootloopMKL.sym: BootloopMKL.elf
	@echo 'Invoking: ARM Symbol Table'
	arm-none-eabi-nm "BootloopMKL.elf" --demangle --defined-only --extern-only --numeric-sort   >"BootloopMKL.sym"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_FILE_OUTPUTS)$(C_UPPER_DEPS)$(ARCHIVES)$(MAP_FILE_OUTPUTS)$(C_DEPS)$(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(LIST_FILE_OUTPUTS)$(OBJS)$(CXX_DEPS)$(SYMBOL_LIST_FILE_OUTPUTS)$(CPP_DEPS) BootloopMKL.elf
	-@echo ' '

secondary-outputs: $(LIST_FILE_OUTPUTS) $(SIZE_FILE_OUTPUTS) $(SYMBOL_LIST_FILE_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
