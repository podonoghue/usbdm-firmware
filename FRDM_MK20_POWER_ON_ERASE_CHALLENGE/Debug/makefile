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
FRDM_MK20_POWER_ON_ERASE_CHALLENGE.map \

ARCHIVES += \
libUSER_OBJS.a \

SIZE_FILE_OUTPUTS += \
FRDM_MK20_POWER_ON_ERASE_CHALLENGE.size \

SYMBOL_LIST_FILE_OUTPUTS += \
FRDM_MK20_POWER_ON_ERASE_CHALLENGE.sym \


# All Target
all: FRDM_MK20_POWER_ON_ERASE_CHALLENGE.elf secondary-outputs

# Tool invocations
FRDM_MK20_POWER_ON_ERASE_CHALLENGE.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -g3 -O0 -Wl,--gc-sections -L"C:/Users/Peter/Documents/Development/Git/usbdm-firmware/FRDM_MK20_POWER_ON_ERASE_CHALLENGE/Project_Settings/Linker_Files" --specs=nano.specs -T ""Linker-rom.ld"" -Xlinker -Map="FRDM_MK20_POWER_ON_ERASE_CHALLENGE.map"  -o "FRDM_MK20_POWER_ON_ERASE_CHALLENGE.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

FRDM_MK20_POWER_ON_ERASE_CHALLENGE.map: FRDM_MK20_POWER_ON_ERASE_CHALLENGE.elf $(OBJS) $(USER_OBJS)

libUSER_OBJS.a: $(OBJS) $(USER_OBJS)
	@echo 'Invoking: ARM Archiver'
	arm-none-eabi-ar -r   "libUSER_OBJS.a" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building: $@'
	@echo ' '

FRDM_MK20_POWER_ON_ERASE_CHALLENGE.size: FRDM_MK20_POWER_ON_ERASE_CHALLENGE.elf
	@echo 'Invoking: ARM Sizer'
	arm-none-eabi-size "FRDM_MK20_POWER_ON_ERASE_CHALLENGE.elf" --format=sysv --radix=10   >"FRDM_MK20_POWER_ON_ERASE_CHALLENGE.size"
	@echo 'Finished building: $@'
	@echo ' '

FRDM_MK20_POWER_ON_ERASE_CHALLENGE.sym: FRDM_MK20_POWER_ON_ERASE_CHALLENGE.elf
	@echo 'Invoking: ARM Symbol Table'
	arm-none-eabi-nm "FRDM_MK20_POWER_ON_ERASE_CHALLENGE.elf" --demangle --defined-only --extern-only --numeric-sort   >"FRDM_MK20_POWER_ON_ERASE_CHALLENGE.sym"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_FILE_OUTPUTS)$(C_UPPER_DEPS)$(ARCHIVES)$(MAP_FILE_OUTPUTS)$(C_DEPS)$(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(LIST_FILE_OUTPUTS)$(OBJS)$(CXX_DEPS)$(SYMBOL_LIST_FILE_OUTPUTS)$(CPP_DEPS) FRDM_MK20_POWER_ON_ERASE_CHALLENGE.elf
	-@echo ' '

secondary-outputs: $(LIST_FILE_OUTPUTS) $(SIZE_FILE_OUTPUTS) $(SYMBOL_LIST_FILE_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
