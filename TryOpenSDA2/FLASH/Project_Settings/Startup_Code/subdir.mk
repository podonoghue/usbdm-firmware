################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Project_Settings/Startup_Code/kinetis_sysinit.c" \

C_SRCS += \
../Project_Settings/Startup_Code/kinetis_sysinit.c \

OBJS += \
./Project_Settings/Startup_Code/kinetis_sysinit_c.obj \

OBJS_QUOTED += \
"./Project_Settings/Startup_Code/kinetis_sysinit_c.obj" \

C_DEPS += \
./Project_Settings/Startup_Code/kinetis_sysinit_c.d \

C_DEPS_QUOTED += \
"./Project_Settings/Startup_Code/kinetis_sysinit_c.d" \

OBJS_OS_FORMAT += \
./Project_Settings/Startup_Code/kinetis_sysinit_c.obj \


# Each subdirectory must supply rules for building sources it contributes
Project_Settings/Startup_Code/kinetis_sysinit_c.obj: ../Project_Settings/Startup_Code/kinetis_sysinit.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Project_Settings/Startup_Code/kinetis_sysinit.args" -o "Project_Settings/Startup_Code/kinetis_sysinit_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Project_Settings/Startup_Code/%.d: ../Project_Settings/Startup_Code/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '


