################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Project_Settings/Startup_Code/start08.c" \

C_SRCS += \
../Project_Settings/Startup_Code/start08.c \

OBJS += \
./Project_Settings/Startup_Code/start08_c.obj \

OBJS_QUOTED += \
"./Project_Settings/Startup_Code/start08_c.obj" \

C_DEPS += \
./Project_Settings/Startup_Code/start08_c.d \

C_DEPS_QUOTED += \
"./Project_Settings/Startup_Code/start08_c.d" \

OBJS_OS_FORMAT += \
./Project_Settings/Startup_Code/start08_c.obj \


# Each subdirectory must supply rules for building sources it contributes
Project_Settings/Startup_Code/start08_c.obj: ../Project_Settings/Startup_Code/start08.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Project_Settings/Startup_Code/start08.args" -ObjN="Project_Settings/Startup_Code/start08_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Project_Settings/Startup_Code/%.d: ../Project_Settings/Startup_Code/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '


