################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/clock.c" \
"../Sources/main.c" \

C_SRCS += \
../Sources/clock.c \
../Sources/main.c \

OBJS += \
./Sources/clock_c.obj \
./Sources/main_c.obj \

OBJS_QUOTED += \
"./Sources/clock_c.obj" \
"./Sources/main_c.obj" \

C_DEPS += \
./Sources/clock_c.d \
./Sources/main_c.d \

C_DEPS_QUOTED += \
"./Sources/clock_c.d" \
"./Sources/main_c.d" \

OBJS_OS_FORMAT += \
./Sources/clock_c.obj \
./Sources/main_c.obj \


# Each subdirectory must supply rules for building sources it contributes
Sources/clock_c.obj: ../Sources/clock.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/clock.args" -o "Sources/clock_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/%.d: ../Sources/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

Sources/main_c.obj: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Compiler'
	"$(ARM_ToolsDirEnv)/mwccarm" -gccinc @@"Sources/main.args" -o "Sources/main_c.obj" -c "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '


