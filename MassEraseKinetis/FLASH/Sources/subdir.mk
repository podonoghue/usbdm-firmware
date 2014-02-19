################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/BDMCommon.c" \
"../Sources/Clock.c" \
"../Sources/SPI.c" \
"../Sources/SWD.c" \
"../Sources/main.c" \

C_SRCS += \
../Sources/BDMCommon.c \
../Sources/Clock.c \
../Sources/SPI.c \
../Sources/SWD.c \
../Sources/main.c \

OBJS += \
./Sources/BDMCommon.o \
./Sources/Clock.o \
./Sources/SPI.o \
./Sources/SWD.o \
./Sources/main.o \

C_DEPS += \
./Sources/BDMCommon.d \
./Sources/Clock.d \
./Sources/SPI.d \
./Sources/SWD.d \
./Sources/main.d \

OBJS_QUOTED += \
"./Sources/BDMCommon.o" \
"./Sources/Clock.o" \
"./Sources/SPI.o" \
"./Sources/SWD.o" \
"./Sources/main.o" \

C_DEPS_QUOTED += \
"./Sources/BDMCommon.d" \
"./Sources/Clock.d" \
"./Sources/SPI.d" \
"./Sources/SWD.d" \
"./Sources/main.d" \

OBJS_OS_FORMAT += \
./Sources/BDMCommon.o \
./Sources/Clock.o \
./Sources/SPI.o \
./Sources/SWD.o \
./Sources/main.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/BDMCommon.o: ../Sources/BDMCommon.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/BDMCommon.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/BDMCommon.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/Clock.o: ../Sources/Clock.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/Clock.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/Clock.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/SPI.o: ../Sources/SPI.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/SPI.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/SPI.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/SWD.o: ../Sources/SWD.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/SWD.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/SWD.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/main.o: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/main.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/main.o"
	@echo 'Finished building: $<'
	@echo ' '


