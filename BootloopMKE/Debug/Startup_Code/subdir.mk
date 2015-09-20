################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Startup_Code/newlib_stubs.c \
../Startup_Code/system.c \
../Startup_Code/vectors.c 

S_UPPER_SRCS += \
../Startup_Code/startup_ARMLtdGCC.S 

OBJS += \
./Startup_Code/newlib_stubs.o \
./Startup_Code/startup_ARMLtdGCC.o \
./Startup_Code/system.o \
./Startup_Code/vectors.o 

C_DEPS += \
./Startup_Code/newlib_stubs.d \
./Startup_Code/system.d \
./Startup_Code/vectors.d 


# Each subdirectory must supply rules for building sources it contributes
Startup_Code/%.o: ../Startup_Code/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -g3 -O0 -ffunction-sections -fdata-sections -I"C:/Users/Peter/Documents/Development/Git/usbdm-firmware/BootloopMKE/Project_Headers" -Wall -std=c99 -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -o "$@" $<
	@echo 'Finished building: $<'
	@echo ' '

Startup_Code/%.o: ../Startup_Code/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -g3 -O0 -ffunction-sections -fdata-sections -x assembler-with-cpp -I"C:/Users/Peter/Documents/Development/Git/usbdm-firmware/BootloopMKE/Project_Headers" -Wall -Wextra -c -fmessage-length=0  -o "$@" $<
	@echo 'Finished building: $<'
	@echo ' '


