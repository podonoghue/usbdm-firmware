################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/leds.c \
../Sources/main.c 

C_DEPS += \
./Sources/leds.d \
./Sources/main.d 

OBJS += \
./Sources/leds.o \
./Sources/main.o 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -g3 -O0 -ffunction-sections -fdata-sections -I"C:/Users/podonoghue/Documents/Development/USBDM/usbdm-eclipse-makefiles-build/KL05_Disable_SWD/Project_Headers" -Wall -std=c99 -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -o "$@" $<
	@echo 'Finished building: $<'
	@echo ' '


