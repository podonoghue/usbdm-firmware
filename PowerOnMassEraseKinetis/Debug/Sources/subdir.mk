################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/BDMCommon.c \
../Sources/SPI.c \
../Sources/SWD.c \
../Sources/main.c 

C_DEPS += \
./Sources/BDMCommon.d \
./Sources/SPI.d \
./Sources/SWD.d \
./Sources/main.d 

OBJS += \
./Sources/BDMCommon.o \
./Sources/SPI.o \
./Sources/SWD.o \
./Sources/main.o 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -g3 -O0 -ffunction-sections -fdata-sections -DTARGET_HARDWARE=H_USBDM_OPENSDA -I"C:/Users/podonoghue/Documents/Development/USBDM/usbdm-eclipse-makefiles-build/PowerOnMassEraseKinetis/Project_Headers" -Wall -std=c99 -c -fmessage-length=0 -MT"$@" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -o "$@" $<
	@echo 'Finished building: $<'
	@echo ' '


