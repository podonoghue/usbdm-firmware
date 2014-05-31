################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/ARM.c" \
"../Sources/CmdProcessingARM.c" \
"../Sources/CmdProcessingSWD.c" \
"../Sources/DummyUserVectorTable.c" \
"../Sources/ICP.c" \
"../Sources/ICP_USB.c" \
"../Sources/SPI.c" \
"../Sources/SWD.c" \

C_SRCS += \
../Sources/ARM.c \
../Sources/CmdProcessingARM.c \
../Sources/CmdProcessingSWD.c \
../Sources/DummyUserVectorTable.c \
../Sources/ICP.c \
../Sources/ICP_USB.c \
../Sources/SPI.c \
../Sources/SWD.c \

OBJS += \
./Sources/ARM_c.obj \
./Sources/CmdProcessingARM_c.obj \
./Sources/CmdProcessingSWD_c.obj \
./Sources/DummyUserVectorTable_c.obj \
./Sources/ICP_c.obj \
./Sources/ICP_USB_c.obj \
./Sources/SPI_c.obj \
./Sources/SWD_c.obj \

OBJS_QUOTED += \
"./Sources/ARM_c.obj" \
"./Sources/CmdProcessingARM_c.obj" \
"./Sources/CmdProcessingSWD_c.obj" \
"./Sources/DummyUserVectorTable_c.obj" \
"./Sources/ICP_c.obj" \
"./Sources/ICP_USB_c.obj" \
"./Sources/SPI_c.obj" \
"./Sources/SWD_c.obj" \

C_DEPS += \
./Sources/ARM_c.d \
./Sources/CmdProcessingARM_c.d \
./Sources/CmdProcessingSWD_c.d \
./Sources/DummyUserVectorTable_c.d \
./Sources/ICP_c.d \
./Sources/ICP_USB_c.d \
./Sources/SPI_c.d \
./Sources/SWD_c.d \

C_DEPS_QUOTED += \
"./Sources/ARM_c.d" \
"./Sources/CmdProcessingARM_c.d" \
"./Sources/CmdProcessingSWD_c.d" \
"./Sources/DummyUserVectorTable_c.d" \
"./Sources/ICP_c.d" \
"./Sources/ICP_USB_c.d" \
"./Sources/SPI_c.d" \
"./Sources/SWD_c.d" \

OBJS_OS_FORMAT += \
./Sources/ARM_c.obj \
./Sources/CmdProcessingARM_c.obj \
./Sources/CmdProcessingSWD_c.obj \
./Sources/DummyUserVectorTable_c.obj \
./Sources/ICP_c.obj \
./Sources/ICP_USB_c.obj \
./Sources/SPI_c.obj \
./Sources/SWD_c.obj \


# Each subdirectory must supply rules for building sources it contributes
Sources/ARM_c.obj: ../Sources/ARM.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/ARM.args" -ObjN="Sources/ARM_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/%.d: ../Sources/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

Sources/CmdProcessingARM_c.obj: ../Sources/CmdProcessingARM.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/CmdProcessingARM.args" -ObjN="Sources/CmdProcessingARM_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/CmdProcessingSWD_c.obj: ../Sources/CmdProcessingSWD.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/CmdProcessingSWD.args" -ObjN="Sources/CmdProcessingSWD_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/DummyUserVectorTable_c.obj: ../Sources/DummyUserVectorTable.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/DummyUserVectorTable.args" -ObjN="Sources/DummyUserVectorTable_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/ICP_c.obj: ../Sources/ICP.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/ICP.args" -ObjN="Sources/ICP_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/ICP_USB_c.obj: ../Sources/ICP_USB.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/ICP_USB.args" -ObjN="Sources/ICP_USB_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/SPI_c.obj: ../Sources/SPI.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/SPI.args" -ObjN="Sources/SPI_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/SWD_c.obj: ../Sources/SWD.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/SWD.args" -ObjN="Sources/SWD_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '


