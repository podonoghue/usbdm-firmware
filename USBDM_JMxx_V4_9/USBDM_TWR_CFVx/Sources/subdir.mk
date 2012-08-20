################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/BDM.c" \
"../Sources/BDMCommon.c" \
"../Sources/BDM_CF.c" \
"../Sources/BDM_RS08.c" \
"../Sources/CmdProcessing.c" \
"../Sources/CmdProcessingCFV1.c" \
"../Sources/CmdProcessingCFVx.c" \
"../Sources/CmdProcessingHCS.c" \
"../Sources/ICP.c" \
"../Sources/ICP_USB.c" \
"../Sources/JTAG.c" \
"../Sources/JTAGSequence.c" \
"../Sources/SCI.c" \
"../Sources/SCI_Debug.c" \
"../Sources/USB.c" \
"../Sources/UserVectorTable.c" \
"../Sources/main.c" \

C_SRCS += \
../Sources/BDM.c \
../Sources/BDMCommon.c \
../Sources/BDM_CF.c \
../Sources/BDM_RS08.c \
../Sources/CmdProcessing.c \
../Sources/CmdProcessingCFV1.c \
../Sources/CmdProcessingCFVx.c \
../Sources/CmdProcessingHCS.c \
../Sources/ICP.c \
../Sources/ICP_USB.c \
../Sources/JTAG.c \
../Sources/JTAGSequence.c \
../Sources/SCI.c \
../Sources/SCI_Debug.c \
../Sources/USB.c \
../Sources/UserVectorTable.c \
../Sources/main.c \

OBJS += \
./Sources/BDM_c.obj \
./Sources/BDMCommon_c.obj \
./Sources/BDM_CF_c.obj \
./Sources/BDM_RS08_c.obj \
./Sources/CmdProcessing_c.obj \
./Sources/CmdProcessingCFV1_c.obj \
./Sources/CmdProcessingCFVx_c.obj \
./Sources/CmdProcessingHCS_c.obj \
./Sources/ICP_c.obj \
./Sources/ICP_USB_c.obj \
./Sources/JTAG_c.obj \
./Sources/JTAGSequence_c.obj \
./Sources/SCI_c.obj \
./Sources/SCI_Debug_c.obj \
./Sources/USB_c.obj \
./Sources/UserVectorTable_c.obj \
./Sources/main_c.obj \

OBJS_QUOTED += \
"./Sources/BDM_c.obj" \
"./Sources/BDMCommon_c.obj" \
"./Sources/BDM_CF_c.obj" \
"./Sources/BDM_RS08_c.obj" \
"./Sources/CmdProcessing_c.obj" \
"./Sources/CmdProcessingCFV1_c.obj" \
"./Sources/CmdProcessingCFVx_c.obj" \
"./Sources/CmdProcessingHCS_c.obj" \
"./Sources/ICP_c.obj" \
"./Sources/ICP_USB_c.obj" \
"./Sources/JTAG_c.obj" \
"./Sources/JTAGSequence_c.obj" \
"./Sources/SCI_c.obj" \
"./Sources/SCI_Debug_c.obj" \
"./Sources/USB_c.obj" \
"./Sources/UserVectorTable_c.obj" \
"./Sources/main_c.obj" \

C_DEPS += \
./Sources/BDM_c.d \
./Sources/BDMCommon_c.d \
./Sources/BDM_CF_c.d \
./Sources/BDM_RS08_c.d \
./Sources/CmdProcessing_c.d \
./Sources/CmdProcessingCFV1_c.d \
./Sources/CmdProcessingCFVx_c.d \
./Sources/CmdProcessingHCS_c.d \
./Sources/ICP_c.d \
./Sources/ICP_USB_c.d \
./Sources/JTAG_c.d \
./Sources/JTAGSequence_c.d \
./Sources/SCI_c.d \
./Sources/SCI_Debug_c.d \
./Sources/USB_c.d \
./Sources/UserVectorTable_c.d \
./Sources/main_c.d \

C_DEPS_QUOTED += \
"./Sources/BDM_c.d" \
"./Sources/BDMCommon_c.d" \
"./Sources/BDM_CF_c.d" \
"./Sources/BDM_RS08_c.d" \
"./Sources/CmdProcessing_c.d" \
"./Sources/CmdProcessingCFV1_c.d" \
"./Sources/CmdProcessingCFVx_c.d" \
"./Sources/CmdProcessingHCS_c.d" \
"./Sources/ICP_c.d" \
"./Sources/ICP_USB_c.d" \
"./Sources/JTAG_c.d" \
"./Sources/JTAGSequence_c.d" \
"./Sources/SCI_c.d" \
"./Sources/SCI_Debug_c.d" \
"./Sources/USB_c.d" \
"./Sources/UserVectorTable_c.d" \
"./Sources/main_c.d" \

OBJS_OS_FORMAT += \
./Sources/BDM_c.obj \
./Sources/BDMCommon_c.obj \
./Sources/BDM_CF_c.obj \
./Sources/BDM_RS08_c.obj \
./Sources/CmdProcessing_c.obj \
./Sources/CmdProcessingCFV1_c.obj \
./Sources/CmdProcessingCFVx_c.obj \
./Sources/CmdProcessingHCS_c.obj \
./Sources/ICP_c.obj \
./Sources/ICP_USB_c.obj \
./Sources/JTAG_c.obj \
./Sources/JTAGSequence_c.obj \
./Sources/SCI_c.obj \
./Sources/SCI_Debug_c.obj \
./Sources/USB_c.obj \
./Sources/UserVectorTable_c.obj \
./Sources/main_c.obj \


# Each subdirectory must supply rules for building sources it contributes
Sources/BDM_c.obj: ../Sources/BDM.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/BDM.args" -ObjN="Sources/BDM_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/%.d: ../Sources/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

Sources/BDMCommon_c.obj: ../Sources/BDMCommon.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/BDMCommon.args" -ObjN="Sources/BDMCommon_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/BDM_CF_c.obj: ../Sources/BDM_CF.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/BDM_CF.args" -ObjN="Sources/BDM_CF_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/BDM_RS08_c.obj: ../Sources/BDM_RS08.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/BDM_RS08.args" -ObjN="Sources/BDM_RS08_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/CmdProcessing_c.obj: ../Sources/CmdProcessing.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/CmdProcessing.args" -ObjN="Sources/CmdProcessing_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/CmdProcessingCFV1_c.obj: ../Sources/CmdProcessingCFV1.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/CmdProcessingCFV1.args" -ObjN="Sources/CmdProcessingCFV1_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/CmdProcessingCFVx_c.obj: ../Sources/CmdProcessingCFVx.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/CmdProcessingCFVx.args" -ObjN="Sources/CmdProcessingCFVx_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/CmdProcessingHCS_c.obj: ../Sources/CmdProcessingHCS.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/CmdProcessingHCS.args" -ObjN="Sources/CmdProcessingHCS_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/ICP_c.obj: ../Sources/ICP.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/ICP.args" -ObjN="Sources/ICP_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/ICP_USB_c.obj: ../Sources/ICP_USB.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/ICP_USB.args" -ObjN="Sources/ICP_USB_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/JTAG_c.obj: ../Sources/JTAG.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/JTAG.args" -ObjN="Sources/JTAG_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/JTAGSequence_c.obj: ../Sources/JTAGSequence.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/JTAGSequence.args" -ObjN="Sources/JTAGSequence_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/SCI_c.obj: ../Sources/SCI.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/SCI.args" -ObjN="Sources/SCI_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/SCI_Debug_c.obj: ../Sources/SCI_Debug.c
	@echo 'Building file: $<'
	@echo 'Executing target #14 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/SCI_Debug.args" -ObjN="Sources/SCI_Debug_c.obj" "$<" -Lm="Sources/SCI_Debug_c.d" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/SCI_Debug_c.d: ../Sources/SCI_Debug.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

Sources/USB_c.obj: ../Sources/USB.c
	@echo 'Building file: $<'
	@echo 'Executing target #15 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/USB.args" -ObjN="Sources/USB_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/UserVectorTable_c.obj: ../Sources/UserVectorTable.c
	@echo 'Building file: $<'
	@echo 'Executing target #16 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/UserVectorTable.args" -ObjN="Sources/UserVectorTable_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '

Sources/main_c.obj: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #17 $<'
	@echo 'Invoking: HCS08 Compiler'
	"$(HC08ToolsEnv)/chc08" -ArgFile"Sources/main.args" -ObjN="Sources/main_c.obj" "$<" -Lm="$(@:%.obj=%.d)" -LmCfg=xilmou
	@echo 'Finished building: $<'
	@echo ' '


