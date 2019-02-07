@echo off
cls

set HOME_DIR=%~dp0

rem ***  Paths to various things
set VERSION_NUMBER=4
set VERSION_MINOR_NUMBER=10
set VERSION_SUFFIX=_V%VERSION_NUMBER%
set VERSION_MINOR_SUFFIX=_V%VERSION_NUMBER%_%VERSION_MINOR_NUMBER%

set USBDM_ROOT=C:\Users\podonoghue\Documents\Development\USBDM

set ECLIPSE_ROOT=%USBDM_ROOT%\usbdm-eclipse-makefiles-build

set CW_Kinetis_ROOT=%HOME_DIR%
set FLASH_IMAGES=%ECLIPSE_ROOT%\PackageFiles\FlashImages
set MK_FLASH_DIR="%FLASH_IMAGES%\MKxxNew"
set MK_FLASH_REFERENCE="%MK_FLASH_DIR%\Reference"

echo ***  ****************************************
echo ***  Do Firmware files
echo ***  ****************************************

if not exist "%FLASH_IMAGES%"       mkdir "%FLASH_IMAGES%"
if not exist "%MK_FLASH_DIR%"       mkdir "%MK_FLASH_DIR%"
if not exist "%MK_FLASH_REFERENCE%" mkdir "%MK_FLASH_REFERENCE%"

echo ***  MKxx...
set FIRMWARE_VERSIONS=OpenSDAv1 OpenSDAv1_Unique_ID OpenSDAv1_Power OpenSDAv1_Unique_ID_Power
for %%f in (%FIRMWARE_VERSIONS%) do copy "%CW_Kinetis_ROOT%\%%f\*.sx"        "%MK_FLASH_REFERENCE%"

set FIRMWARE_VERSIONS=OpenSDAv2_0 OpenSDAv2_0_Power OpenSDAv2_0_Unique_ID OpenSDAv2_0_Unique_ID_Power OpenSDAv2_1_Power OpenSDAv2_1_Unique_ID_Power
for %%f in (%FIRMWARE_VERSIONS%) do copy /B "%CW_Kinetis_ROOT%\%%f\*.bin"        "%MK_FLASH_REFERENCE%"

REM OpenSDA V1 (P&E)
set BOARD=FRDM-KE02Z FRDM-KE02Z40M FRDM-KE04Z FRDM-KE06Z FRDM-KL02Z FRDM-KL03Z FRDM-KL05Z FRDM-KL25Z FRDM-KL25Z
set BOARD=%BOARD% FRDM-KL26Z FRDM-KL27Z FRDM-KL43Z FRDM-KL46Z FRDM-K20D50M
for %%f in (%BOARD%) do copy "%CW_Kinetis_ROOT%\OpenSDAv1\*.sx"            %MK_FLASH_DIR%\%%f.sx
for %%f in (%BOARD%) do copy "%CW_Kinetis_ROOT%\OpenSDAv1_Unique_ID\*.sx"  %MK_FLASH_DIR%\%%f_Unique_ID.sx

REM OpenSDA V1 + Power (P&E)
set BOARD=FRDM-KEAZ64Q64 FRDM-KEAZN32Q64 FRDM-KEAZ128Q80 S32K144EVM
for %%f in (%BOARD%) do copy "%CW_Kinetis_ROOT%\OpenSDAv1_Power\*.sx"            %MK_FLASH_DIR%\%%f.sx
for %%f in (%BOARD%) do copy "%CW_Kinetis_ROOT%\OpenSDAv1_Unique_ID_Power\*.sx"  %MK_FLASH_DIR%\%%f_Unique_ID.sx

REM OpenSDA V2
set BOARD=FRDM-KL82Z
for %%f in (%BOARD%) do copy /B "%CW_Kinetis_ROOT%\OpenSDAv2_0\*.bin"            %MK_FLASH_DIR%\%%f.bin
for %%f in (%BOARD%) do copy /B "%CW_Kinetis_ROOT%\OpenSDAv2_0_Unique_ID\*.bin"  %MK_FLASH_DIR%\%%f_Unique_ID.bin

REM OpenSDA V2 + Power
set BOARD=FRDM-K64F
for %%f in (%BOARD%) do copy /B "%CW_Kinetis_ROOT%\OpenSDAv2_0_Power\*.bin"            %MK_FLASH_DIR%\%%f.bin
for %%f in (%BOARD%) do copy /B "%CW_Kinetis_ROOT%\OpenSDAv2_0_Unique_ID_Power\*.bin"  %MK_FLASH_DIR%\%%f_Unique_ID.bin

REM OpenSDA V2.1 + Power
set BOARD=FRDM-K22F FRDM-K28F FRDM-K66F FRDM-K82F FRDM-KV10Z FRDM-KV31F FRDM-KE15Z 
for %%f in (%BOARD%) do copy /B "%CW_Kinetis_ROOT%\OpenSDAv2_1_Power\*.bin"            %MK_FLASH_DIR%\%%f.bin
for %%f in (%BOARD%) do copy /B "%CW_Kinetis_ROOT%\OpenSDAv2_1_Unique_ID_Power\*.bin"  %MK_FLASH_DIR%\%%f_Unique_ID.bin

goto allDone

:allDone
pause
