@echo off
cls

rem ***  Paths to various things
set VERSION_NUMBER=4
set VERSION_MINOR_NUMBER=10
set VERSION_SUFFIX=_V%VERSION_NUMBER%
set VERSION_MINOR_SUFFIX=_V%VERSION_NUMBER%_%VERSION_MINOR_NUMBER%

set USBDM_ROOT=C:\Users\podonoghue\Documents\Development\USBDM

set ECLIPSE_ROOT=%USBDM_ROOT%\usbdm-eclipse-makefiles-build

set CW_Kinetis_ROOT=%ECLIPSE_ROOT%\USBDM_Kinetis_Firmware
set FLASH_DIR=%ECLIPSE_ROOT%\PackageFiles\FlashImages
set MK_FLASH_DIR="%FLASH_DIR%\MKxxNew"

echo ***  ****************************************
echo ***  Do Firmware files
echo ***  ****************************************

if not exist "%FLASH_DIR%"        mkdir "%FLASH_DIR%"
if not exist "%MK_FLASH_DIR%"     mkdir "%MK_FLASH_DIR%"

echo ***  MKxx...
set FIRMWARE_VERSIONS=OpenSDAv1 OpenSDAv1_Unique_ID
for %%f in (%FIRMWARE_VERSIONS%) do copy "%CW_Kinetis_ROOT%\%%f\*.sx"        "%MK_FLASH_DIR%"
set FIRMWARE_VERSIONS=OpenSDAv2_0 OpenSDAv2_0_Unique_ID OpenSDAv2_1 OpenSDAv2_1_Unique_ID
for %%f in (%FIRMWARE_VERSIONS%) do copy "%CW_Kinetis_ROOT%\%%f\*.bin"        "%MK_FLASH_DIR%"

goto allDone

:allDone
pause
