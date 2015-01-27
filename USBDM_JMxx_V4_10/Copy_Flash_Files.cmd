@echo off
cls
rem ***  Paths to various things
set VERSION_NUMBER=4
set VERSION_MINOR_NUMBER=10
set VERSION_SUFFIX=_V%VERSION_NUMBER%
set VERSION_MINOR_SUFFIX=_V%VERSION_NUMBER%_%VERSION_MINOR_NUMBER%

set USBDM_ROOT=C:\Users\podonoghue\Development\USBDM

rem where to find 'fixed' stuff
set PACKAGE_FILES=%USBDM_ROOT%\usbdm-eclipse-makefiles-build\PackageFiles

set FLASH_DIR=.\FlashImages

echo ***  ****************************************
echo ***  Do Firmware files
echo ***  ****************************************

if not exist "%FLASH_DIR%"        mkdir "%FLASH_DIR%"
if not exist "%FLASH_DIR%\JS16"   mkdir "%FLASH_DIR%\JS16"
if not exist "%FLASH_DIR%\JMxx"   mkdir "%FLASH_DIR%\JMxx"
if not exist "%FLASH_DIR%\Tower"  mkdir "%FLASH_DIR%\Tower"

echo ***  JS16...
set FIRMWARE_VERSIONS=USBDM_CF_JS16CWJ USBDM_JS16CWJ USBDM_CF_SER_JS16CWJ USBDM_SER_JS16CWJ USBDM_SWD_SER_JS16CWJ USBDM_SWD_JS16CWJ
for %%f in (%FIRMWARE_VERSIONS%) do copy .\%%f\USBDM*.sx      "%FLASH_DIR%\JS16"
echo ***  JMxx...
set FIRMWARE_VERSIONS=USBDM_CF_JMxxCLD USBDM_CF_SER_JMxxCLD USBDM_JMxxCLC USBDM_JMxxCLD USBDM_JMxx_MC56F8006Demo
for %%f in (%FIRMWARE_VERSIONS%) do copy .\%%f\USBDM*.sx      "%FLASH_DIR%\JMxx"
echo ***  TWR...
set FIRMWARE_VERSIONS=USBDM_TWR_CFV1 USBDM_TWR_HCS08 USBDM_TWR_Kinetis USBDM_TWR_CFVx USBDM_TWR_HCS12
for %%f in (%FIRMWARE_VERSIONS%) do copy .\%%f\USBDM*.sx      "%FLASH_DIR%\Tower"

goto allDone

:allDone
pause
