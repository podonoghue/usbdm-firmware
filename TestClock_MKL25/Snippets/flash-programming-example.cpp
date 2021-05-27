/**
 ====================================================================================================
 * @file    flash-programming-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo of Flash programming interface
 *
 * This example erases and then programs a range of flash memory.
 *
 * @note Alignment requirements
 * - The minimum erase element for the flash is a sector (usually 1, 2 or 4 K bytes).\n
 *   This requires erasing to be done on a sector boundary and be a multiple of the sector size.
 * - The minimum program element size is a phrase (usually 4 or 8 bytes).\n
 *   This requires programming to be done on a phrase boundary and be a multiple of the phrase size.
 *
 *  Created on: 14/6/2018
 *      Author: podonoghue
 ====================================================================================================
 */
#include <stdlib.h>
#include <time.h>       /* time */
#include "flash.h"

using namespace USBDM;

/*
 * Uncomment to use Data flash rather that Program flash region
 * Not all devices have Data flash e.g. MKxxFX does, MKxxFN doesn't
 * Note - This is using the data flash as simple flash not EEPROM.
 *        See nonvolatile_example.cpp for FlexRAM/EEPROM example.
 */
//#define USE_DATA_FLASH

#if defined(USE_DATA_FLASH) && (defined(USBDM_FTFL_IS_DEFINED) || defined(USBDM_FTFE_IS_DEFINED))
/**
 * Data flash example - the array is located in Data Flash (FlexNVM region used as regular flash)
 */
/** Size of flash region to program - must be a multiple of flash sector size */
static constexpr unsigned ArraySize =  2 * Flash::dataFlashSectorSize;

/** Array located in data flash that will be programmed */
__attribute__ ((section(".flexNVM"), aligned(Flash::dataFlashSectorSize)))
static uint8_t copy[ArraySize];

#else
/**
 * Program flash example - the array is located in Program Flash (Regular flash region)
 */
/** Size of flash region to program - must be a multiple of flash sector size */
static constexpr unsigned ArraySize =  2 * Flash::programFlashSectorSize;

/**
 * Array located in program flash that will be programmed.
 * Must not be in same sector as code or constant data!
 */
__attribute__ ((section(".flash"), aligned(Flash::programFlashSectorSize)))
static uint8_t copy[ArraySize];
#endif

/** Data to be programmed to flash */
static uint8_t data[ArraySize];

/**
 * Print a range of memory as a hex table
 *
 * @param address Address to start at
 * @param size    Number bytes to print
 */
void printDump(uint8_t *address, uint32_t size) {
   constexpr unsigned RowWidth = 32;

   console.setPadding(Padding_LeadingZeroes).setWidth(2);
   console.write("          ");
   for (unsigned index=0; index<RowWidth; index++) {
      console.write(index).write(" ");
   }
   console.writeln();
   bool needNewline = true;
   for (unsigned index=0; index<size; index++) {
      if (needNewline) {
         console.setPadding(Padding_LeadingZeroes).setWidth(8);
         console.write((int)address+index, Radix_16).write(": ");
         console.setPadding(Padding_LeadingZeroes).setWidth(2);
      }
      console.write(address[index], Radix_16).write(" ");
      needNewline = (((index+1)&(RowWidth-1))==0);
      if (needNewline) {
         console.writeln();
      }
   }
   console.writeln().reset();
}

/**
 * Flash programming example
 */
int main(void) {
   console.writeln("Starting");

   static_assert(((sizeof(copy)&(Flash::programFlashSectorSize-1)) == 0), "Data must be correct size");
   usbdm_assert((((unsigned)copy&(Flash::programFlashSectorSize-1)) == 0), "Data must be correctly aligned");

   // Report original flash contents
   console.writeln("Flash before programming");
   printDump(copy, sizeof(copy));

   // Erase array located in flash
   FlashDriverError_t rc = Flash::eraseRange(copy, sizeof(copy));
   if (rc != USBDM::FLASH_ERR_OK) {
      console.writeln("Flash erasing failed");
      __BKPT();
   }

   // Report programmed flash contents
   console.writeln("Flash after erasing");
   printDump(copy, sizeof(copy));

   // Verify programmed data
   for (unsigned index=0; index<sizeof(copy); index++) {
      if (copy[index] != 0xFF) {
         console.
         write("Flash failed erase @").write(&copy[index]).
         write(": (data[").write(index).write("],").write(copy[index]).writeln(") != 0xFF");
         console.writeln("Verify of flash erasing failed\n\r");
         __BKPT();
      }
   }

   // Fill source with random data
   srand (time(NULL));
   for (unsigned index=0; index<sizeof(data); index++) {
      data[index] = rand()%256;
   }

   // Program data to array located in flash
   rc = Flash::programRange(data, copy, sizeof(data));
   if (rc != USBDM::FLASH_ERR_OK) {
      console.writeln("Flash programming failed\n\r");
      __BKPT();
   }

   // Report programmed flash contents
   console.writeln("Flash after programming\n\r");
   printDump(copy, sizeof(copy));

   // Verify programmed data
   for (unsigned index=0; index<sizeof(data); index++) {
      if (data[index] != copy[index]) {
         console.
         write("Flash failed verify @").write(&copy[index]).
         write(": (data[").write(index).write("],").write(data[index]).write(") != ").
         write(": (copy[").write(index).write("],").writeln(copy[index]);
         console.writeln("Verify of flash programming failed\n\r");
         __BKPT();
      }
   }

   console.writeln("Flash programming sequence completed without error\n\r");

   for(;;) {
      __asm__("bkpt");
   }
}
