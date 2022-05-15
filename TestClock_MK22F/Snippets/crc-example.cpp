/**
 ============================================================================
 * @file    crc-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo using CRC class
 *
 *  Created on: 5 Jul 2018
 *      Author: podonoghue
 ============================================================================
 */
#include <stdlib.h>  // srand(), rand()
#include <time.h>    // time()

#include "hardware.h"
#include "crc.h"

using namespace USBDM;

using Crc = Crc0;

/** Size of data array to use */
static constexpr uint8_t ArraySize = 100;

/** Data for CRC calculation */
static uint8_t data[ArraySize];

int main(){

   /*
    * Simple tests
    * Verify against https://crccalc.com
    */
   console.setWidth(8).setPadding(Padding_LeadingZeroes);

   static const uint8_t simpleData[] = "123456789";

   Crc0::configure_Crc32();
   console.writeln("Crc32(", (const char *)simpleData, "          => ", Crc0::calculateCrc(simpleData, sizeof(simpleData)-1), Radix_16);

   Crc0::configure_Crc32_BZIP();
   console.writeln("Crc32_BZIP(", (const char *)simpleData, "     => ", Crc0::calculateCrc(simpleData, sizeof(simpleData)-1), Radix_16);

   Crc0::configure_Crc32_C();
   console.writeln("Crc32_C(", (const char *)simpleData, "        => ", Crc0::calculateCrc(simpleData, sizeof(simpleData)-1), Radix_16);

   Crc0::configure_Crc32_D();
   console.writeln("Crc32_D(", (const char *)simpleData, "        => ", Crc0::calculateCrc(simpleData, sizeof(simpleData)-1), Radix_16);

   Crc0::configure_Crc32_MPEG_2();
   console.writeln("Crc32_MPEG_2(", (const char *)simpleData, "   => ", Crc0::calculateCrc(simpleData, sizeof(simpleData)-1), Radix_16);

   Crc0::configure_Crc32_POSIX();
   console.writeln("Crc32_POSIX(", (const char *)simpleData, "    => ", Crc0::calculateCrc(simpleData, sizeof(simpleData)-1), Radix_16);


   // Fill source with random data
   srand (time(0));
   for (unsigned index=0; index<sizeof(data); index++) {
      data[index] = (uint8_t)rand();
   }

   console.writeln("\nStarting");

   // Print data
   console.writeArray(data, sizeof(data)/sizeof(data[0]));

   // Configure CRC-32
   Crc::configure_Crc32();

   // Write data to CRC unit i.e. calculate CRC
   for(unsigned index=0; index<sizeof(data); index++) {
      Crc::writeData8(data[index]);
   }

   // Report result
   console.write("Calculated CRC = 0x", Crc::getCalculatedCrc(), Radix_16);

   for(;;) {
      __asm__("nop");
   }
   return 0;
}
