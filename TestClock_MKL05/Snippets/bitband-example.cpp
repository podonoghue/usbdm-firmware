/**
 ==================================================================================
 * @file    bitband-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Demonstration of use of ARM Cortex-M4 bitband region for variables
 *
 *  Created on: 7/1/2022
 *      Author: podonoghue
 *
 * @note It is necessary to allocate some space to bit-band use in configure.usbdm
 ==================================================================================
 */
#include "hardware.h"

using namespace USBDM;

/**
 * Allocate a pair of variables
 * - The first is located in .bitband_ram and is a conventional variable in RAM
 * - The second is located in .bitband and is a word->bit mapping to the first variable
 */
// Variable allocated in bit-band accessible region (.bitband_ram)
// May be uint8_t, uint16_t or uint32_t
__attribute__ ((section(".bitband_ram")))
uint8_t variable;

// Each word of 'variable_bits' maps to an individual bit of 'variable' in bit-band region (.bitband)
__attribute__ ((section(".bitband")))
uint32_t variable_bits[8*sizeof(variable)];

int main() {
   console.writeln("Starting\n");

   // Note that variable is NOT initialised before entering main()
   variable = 0;

   // Set each bit in variable from LSB to MSB by writing '1' to variable_bits
   console.writeln("\nSetting each bit");
   for (unsigned bitNum=0; bitNum<sizeofArray(variable_bits); bitNum++) {
      variable_bits[bitNum] = 1;
      console.writeln("variable = 0b", variable, Radix_2);
   }

   // Clear each bit in variable from LSB to MSB by writing '0' to variable_bits
   console.writeln("\nClearing each bit");
   for (unsigned bitNum=0; bitNum<sizeofArray(variable_bits); bitNum++) {
      variable_bits[bitNum] = 0;
      console.writeln("variable = 0b", variable, Radix_2);
   }
   for(int count = 0;;count++) {
      __asm__("bkpt");
   }
   return 0;
}
