/**
 ============================================================================
 * @file    rnga-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo using rnga class
 *
 *  Created on: 5 Jul 2018
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "rnga.h"

using namespace USBDM;

// RNGA underflow handler
void callback() {
   console.writeln("Rnga underflow occurred");
   Rnga::clearInterruptFlag();
}

int main(){

   console.writeln("\nStarting");

   // Configure Rnga
   Rnga::configure(RngaHighAssurance_Enabled);

   Rnga::setCallback(callback);
   Rnga::enableInterrupt();
   Rnga::enableNvicInterrupts(NvicPriority_Normal);

#if 0
   // Used to test underflow handler
   for(;;) {
      if (Rnga::getRandomValue() == 0) {
         console.writeln("Rnga failed to provide number");
      }
   }
#endif
   console.setPadding(Padding_LeadingZeroes).setWidth(8);
   for(;;) {
      // Report result
      console.write("Calculated Random No = 0x").writeln(Rnga::getSafeRandomValue(), Radix_16);
   }
   return 0;
}
