/*
 * pmc-example.cpp
 *
 *  Created on: 4 Jun 2019
 *      Author: podonoghue
 */
#include "hardware.h"
#include "pmc.h"
#include "rcm.h"

using namespace USBDM;

__attribute__ ((section (".noinit")))
static bool pmcHandlerRan;

/**
 * PMC call-back.
 *
 * Executes when PMC detect Warning or Reset Low-voltage events
 */
void callback(PmcInterruptReason reason) {
   pmcHandlerRan = true;

   switch(reason) {
      case PmcInterruptReason_LowVoltageDetect:
         for(;;) {
            // Wait here until reset occurs
            __asm__("nop");
         }
         break;
      case PmcInterruptReason_LowVoltageWarning:
         console.writeln("PMC detected Low Voltage Warning level");
         break;
   }
}

int main() {
   console.writeln("\n\nStarting");

   if ((Rcm::getResetSource() & RcmSource_lvd) != 0) {
      console.writeln("========================================");
      console.writeln("Reset due to low-voltage event");
      if (pmcHandlerRan) {
         console.writeln("PMC handler ran before reset");
      }
      else {
         console.writeln("PMC handler did not run before reset");
      }
      console.writeln("========================================\n");
   }
   // Need to manually set this variable as located in a non-initialised section
   pmcHandlerRan = false;

   console.write("Reset source = 0x").write(Rcm::getResetSource(), Radix_16).write(" = ").writeln(Rcm::getResetSourceDescription());

   Pmc::setCallback(callback);

   Pmc::enable();
   Pmc::setLowVoltageReset(PmcLowVoltageDetectAction_Interrupt, PmcLowVoltageDetectLevel_High);
   Pmc::setLowVoltageWarning(PmcLowVoltageWarningAction_Interrupt, PmcLowVoltageWarningLevel_High);

   Pmc::enableNvicInterrupts(NvicPriority_Normal);

   for(;;) {
      console.writeln("Waiting for low-voltage event");
      waitMS(100);
   }
   return 0;
}
