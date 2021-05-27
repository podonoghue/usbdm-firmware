/*
 ============================================================================
 * @file    mcg-test.c
 * @brief   Test Mcg configuration
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "mcg.h"

/**
 * Test MCG
 */

using namespace USBDM;

int main() {

   console.write(" SystemCoreClock = ").writeln(::SystemCoreClock);
   console.write(" SystemBusClock  = ").writeln(::SystemBusClock);

   // These clocks are updated when the clock configuration changes
   console.write(" SystemCoreClock       = ").writeln(USBDM::SystemCoreClock);
   console.write(" SystemBusClock        = ").writeln(USBDM::SystemBusClock);
   console.write(" SystemMcgffClock      = ").writeln(SystemMcgffClock);
   console.write(" SystemMcgFllClock     = ").writeln(SystemMcgFllClock);
   console.write(" SystemMcgPllClock     = ").writeln(SystemMcgPllClock);
   console.write(" SystemMcgOutClock     = ").writeln(SystemMcgOutClock);
   console.write(" SystemLpoClock        = ").writeln(SystemLpoClock);

   // These clocks are determined dynamically
   console.write(" SystemMcgirClock      = ").writeln(McgInfo::getMcgIrClock());
   console.write(" SystemPeripheralClock = ").writeln(SimInfo::getPeripheralClock());
   console.write(" SystemOscerClock      = ").writeln(Osc0Info::getOscerClock());
   console.write(" SystemErclk32kClock   = ").writeln(SimInfo::getErc32kClock());

   for(;;){

   }
}
