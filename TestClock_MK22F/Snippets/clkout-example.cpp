/**
 ============================================================================
 * @file    clkout-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Demonstrates CLKOUT selection
 *
 *  Created on: 11/6/2019
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"

using namespace USBDM;

int main() {
   console.writeln("Starting\n");
   console.write("SystemCoreClock = ").writeln(SystemCoreClock);
   console.write("SystemBusClock  = ").writeln(SystemBusClock);

   ControlInfo::initPCRs(pcrValue());

   console.setEcho(EchoMode_Off);

   for(;;) {
      // Get value [0-7]
      int ch = console.readChar();
      if ((ch<'0')||(ch>'7')) {
         // Ignore invalid input
         continue;
      }
      SimClkoutSel simClkoutSel = (SimClkoutSel)SIM_SOPT2_CLKOUTSEL(ch-'0');
      SimInfo::setClkout(simClkoutSel);
      console.write((char)ch).write(" : clkout = ");
      switch(simClkoutSel) {
         case SimClkoutSel_FlexBus   : console.writeln("FlexBus  "); break;
         case SimClkoutSel_Reserved1 : console.writeln("Reserved1"); break;
         case SimClkoutSel_Flash     : console.writeln("Flash    "); break;
         case SimClkoutSel_Lpo       : console.writeln("Lpo 1kHz "); break;
         case SimClkoutSel_McgirClk  : console.writeln("McgirClk "); break;
         case SimClkoutSel_RTC       : console.writeln("RTC      "); break;
         case SimClkoutSel_OscerClk0 : console.writeln("OscerClk0"); break;
         case SimClkoutSel_Irc48Mhz  : console.writeln("IRC 48MHz"); break;
      }
   }
   return 0;
}
