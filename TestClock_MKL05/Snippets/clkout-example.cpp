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
   console.writeln("SystemCoreClock = ", SystemCoreClock);
   console.writeln("SystemBusClock  = ", SystemBusClock);

   // Need to check Info table index in ControlInfo or use configuration editor (much easier)
   using ClkOut = PcrTable_T<ControlInfo, 10>;
   ClkOut::setOutput();

   console.setEcho(EchoMode_Off);

   console.writeln("press 1 - 7");
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
         case SimClkoutSel_FlexBus   : console.write("FlexBus  \r"); break;
         case SimClkoutSel_Reserved1 : console.write("Reserved1\r"); break;
         case SimClkoutSel_Flash     : console.write("Flash    \r"); break;
         case SimClkoutSel_Lpo       : console.write("Lpo 1kHz \r"); break;
         case SimClkoutSel_McgirClk  : console.write("McgirClk \r"); break;
         case SimClkoutSel_RTC       : console.write("RTC      \r"); break;
         case SimClkoutSel_OscerClk0 : console.write("OscerClk0\r"); break;
#ifdef USB_CLK_RECOVER_IRC_EN_REG_EN_MASK
         case SimClkoutSel_Irc48MHz  : console.write("IRC 48MHz\r"); break;
#endif
         default: break;
      }
   }
   return 0;
}
