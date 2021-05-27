/**
 ============================================================================
 * @file digital-example1.cpp (180.ARM_Peripherals/Snippets/)
 * @brief Basic C++ GPIO output example
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"

using namespace USBDM;

/*
 * Simple example flashing LEDs on digital outputs
 */

// Connection mapping - change as required
using BlueLed  = gpio_LED_BLUE;
using RedLed   = gpio_LED_RED;
using GreenLed = gpio_LED_GREEN;

int main() {
   RedLed::setOutput(
         PinDriveStrength_High,
         PinDriveMode_PushPull,
         PinSlewRate_Slow);
   GreenLed::setOutput(
         PinDriveStrength_High,
         PinDriveMode_PushPull,
         PinSlewRate_Slow);
   BlueLed::setOutput(
         PinDriveStrength_High,
         PinDriveMode_PushPull,
         PinSlewRate_Slow);

   console.setEcho(EchoMode_Off);

   for(;;) {
      console.write("\rPress (R)ed or (G)reen or (B)lue :");
      switch(console.readChar()) {
         case 'r': case 'R' : RedLed::toggle();   break;
         case 'g': case 'G' : GreenLed::toggle(); break;
         case 'b': case 'B' : BlueLed::toggle();   break;
         default: break;
      }
   }
}
