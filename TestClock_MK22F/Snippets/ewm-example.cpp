/**
 ============================================================================
 * @file    ewm-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo using EWM class
 *
 *  Created on: 5 Jul 2018
 *      Author: podonoghue
 ============================================================================
 */

#include "hardware.h"
#include "ewm.h"

using namespace USBDM;

using Led = GpioA<2, ActiveLow>;

bool handlerCalled = false;

/**
 * EWM call-back.
 *
 * Executes when EWM times-out
 */
void callback() {
   console.writeln("\nEWM Handler!!!");
   Ewm::enableInterrupt(EwmInterrupt_Disabled);
   handlerCalled = true;
}

int main(){
   console.writeln("\nPress enter to stop watchdog refresh and cause EWM timeout");

   Led::setOutput(PinDriveStrength_High);

   // Configure EWM for no external input
   Ewm::configure(EwmInput_Disabled);

   // Expire if not serviced for ~254ms (LPO ~1kHz)
   Ewm::setWindow(0,254);

   // Set up call-back to execute
   Ewm::setCallback(callback);
   Ewm::enableNvicInterrupts(NvicPriority_Normal);
   Ewm::enableInterrupt();

   // Watch for key-press while servicing watchdog
   while (console.peek()<0) {
      // Flash LED
      Led::toggle();
      waitMS(200);

      // This is not a sensible way to service a watchdog!
      Ewm::writeKeys(EwmKey1, EwmKey2);
   }

   console.write("Waiting for watchdog timeout");

   // Wait for EWM handler to run
   while(!handlerCalled) {
      // Flash LED
      Led::toggle();
      console.write('.');
      waitMS(200);
   }
   // All done
   console.writeln("Done");

   for(;;) {
      __asm__("nop");
   }
   return 0;
}
