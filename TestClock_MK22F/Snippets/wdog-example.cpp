/**
 ============================================================================
 * @file    wdog-example.cpp
 * @brief   Basic C++ demo using WDOG class
 *
 *  Created on: 5 Jul 2018
 *      Author: podonoghue
 ============================================================================
 */

#include "hardware.h"
#include "wdog.h"
#include "rcm.h"

using namespace USBDM;

using BlueLED  = GpioD<5,  ActiveLow>;

__attribute__ ((section (".noinit")))
static bool watchdogHandlerRan;

/**
 * WDOG call-back.
 *
 * Executes when WDOG times-out before actual reset
 */
void callback() {
   watchdogHandlerRan = true;
   for(;;) {
      // Wait here until reset
      __asm__("nop");
   }
}

int main(){
   console.writeln("\n\nStarting");

   if ((Rcm::getResetSource() & RcmSource_Wdog) != 0) {
      console.writeln("========================================");
      console.writeln("Reset due to watchdog");
      if (watchdogHandlerRan) {
         console.writeln("Watchdog handler ran before reset");
      }
      else {
         console.writeln("Watchdog handler did not run before reset");
      }
      console.writeln("========================================\n");
   }
   // Need to manually set this variable as located in a non-initialised section
   watchdogHandlerRan = false;

   console.write("Reset source = 0x").write(Rcm::getResetSource(), Radix_16).write(" = ").writeln(Rcm::getResetSourceDescription());

   Led::setOutput(PinDriveStrength_High);

   // Configure
   Wdog::configure(
         WdogEnable_Enabled,           // Watchdog enabled
         WdogClock_Lpo,                // Use LPO as clock
         WdogWindow_Disabled,          // No window
         WdogInterrupt_Enabled,        // Interrupt before reset
         WdogEnableInDebug_Disabled,   // Disable WDOG while debugging
         WdogEnableInStop_Disabled,    // Disable WDOG while CPU stopped
         WdogEnableInWait_Disabled);   // Disable WDOG while CPU waiting

   // Time-out ~5s (LPO ~1kHz)
   Wdog::setTimeout(5.0_s);

   // Lock registers
   // Now no watchdog changes are possible unless reset
   Wdog::lockRegisters();

   // Set up interrupt handling
   Wdog::setCallback(callback);
   Wdog::enableNvicInterrupts(NvicPriority_Normal);

   // Check for error so far
   checkError();

   console.setEcho(EchoMode_Off);
   console.writeln("\nPress any key to stop watchdog refresh and cause timeout");

   // Watch for key-press while servicing watchdog
   while (console.peek()<0) {
      // Flash LED
      Led::toggle();
      waitMS(200);

      // Feed the dog - This is not a sensible way to service a watchdog!
      Wdog::writeRefresh(WdogRefresh1, WdogRefresh2);
   }

   console.write("Waiting for watchdog timeout and reset ");

   // Wait for Reset
   unsigned count = 5;
   for(;;) {
      // Flash LED while counting down
      Led::toggle();
      wait(1_s);
      console.write(count--).write(",");
   }

   return 0;
}
