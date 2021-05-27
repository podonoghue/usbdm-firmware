/**
 ============================================================================
 * @file  pit-example3.cpp (180.ARM_Peripherals/Snippets/pit-example3-MK.cpp)
 * @brief Programmable Interrupt Timer (PIT) Example
 *
 * Programmable Interrupt Timer (PIT) Example
 *
 * Demonstrates PIT call-back

 * @author   podonoghue
============================================================================
 */
#include "hardware.h"
#include "pit.h"
#include "smc.h"

using namespace USBDM;

/**
 * This example uses PIT interrupts.
 *
 * It is necessary to enable these in Configure.usbdmProject
 * under the "Peripheral Parameters"->PIT tab.
 * Select irqHandlingMethod option (Software (Use setCallback() or override class method)
 */

// Connection mapping - change as required
using Led = gpio_LED_BLUE;

using Timer        = Pit;
using TimerChannel = Timer::Channel<0>;
/*
 * This callback is set programmatically
 */
void flash(void) {
   Led::toggle();
}

int main() {
   Led::setOutput(
         PinDriveStrength_High,
         PinDriveMode_PushPull,
         PinSlewRate_Slow);

   Timer::configure(PitDebugMode_Stop);

   // Set handler programmatically
   TimerChannel::setCallback(flash);

   // Flash LED @ 1Hz
//   TimerChannel::configureInTicks(::SystemBusClock/2, PitChannelIrq_Enabled);
   TimerChannel::configure(0.5 * seconds, PitChannelIrq_Enabled);

   TimerChannel::enableNvicInterrupts(NvicPriority_Normal);

   // Check for errors so far
   checkError();

   for(;;) {
      // Sleep between interrupts
      Smc::enterWaitMode();
   }
}
