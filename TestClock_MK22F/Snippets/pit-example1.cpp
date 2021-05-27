/**
 ============================================================================
 * @file  pit-example1.cpp (180.ARM_Peripherals/Snippets/)
 * @brief Programmable Interrupt Timer (PIT) Example
 * @author   podonoghue
 * Toggles LED use PIT for delay.
 * This example uses busy-waiting so it not a practical solution
============================================================================
 */
#include "hardware.h"
#include "pit.h"

using namespace USBDM;

// Connection mapping - change as required
// Led is assumed active-low
using LED   = gpio_LED_RED;

using Timer        = Pit;
using TimerChannel = Timer::Channel<0>;

int main() {

   LED::setOutput(PinDriveStrength_High);

   // Enable PIT
   Timer::configure();

   // Check for errors so far
   checkError();

   for(;;) {
      // Delay in ticks using channel 0
      // This is a busy-waiting loop!
      TimerChannel::delay(100*ms);
      //console.writeln("Tick");
      LED::toggle();
   }
}
