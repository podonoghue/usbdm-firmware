/*
 ============================================================================
 * @file lptmr-reaction-time.cpp (180.ARM_Peripherals/Snippets/)
 * @brief   Basic C++ LPTMR demo
 *          Measures reaction time (no refinements!)
 *
 *  Created on: 25/09/2017
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "lptmr.h"
#include "random"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

// I/O connections - change as required
using Led    = GpioC<1,ActiveHigh>;
using Button = GpioD<5,ActiveLow>;

// Using LPTMR0
using Timer = Lptmr0;

int main() {

   Led::setOutput(
         PinDriveStrength_Low,
         PinDriveMode_PushPull,
         PinSlewRate_Slow);

   Button::setInput(
         PinPull_Up,
         PinAction_None,
         PinFilter_None);

   // 8MHz / 4096 => ~500 ns resolution
   Timer::configureTimeCountingMode(
         LptmrResetOn_Overflow,
         LptmrInterrupt_Disabled,
         LptmrClockSel_Oscerclk,
         LptmrPrescale_4096);

   for(;;) {
      Led::off();
      waitMS(3000+rand()%4000);
      Led::on();

      // This code assumes reasonable reaction times!
      uint32_t startTime = Timer::getCounterValue();
      while (Button::isReleased()) {
      }
      uint32_t endTime = Timer::getCounterValue();

      // Roll-over of the timer is not an issue as modulo arithmetic is used (uint32_t).
      uint32_t reactionTimeInTicks = endTime-startTime;
      float reactionTime = Timer::convertTicksToSeconds(reactionTimeInTicks);
      console.write("Your reaction time is ").write(reactionTime).writeln(" s");
   }
   return 0;
}
