/*
 ============================================================================
 * @file    main.cpp (180.ARM_Peripherals/Sources/main.cpp)
 * @brief   Basic C++ demo
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "mcg.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

/**
 * See more examples in Snippets directory
 */

// LED connection - change as required
using Led   = GpioB<8,ActiveLow>;

void report() {
   console.setBaudRate(defaultBaudRate);
   console.setWidth(10).setPadding(Padding_LeadingSpaces);
   console.write(SystemCoreClock).write(',').write(SystemBusClock).write(": ").writeln(Mcg::getClockModeName());
}

int main() {
   console.writeln("Starting\n");

   Led::setOutput();

   for(;;) {
      Led::toggle();
      ClockConfig index = (ClockConfig)(rand()%6);
      report();
      Mcg::clockTransition(Mcg::clockInfo[index]);
   }
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_FEI_42MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_FEE_42MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_BLPI_4MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_BLPE_32kHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_FEI_42MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_FBE_33kHz]);
   report();

   for(int count = 0;;count++) {
      Led::toggle();
      waitMS(100);
      console.write(count).writeln(": Tick...");
   }
   return 0;
}
