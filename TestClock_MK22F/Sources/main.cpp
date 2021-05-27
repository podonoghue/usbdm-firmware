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
using Led   = GpioA<1,ActiveLow>;

void report() {
   console.setBaudRate(defaultBaudRate);
   console.setWidth(10).setPadding(Padding_LeadingSpaces);
   console.write(SystemCoreClock).write(',').write(SystemBusClock).write(": ").writeln(Mcg::getClockModeName());
   console.flushOutput();
}

int main() {
   console.writeln("Starting\n");

   Led::setOutput();

   for(;;) {
      Led::toggle();
      ClockConfig index = (ClockConfig)(rand()%8);
      report();
      Mcg::clockTransition(Mcg::clockInfo[index]);
   }
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_FEI_84MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_FEE_80MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_BLPI_4MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_BLPE_8MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_FEI_84MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_FBE_8MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_PBE_8MHz]);
   report();
   Mcg::clockTransition(Mcg::clockInfo[ClockConfig::ClockConfig_PEE_48M]);
   report();

   for(int count = 0;;count++) {
      Led::toggle();
      waitMS(100);
      console.write(count).writeln(": Tick...");
   }
   return 0;
}
