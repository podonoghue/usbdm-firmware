/*
 ========================================================================================
 * @file    vlpr-run-example.cpp (180.ARM_Peripherals/snippets)
 * @brief   Basic C++ demo using Smc and Mcg classes
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ========================================================================================
 */
/*
 * This examples assumes that appropriate clock configurations have been created:
 *  - ClockConfig_PEE_48MHz   For RUN mode (Core/Bus=48MHz, Flash=24MHz)
 *  - ClockConfig_BLPE_4MHz   For VLPR (Core/Bus = 4MHz, Flash = 1MHz)
 */
#include "hardware.h"
#include "mcg.h"
#include "smc.h"

using namespace USBDM;

// Map clock settings for each mode to available settings
static constexpr ClockConfig ClockConfig_RUN   = ClockConfig_PEE_48MHz;
static constexpr ClockConfig ClockConfig_VLPR  = ClockConfig_BLPE_4MHz;

// LED connection - change as required
using Led   = GpioC<3>;

using namespace USBDM;

void report() {
   console.write("Run mode = ").write(Smc::getSmcStatusName());
   console.write(", Clock = ").write(Mcg::getClockModeName());
   console.write("@").write(::SystemCoreClock).writeln(" Hz").flushOutput();
}

int main() {
   console.writeln("Starting\n");
   console.write("SystemCoreClock = ").writeln(::SystemCoreClock);
   console.write("SystemBusClock  = ").writeln(::SystemBusClock);
   
   report();

   Smc::enablePowerModes(
         SmcVeryLowPower_Enabled,
         SmcLowLeakageStop_Enabled,
         SmcVeryLowLeakageStop_Enabled);

   Led::setOutput();

   for (;;) {
      Led::toggle();
      /*
       * RUN -> VLPR
       * Change clock down then run mode
       */
      Mcg::clockTransition(Mcg::clockInfo[ClockConfig_VLPR]);
      Smc::enterRunMode(SmcRunMode_VeryLowPower);
      console_setBaudRate(defaultBaudRate);
      report();
      waitMS(1000);

      /*
       * VLPR -> RUN
       * Change mode then clock up
       */
      Smc::enterRunMode(SmcRunMode_Normal);
      Mcg::clockTransition(Mcg::clockInfo[ClockConfig_RUN]);
      console_setBaudRate(defaultBaudRate);
      report();
      waitMS(1000);
   }
   return 0;
}
