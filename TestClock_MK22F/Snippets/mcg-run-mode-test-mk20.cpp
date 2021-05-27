/*
 ============================================================================
 * @file    mcg-run-mode-test-mk20.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
/*
 * This examples assumes that two appropriate clock configurations have been created:
 *  - ClockConfig_PEE_48MHz   For RUN mode (Core=48MHz, Bus=48MHz, Flash = 24MHz)
 *  - ClockConfig_BLPE_4MHz   For VLPR mode (Core=4MHz, Bus=4MHz, Flash = 800kHz)
 */
#include "hardware.h"
#include "mcg.h"
#include "smc.h"
#include "lptmr.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

// Clock configurations to use
static constexpr ClockConfig VLPR_CLOCK = ClockConfig_BLPE_4MHz;
static constexpr ClockConfig RUN_CLOCK  = ClockConfig_PEE_48MHz;

// LED connection - change as required
using Led   = USBDM::GpioA<2>;

using Lptmr = Lptmr0;

void toggleLED() {
   Led::toggle();
}

int main() {
   console.writeln("Starting\n");
   console.write("SystemCoreClock = ").writeln(::SystemCoreClock);
   console.write("SystemBusClock  = ").writeln(::SystemBusClock);
   waitMS(200);

   // Enable all power modes
   Smc::enablePowerModes(
         SmcVeryLowPower_Enabled,
         SmcLowLeakageStop_Enabled,
         SmcVeryLowLeakageStop_Enabled
         );

   Led::setOutput();

   /*
    * The LPTMR is used to toggle the LED as a fixed rate irrespective of clock and run mode.
    * The LPO clock is used since it is independent of run mode.
    */
   Lptmr::configureTimeCountingMode(LptmrResetOn_Compare, LptmrInterrupt_Enabled, LptmrClockSel_Lpoclk);
   Lptmr::setPeriod(100*ms);
   Lptmr::setCallback(toggleLED);
   Lptmr::enableNvicInterrupts(NvicPriority_Normal);

   for(;;) {
      /*
       * RUN -> VLPR
       * Change clock down then run mode
       */
      Led::off();
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock/1000000.0<<" MHz -> "<<Flush;
      Mcg::clockTransition(Mcg::clockInfo[VLPR_CLOCK]);
      console.setBaudRate(defaultBaudRate);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock/1000000.0<<" MHz -> ";
      Smc::enterRunMode(SmcRunMode_VeryLowPower);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock/1000000.0<<" MHz\n\n";
      waitMS(2000);
      /*
       * VLPR -> RUN
       * Change mode then clock up
       */
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock/1000000.0<<" MHz -> "<<Flush;
      Smc::enterRunMode(SmcRunMode_Normal);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock/1000000.0<<" MHz -> "<<Flush;
      Mcg::clockTransition(Mcg::clockInfo[RUN_CLOCK]);
      console.setBaudRate(defaultBaudRate);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock/1000000.0<<" MHz\n\n";
      Led::on();
      waitMS(2000);
   }
   return 0;
}
