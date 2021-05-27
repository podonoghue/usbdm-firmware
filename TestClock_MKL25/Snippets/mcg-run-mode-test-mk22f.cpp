/*
 ============================================================================
 * @file    mcg-run-mode-test-mk22f.cpp (180.ARM_Peripherals/Sources/)
 * @brief   Basic C++ demo
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
/*
 * This examples assumes that three appropriate clock configurations have been created:
 *  - ClockConfig_PEE_80MHz   For RUN mode (Core=80MHz, Bus=40MHz, Flash = 27MHz)
 *  - ClockConfig_BLPI_4MHz   For VLPR mode (Core=4MHz, Bus=4MHz, Flash = 800kHz)
 *  - ClockConfig_PEE_120MHz  For HSRUN mode (Core=120MHz, Bus=60MHz, Flash = 24MHz)
 */
#include "hardware.h"
#include "mcg.h"
#include "smc.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

// LED connection - change as required
using Led   = USBDM::GpioA<2>;

int main() {
   console.writeln("Starting\n");
   console.write("SystemCoreClock = ").writeln(::SystemCoreClock);
   console.write("SystemBusClock  = ").writeln(::SystemBusClock);
   waitMS(200);

   // Enable all power modes
   Smc::enablePowerModes(
         SmcVeryLowPower_Enabled,
         SmcLowLeakageStop_Enabled,
         SmcVeryLowLeakageStop_Enabled,
         SmcHighSpeedRun_Enabled
         );

   Led::setOutput();

   for(;;) {
      /*
       * RUN -> VLPR
       * Change clock down then run mode
       */
      Mcg::clockTransition(McgInfo::clockInfo[ClockConfig_BLPI_4MHz]);
      console.setBaudRate(defaultBaudRate);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock<<" Hz\n";
      Smc::enterRunMode(SmcRunMode_VeryLowPower);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock<<" Hz\n";
      waitMS(200);
      Led::toggle();
      /*
       * VLPR -> RUN
       * Change mode then clock up
       */
      Smc::enterRunMode(SmcRunMode_Normal);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock<<" Hz\n"<<Flush;
      Mcg::clockTransition(McgInfo::clockInfo[ClockConfig_PEE_80MHz]);
      console.setBaudRate(defaultBaudRate);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock<<" Hz\n";
      waitMS(200);
      Led::toggle();
      /*
       * RUN -> HSRUN
       * Change mode then clock up
       */
      Smc::enterRunMode(SmcRunMode_HighSpeed);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock<<" Hz\n"<<Flush;
      Mcg::clockTransition(McgInfo::clockInfo[ClockConfig_PEE_120MHz]);
      console.setBaudRate(defaultBaudRate);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock<<" Hz\n";
      waitMS(200);
      Led::toggle();
      /*
       * HSRUN -> RUN
       * Change clock down then run mode
       */
      Mcg::clockTransition(McgInfo::clockInfo[ClockConfig_PEE_80MHz]);
      console.setBaudRate(defaultBaudRate);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock<<" Hz\n"<<Flush;
      Smc::enterRunMode(SmcRunMode_Normal);
      console<<Smc::getSmcStatusName()<<":"<<Mcg::getClockModeName()<<"@"<<::SystemCoreClock<<" Hz\n";
      waitMS(200);
      Led::toggle();

   }
   return 0;
}
