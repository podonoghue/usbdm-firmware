/*
 ========================================================================================
 * @file    vlpr-run-hsrun-example.cpp (180.ARM_Peripherals/snippets)
 * @brief   Basic C++ demo using Smc and Mcg classes
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ========================================================================================
 */
/*
 * This examples assumes that appropriate clock configurations have been created:
 *  - ClockConfig_PEE_120MHz  For HSRUN mode (Core=120MHz, Bus=60MHz, Flash=24MHz)
 *  - ClockConfig_PEE_80MHz   For RUN mode (Core=80MHz, Bus=40MHz, Flash=27MHz)
 *  - ClockConfig_BLPE_4MHz   For VLPR (Core/Bus = 4MHz, Flash = 1MHz)
 *
 *  It is also necessary to configure the CLKOUT Pin
 */
#include "hardware.h"
#include "mcg.h"
#include "smc.h"

using namespace USBDM;

// Map clock settings for each mode to available settings
static constexpr ClockConfig ClockConfig_HSRUN = ClockConfig_PEE_120MHz;
static constexpr ClockConfig ClockConfig_RUN   = ClockConfig_PEE_80MHz;
static constexpr ClockConfig ClockConfig_VLPR  = ClockConfig_BLPE_4MHz;

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
   
   // Monitor clock changes on CLKOUT pin
   ControlInfo::initPCRs(pcrValue());
   SimInfo::setClkout(SimClkoutSel_FlexBus);

   report();

   Smc::enablePowerModes(
         SmcVeryLowPower_Enabled,
         SmcLowLeakageStop_Enabled,
         SmcVeryLowLeakageStop_Enabled,
         SmcHighSpeedRun_Enabled);

   for (;;) {
      /*
       * RUN -> VLPR
       * Change clock down then run mode
       */
      Mcg::clockTransition(Mcg::clockInfo[ClockConfig_VLPR]);
      Smc::enterRunMode(SmcRunMode_VeryLowPower);
      console_setBaudRate(defaultBaudRate);
      report();
      waitMS(2000);

      /*
       * VLPR -> RUN
       * Change mode then clock up
       */
      Smc::enterRunMode(SmcRunMode_Normal);
      Mcg::clockTransition(Mcg::clockInfo[ClockConfig_RUN]);
      console_setBaudRate(defaultBaudRate);
      report();
      waitMS(2000);

#ifdef SMC_PMPROT_AHSRUN
      /*
       * RUN -> HSRUN
       * Change mode then clock up
       */
      Smc::enterRunMode(SmcRunMode_HighSpeed);
      Mcg::clockTransition(Mcg::clockInfo[ClockConfig_HSRUN]);
      console_setBaudRate(defaultBaudRate);
      report();
      waitMS(2000);

      /*
       * HSRUN -> RUN
       * Change clock down then run mode
       */
      Mcg::clockTransition(Mcg::clockInfo[ClockConfig_RUN]);
      Smc::enterRunMode(SmcRunMode_Normal);
      console_setBaudRate(defaultBaudRate);
      report();
      waitMS(2000);
#endif
   }
   return 0;
}
