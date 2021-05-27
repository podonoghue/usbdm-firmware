/*
 ============================================================================
 * @file    llwu-example-mk22f.cpp (180.ARM_Peripherals/Snippets/)
 * @brief   Demonstrates various Run and Low-power modes and LLWU
 *
 *  Created on: 25/09/2017
 *      Author: podonoghue
 ============================================================================
 */
/*
 * This examples assumes that appropriate clock configurations have been created:
 *  - HSRUN_CLOCK_CONFIG = ClockConfig_PEE_120MHz  For HSRUN mode (Core=120MHz, Bus=60MHz, Flash=24MHz)
 *  - RUN_CLOCK_CONFIG   = ClockConfig_PEE_80MHz   For RUN mode (Core=80MHz, Bus=40MHz, Flash=27MHz)
 *  - VLPR_CLOCK_CONFIG  = ClockConfig_BLPE_4MHz   For VLPR (Core/Bus = 4MHz, Flash = 1MHz)
 *
 * Interrupts must be configured for GPIO pin used, LLWU, LPTMR
 * It will also be necessary to modify the linker memory map so that only
 * lowest 32K of SRAM_U (0x10000000..) is used if testing of LLS2 is intended.
 * Interrupts will need to be enabled for Pin (GPIO), LPTMR and LLWU.
 *
  */
#include "hardware.h"
#include "mcg.h"
#include "smc.h"
#include "lptmr.h"
#include "llwu.h"
#include "pmc.h"
#include "rcm.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

// Define clock modes to use
static ClockConfig HSRUN_CLOCK_CONFIG  = ClockConfig_PEE_120MHz;
static ClockConfig RUN_CLOCK_CONFIG    = ClockConfig_PEE_80MHz;
static ClockConfig VLPR_CLOCK_CONFIG   = ClockConfig_BLPE_4MHz;

// May need reduced baud rate for slow clocks
static constexpr int BAUD_RATE = 115200;

// Using LEDs rather defeats VLLSx mode!
using RedLed    = GpioA<1,ActiveLow>;
using GreenLed  = GpioA<2,ActiveLow>;
using BlueLed   = GpioD<5,ActiveLow>;

// Timer to use for timed wake-up
using WakeupTimer = Lptmr0;

// LLWU Pin Filter to use
static constexpr LlwuFilterNum FILTER_NUM = LlwuFilterNum_0;

// LLWU Pin to use for wake-up
using WakeupPin = Llwu::Pin<LlwuPin_Ptc1>;

/** Possible tests - must be in this order */
enum Test {
   NONE, STOP, VLPS, WAIT, VLPW, LLS2, LLS3, VLLS0, VLLS1, VLLS2, VLLS3,
};

/** Names of tests - must match enum Test{} */
static const char *TestNames[] = {
   "NONE", "STOP ", "VLPS ", "WAIT ", "VLPW ", "LLS2 ", "LLS3 ", "VLLS0", "VLLS1", "VLLS2", "VLLS3",
};

struct PreservedData {
   Test     test;                ///< Test being run
   unsigned testCount;           ///< Current repeated test count
   uint8_t  timerDelay;          ///< Delay for timer (seconds)
   bool     continuousTest;      ///< Whether to run test continuously
   bool     enablePin;           ///< Whether to enable Pin interrupt/wake-up
   bool     enableTimer;         ///< Whether to enable Timer interrupt/wake-up
   bool     pinHandlerRan;       ///< Flag to indicate Pin handler ran
   bool     timerHandlerRan;     ///< Flag to indicate Timer handler ran
   bool     llwuHandlerRan;      ///< Flag to indicate LLWU handler ran
};

static_assert(sizeof(PreservedData) < sizeof(RFSYS_Type));

/**
 * The following are located in the Register file which is maintained in
 * Low-leakage modes and preserved across reset.
 */
static PreservedData preservedData = (*(PreservedData*)(RFSYS_BASE_PTR));

/**
 * Call-back for Timer
 */
static void wakeupTimerCallback() {
   // We could also put code here that would execute on LPTMR event
   preservedData.timerHandlerRan = true;
   WakeupTimer::clearInterruptFlag();
   WakeupTimer::enableInterrupts(false);
   __asm__("nop");
}

/**
 * Call-back for direct pin interrupt
 *
 * @param[in] status 32-bit value from ISFR (each bit indicates a pin interrupt source)
 */
static void pinCallback(uint32_t status __attribute__((unused))) {
   usbdm_assert(status & (WakeupPin::PORT_BITMASK), "Unexpected pin interrupt");

   if (status & (WakeupPin::PORT_BITMASK)) {
      preservedData.pinHandlerRan = true;
   }
}

/**
 * Call-back for LLWU events
 */
static void llwuCallback() {
   preservedData.llwuHandlerRan = true;
   if (Llwu::isPeripheralWakeupSource(LlwuPeripheral_Lptmr)) {
      // Wake-up from LPTMR
      WakeupTimer::clearInterruptFlag();
      WakeupTimer::enableInterrupts(false);
   }
   if (Llwu::isPinWakeupSource(WakeupPin::pin)) {
      // Wake-up from pin
      Llwu::clearPinWakeupFlag(WakeupPin::pin);
   }
   if (Llwu::isFilteredPinWakeupSource(FILTER_NUM)) {
      // Wake-up from filtered pin
      Llwu::clearFilteredPinWakeupFlag(FILTER_NUM);
   }
}

/**
 *  Test Stop Modes
 *
 * @param smcStopMode            STOP mode to enter - STOP,VLPS,LLS,VLLS
 * @param smcLowLeakageStopMode  LLS/VLLS mode to enter VLLS0,1,2,3 (in VLLS), LLS2,LLS3 (in LLS)
 */
static void testStopMode(
      SmcStopMode             smcStopMode,
      SmcLowLeakageStopMode   smcLowLeakageStopMode=SmcLowLeakageStopMode_VLLS3) {

   switch (smcStopMode) {
      case SmcStopMode_NormalStop:           console.write("Doing Normal Stop\n");           break;
      case SmcStopMode_VeryLowPowerStop:     console.write("Doing Very Low Power Stop\n");   break;
      case SmcStopMode_LowLeakageStop:       console.write("Doing Low Leakage Stop\n");      break;
      case SmcStopMode_VeryLowLeakageStop:   console.write("Doing Very Low Leakage Stop\n"); break;
   };

   if ((smcStopMode!=SmcStopMode_LowLeakageStop)&&(smcStopMode!=SmcStopMode_VeryLowLeakageStop)) {
      // Not using LLWU
      console.writeln("Disabling LLWU").flushOutput();
      Llwu::disableAllSources();
   }

   // Set STOP mode to enter
   Smc::setStopMode(smcStopMode);
   Smc::setStopOptions(smcLowLeakageStopMode);

   /*
    * Go to sleep - LPTMR or PIN wake-up
    */
   console.writeln("Deep Sleeping...").flushOutput();

   Smc::enterStopMode();

   Llwu::disableAllSources();

   // Make sure handlers have run
   waitMS(10);

#ifdef MCG_C6_PLLS_MASK
   /*
    * If back in RUN mode we need to restore clock as
    * MCG transitions PEE->PBE when in STOP modes.
    * This assumes run mode is PEE
    */
   if (Smc::getStatus() == SmcStatus_RUN) {
      Mcg::clockTransition(Mcg::clockInfo[RUN_CLOCK_CONFIG]);
      console.setBaudRate(BAUD_RATE);
      console.writeln("**** Awake ****").flushOutput();
      console.writeln("Restored clock frequency").flushOutput();
   }
   else
#endif
   {
      console.writeln("**** Awake ****").flushOutput();
   }
}

/**
 * Test Wait modes
 *
 * @param smcRunMode Mode to test
 */
static void testWaitMode(SmcRunMode smcRunMode) {
   /*
    * Go to sleep - LPTMR or PIN wake-up
    */
   switch (smcRunMode) {
      case SmcRunMode_Normal:           console.write("Doing Wait\n");                  break;
      case SmcRunMode_VeryLowPower:     console.write("Doing Very Low Power Wait\n");   break;
      default: break;
   };
   console.writeln("Sleeping...").flushOutput();
   Smc::enterWaitMode();
   console.writeln("Awake!").flushOutput();
}

/**
 * Enable pin wake-up
 *
 * @param[in] preservedData Setting for test
 */
static void enablePin(const PreservedData preservedData) {

   // Disable filtered pin
   Llwu::configureFilteredPinSource(
         FILTER_NUM,
         WakeupPin::pin,
         LlwuFilterPinMode_Disabled);

   // Disable direct pin
   Llwu::configurePinSource(
         WakeupPin::pin,
         LlwuPinMode_Disabled);

   // Disable wake-up pin
   WakeupPin::setInput(
         PinPull_Up,
         PinAction_None,
         PinFilter_Passive);

   if (preservedData.enablePin && (preservedData.test>=LLS2)) {

      // Use LLWU in most Low-leakage modes
      Llwu::clearAllFlags();

      if (preservedData.test!=VLLS0) {
         // LLWU from filtered pin
         // Not available in VLLS0 as LPO not running
         console.writeln("Configuring filtered LLWU pin wake-up").flushOutput();
         Llwu::configureFilteredPinSource(
               FILTER_NUM,
               WakeupPin::pin,
               LlwuFilterPinMode_FallingEdge);
      }
      else {
         // LLWU direct from pin
         console.writeln("Configuring direct LLWU pin wake-up").flushOutput();
         Llwu::configurePinSource(
               WakeupPin::pin,
               LlwuPinMode_FallingEdge);
      }
      Llwu::setCallback(llwuCallback);
      Llwu::enableNvicInterrupts(NvicPriority_Normal);
   }
   if (preservedData.enablePin && (preservedData.test<LLS2)) {

      // Enable pin interrupt if not low-leakage mode
      console.writeln("Configuring pin interrupt for wake-up").flushOutput();

      // Configure wake-up via GPIO interrupt
      WakeupPin::setInput(
            PinPull_Up,
            PinAction_IrqFalling,
            PinFilter_Passive);

      WakeupPin::clearInterruptFlag();
      WakeupPin::setCallback(pinCallback);
      WakeupPin::enablePinNvicInterrupts(NvicPriority_Normal);
   }
   else {
      WakeupPin::disablePinNvicInterrupts();
   }
}

/**
 * Enable LPTMR wake-up
 *
 * @param[in] preservedData Setting for test
 */
static void enableTimer(const PreservedData preservedData) {

   if (preservedData.enableTimer) {
      // Set up wake-up timer
      // Note - need a clock source that operates in LLSx e.g. ERCLK32

      console.writeln("Configuring timer interrupt").flushOutput();

      WakeupTimer::configureTimeCountingMode(
            LptmrResetOn_Compare,
            LptmrInterrupt_Enabled,
            LptmrClockSel_Lpoclk);
      WakeupTimer::setPeriod(preservedData.timerDelay*seconds);
      WakeupTimer::setCallback(wakeupTimerCallback);
      WakeupTimer::enableNvicInterrupts(NvicPriority_Normal);

      if ((preservedData.test>=LLS2) && (preservedData.test<=VLLS3)) {

         // Use LLWU with timer
         Llwu::clearAllFlags();

         console.writeln("Configuring timer LLWU wake-up").flushOutput();
         Llwu::configurePeripheralSource(LlwuPeripheral_Lptmr);
      }
   }
   else {
      WakeupTimer::disableNvicInterrupts();
   }
}

/**
 * Report which handlers ran
 */
static void reportHandlersRun() {
   console.write("Timer callback() ").writeln(preservedData.timerHandlerRan?"Ran":"Didn't run");
   console.write("Pin callback()   ").writeln(preservedData.pinHandlerRan?"Ran":"Didn't run");
   console.write("LLWU callback()  ").writeln(preservedData.llwuHandlerRan?"Ran":"Didn't run");
   console.writeln("**************************************").flushOutput();
}

/**
 * Run single test
 *
 * @param[in, out] preservedData Setting for test
 */
static void runSingleTest(PreservedData &preservedData) {

   // Timer can't be used with VLLS0
   preservedData.enableTimer =  preservedData.enableTimer && (preservedData.test!=VLLS0);

   console.writeln("\n**************************************").flushOutput();
   console.write("Running Test: ").write(TestNames[preservedData.test]).write(", #").writeln(preservedData.testCount);

   if (preservedData.test == NONE) {
      // Not a test
      return;
   }

   if (!preservedData.enablePin && !preservedData.enableTimer) {
      console.writeln("Can't do test without Pin or Timer wake-up method\n");
      return;
   }

   // Clear call-back flags
   preservedData.pinHandlerRan   = false;
   preservedData.llwuHandlerRan  = false;
   preservedData.timerHandlerRan = false;

   enableTimer(preservedData);
   enablePin(preservedData);

   console.write("Wake-up using ").write(preservedData.enablePin?"Pin, ":"").writeln(preservedData.enableTimer?"Timer":"");

   switch(preservedData.test) {
      case WAIT:  testWaitMode(SmcRunMode_Normal);             break;
      case VLPW:  testWaitMode(SmcRunMode_VeryLowPower);       break;
      case STOP:  testStopMode(SmcStopMode_NormalStop);        break;
      case VLPS:  testStopMode(SmcStopMode_VeryLowPowerStop);  break;
      case LLS2:  testStopMode(SmcStopMode_LowLeakageStop,     SmcLowLeakageStopMode_LLS2);  break;
      case LLS3:  testStopMode(SmcStopMode_LowLeakageStop,     SmcLowLeakageStopMode_LLS3);  break;
      case VLLS0: testStopMode(SmcStopMode_VeryLowLeakageStop, SmcLowLeakageStopMode_VLLS0); break;
      case VLLS1: testStopMode(SmcStopMode_VeryLowLeakageStop, SmcLowLeakageStopMode_VLLS1); break;
      case VLLS2: testStopMode(SmcStopMode_VeryLowLeakageStop, SmcLowLeakageStopMode_VLLS2); break;
      case VLLS3: testStopMode(SmcStopMode_VeryLowLeakageStop, SmcLowLeakageStopMode_VLLS3); break;
      case NONE: break;
   }
   Llwu::disableNvicInterrupts();
   WakeupTimer::disableNvicInterrupts();
   WakeupPin::disablePinNvicInterrupts();

   reportHandlersRun();
}

/**
 * Run repeated test
 *
 * @param[in] preservedData Setting for test
 */
static void runRepeatedTest(PreservedData &preservedData) {
   do {
      preservedData.testCount++;
      runSingleTest(preservedData);
   } while (preservedData.continuousTest & (console.peek() < 0));

   if (preservedData.continuousTest) {
      // Discard break char
      console.readChar();
   }
}

/**
 * Change run mode
 * VLPR->RUN->HSRUN
 *
 * @return Run mode entered
 */
static SmcStatus changeRunMode() {
   SmcStatus smcStatus = Smc::getStatus();
   if (smcStatus == SmcStatus_HSRUN) {
      // HSRUN->RUN
      Mcg::clockTransition(Mcg::clockInfo[RUN_CLOCK_CONFIG]);
      Smc::enterRunMode(SmcRunMode_Normal);
      console.setBaudRate(defaultBaudRate);
      console.writeln("Changed to RUN mode").flushOutput();
      // RUN->VLPR
      Mcg::clockTransition(Mcg::clockInfo[VLPR_CLOCK_CONFIG]);
      Smc::enterRunMode(SmcRunMode_VeryLowPower);
      console.setBaudRate(BAUD_RATE);
      console.writeln("Changed to VLPR mode").flushOutput();
   }
   else if (smcStatus == SmcStatus_VLPR) {
      // VLPR->RUN mode
      Smc::enterRunMode(SmcRunMode_Normal);
      Mcg::clockTransition(Mcg::clockInfo[RUN_CLOCK_CONFIG]);
      console.setBaudRate(BAUD_RATE);
      console.writeln("Changed to RUN mode").flushOutput();
   }
   else if (smcStatus == SmcStatus_RUN) {
      // RUN->HSRUN
      Smc::enterRunMode(SmcRunMode_HighSpeed);
      Mcg::clockTransition(Mcg::clockInfo[HSRUN_CLOCK_CONFIG]);
      console.setBaudRate(defaultBaudRate);
      console.writeln("Changed to HSRUN mode").flushOutput();
   }
   return Smc::getStatus();
}

static void help() {
   console.write(
         "\n\n"
         "********************************************************************************\n"
#ifdef SMC_PMCTRL_LPWUI_MASK
         "LPWUI - Whether to exit VLPR, VLPW and VLPS to RUN mode on interrupt\n"
#endif
         "RUN   - Full speed run mode\n"
         "VLPR  - Very Low Power Run mode (reduced speed)\n"
         "WAIT  - Wait mode (enter from RUN only)\n"
         "VLPW  - Very low power wait (enter from VLPR only)\n"
         "STOP  - Stop mode (enter from RUN only)\n"
         "VLPS  - Very low power Stop\n"
         "LLS2  - Low Leakage Stop 2 - Partial SRAMU retained\n"
         "LLS3  - Low Leakage Stop 3 - All RAM retained\n"
         "VLLS0 - Very Low Leakage Stop 0 - No SRAM, exit via LLWU reset\n"
         "VLLS1 - Very Low Leakage Stop 1 - No SRAM, exit via LLWU reset\n"
         "VLLS2 - Very Low Leakage Stop 2 - Partial SRAMU retained, exit via LLWU reset\n"
         "VLLS3 - Very Low Leakage Stop 3 - All RAM retained, exit via LLWU reset\n"
         "LPTMR - Low Power Timer (Uses LPO so not available in VLLS0)\n"
         "Pin   - Port Pin used for interrupt or wake-up source\n"
         "Cont  - Run tests continuously until a key is pressed\n"
         "********************************************************************************\n"
   );
}

int main() {
   // Set LPUART (console) clock to clock source available in VLPR mode
  //SimInfo::setLpuartClock(SimLpuartClockSource_OscerClk);
   console.setBaudRate(BAUD_RATE);

   console.writeln("\n**************************************");
   console.write("Executing from RESET, SRS=").writeln(Rcm::getResetSourceDescription());

   if ((Rcm::getResetSource() & RcmSource_Wakeup) != 0) {
      console.writeln("========================================");
      console.writeln("Reset due to LLWU");

      bool llwuDeviceFlag  = Llwu::getPeripheralWakeupSources()&LlwuPeripheral_Lptmr;
      bool llwuPinFlags    = Llwu::isPinWakeupSource(WakeupPin::pin);
      bool llwuFilterFlag  = Llwu::isFilteredPinWakeupSource(FILTER_NUM);

      console.write("LLWU DeviceFlag = ").writeln(llwuDeviceFlag);
      console.write("LLWU PinFlag    = ").writeln(llwuPinFlags);
      console.write("LLWU FilterFlag = ").writeln(llwuFilterFlag);
      reportHandlersRun();
   }
   else {
      // Only allows repeated tests from reset if LLWU
      preservedData.continuousTest = false;
   }

   // Configure LEDs as off to reduce power
   RedLed::setOutput(   PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Slow);
   GreenLed::setOutput( PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Slow);
   BlueLed::setOutput(  PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Slow);

   // Enable all power modes
   Smc::enablePowerModes(
         SmcVeryLowPower_Enabled,
         SmcLowLeakageStop_Enabled,
         SmcVeryLowLeakageStop_Enabled,
         SmcHighSpeedRun_Enabled
   );

   //Errata e4481 STOP mode recovery unstable
//   Pmc::setBandgapOperation(PmcBandgapBuffer_Off, PmcBandgapLowPowerEnable_On);

   // Retain all RAM during LLS2 mode and VLLS2 modes.
//   Pmc::setVlpRamRetention(0b11111111);

   checkError();

   if (preservedData.continuousTest) {
      // Must be running repeated VLLS test
      runRepeatedTest(preservedData);
   }

   preservedData.test           = STOP;
   preservedData.continuousTest = false;
   preservedData.enablePin      = true;
   preservedData.enableTimer    = true;
   preservedData.timerDelay     = 5;

   bool  refresh                 = true;
   Test  oldTest     = STOP;

#ifdef SMC_PMCTRL_LPWUI_MASK
   bool lpwui = false;
#endif

   for(;;) {
      SmcStatus smcStatus = Smc::getStatus();
      if (refresh) {
         console.write("SystemCoreClock  = ").write(::SystemCoreClock/1000000.0).writeln(" MHz");
         console.write("SystemBusClock   = ").write(::SystemBusClock/1000000.0).writeln(" MHz");

         switch(smcStatus) {
            case SmcStatus_HSRUN:
               console.write(
                     "\nTests\n"
                     "====================================\n"
                     "R - Change run mode - VLPR, RUN, HSRUN\n"
                     "T - Toggle LPTMR wake-up source\n"
                     "P - Toggle PIN wake-up source\n"
                     "H - Help\n"
               );
               break;
            default:
            case SmcStatus_RUN:
               console.write(
                     "\nTests\n"
                     "====================================\n"
                     "R   - Change run mode - VLPR, RUN, HSRUN\n"
                     "S   - Select STOP, VLPS test\n"
                     "W   - Select WAIT test\n"
                     "L   - Select LLS2, LLS3 test\n"
                     "V   - Select VLLS0, VLLS1, VLLS2, VLLS3 test\n"
#ifdef SMC_PMCTRL_LPWUI_MASK
                     "I   - Toggle LPWUI (Exit VLP to Run mode on interrupt)\n"
#endif
                     "T   - Toggle LPTMR wake-up (not available in VLLS0)\n"
                     "P   - Toggle PIN wake-up source\n"
                     "C   - Toggle Continuous tests\n"
                     "+/- - Change timer delay\n"
                     "H - Help\n"
               );
               break;
            case SmcStatus_VLPR:
               console.write(
                     "\nTests\n"
                     "====================================\n"
                     "R   - Change run mode - VLPR, RUN, HSRUN\n"
                     "S   - Select VLPS test\n"
                     "W   - Select VLPW test\n"
                     "L   - Select LLS2, LLS3 test\n"
                     "V   - Select VLLS0, VLLS1, VLLS2, VLLS3 test\n"
                     "T   - Toggle LPTMR wake-up (not available in VLLS0)\n"
                     "P   - Toggle PIN wake-up\n"
                     "C   - Toggle Continuous tests\n"
                     "+/- - Change timer delay\n"
                     "H   - Help\n"
               );
               break;
         }
         refresh = false;
      }
      console.write("\rE - Run Test (");
      console.write(Smc::getSmcStatusName());
      console.write(":");
      console.write(Mcg::getClockModeName());
      console.write("@");
      console.write(::SystemCoreClock);
#ifdef SMC_PMCTRL_LPWUI_MASK
      console.write(lpwui?", LPWUI":"       ");
#endif
      console.write(preservedData.continuousTest?", Cont":"      ");
      console.write(preservedData.enablePin?", Pin":"     ");
      if (preservedData.enableTimer&&(preservedData.test!=VLLS0)) {
         console.write(", Timer(").setWidth(2).setPadding(Padding_LeadingSpaces).write(preservedData.timerDelay).write(")");
      }
      else {
         console.write("           ");
      }
      console.resetFormat();
      console.write(", Test=");
      console.write(TestNames[preservedData.test]);
      console.write(") :   ");
      console.flushOutput();
      console.setEcho(EchoMode_Off);
      int command = toupper(console.readChar());
      switch(command) {
         case 'S':
            if (smcStatus==SmcStatus_RUN) {
               preservedData.test = (preservedData.test==VLPS)?STOP:VLPS;
            }
            else if (smcStatus==SmcStatus_VLPR) {
               preservedData.test = VLPS;
            }
            break;
         case 'W':
            if (smcStatus==SmcStatus_RUN) {
               preservedData.test = WAIT;
            }
            else if (smcStatus==SmcStatus_VLPR) {
               preservedData.test = VLPW;
            }
            break;
         case 'L':
            if (smcStatus!=SmcStatus_HSRUN) {
               preservedData.test = (preservedData.test != LLS2)?LLS2:LLS3;
            }
            break;
         case 'V':
            if (smcStatus!=SmcStatus_HSRUN) {
               preservedData.test = 
                 ((preservedData.test != VLLS0)&&
                  (preservedData.test != VLLS1)&&
                  (preservedData.test != VLLS2))?VLLS0:(Test)(preservedData.test+1);
            }
            break;
         case 'R':
            console.writeln("\n").flushOutput();
            switch(changeRunMode()) {
               case SmcStatus_HSRUN:
                  oldTest = preservedData.test;
                  preservedData.test=NONE;
                  break;
               default:
               case SmcStatus_RUN:
                  if (preservedData.test==VLPW) {
                     preservedData.test=WAIT;
                  }
                  else if (preservedData.test==VLPS) {
                     preservedData.test=STOP;
                  }
                  break;
               case SmcStatus_VLPR:
                  preservedData.test = oldTest;
                  if (preservedData.test==WAIT) {
                     preservedData.test=VLPW;
                  }
                  else if (preservedData.test==STOP) {
                     preservedData.test=VLPS;
                  }
               break;
            }
            refresh = true;
            break;
#ifdef SMC_PMCTRL_LPWUI_MASK
         case 'I':
            if (smcStatus==SmcStatus_RUN) {
               lpwui = !lpwui;
               Smc::setExitVeryLowPowerOnInterrupt(lpwui?SmcExitVeryLowPowerOnInt_Enabled:SmcExitVeryLowPowerOnInt_Disabled);
            }
            break;
#endif
         case 'P':
            preservedData.enablePin = !preservedData.enablePin;
            break;
         case 'T':
            preservedData.enableTimer = !preservedData.enableTimer;
            break;
         case 'H':
         case '?':
            help();
            refresh = true;
            break;
         case 'C':
            preservedData.continuousTest = !preservedData.continuousTest;
            break;
         case '+':
            if (preservedData.timerDelay<99) {
               preservedData.timerDelay++;
            }
            break;
         case '-':
            if (preservedData.timerDelay>1) {
               preservedData.timerDelay--;
            }
            break;
         case 'E':
            if (preservedData.test != NONE) {
               console.writeln().flushOutput();
               preservedData.testCount = 0;
               runRepeatedTest(preservedData);
               refresh = true;
            }
            break;
         default: break;
      }
   }
   return 0;
}
