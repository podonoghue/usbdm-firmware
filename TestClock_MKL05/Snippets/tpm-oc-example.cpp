/**
 ============================================================================
 * @file    tpm-oc-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Demo using Tpm class to implement a basic Output Compare system
 *
 *  An TPM output generates a square wave with 100ms period
 *
 *  Created on: 3/7/2017
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "smc.h"

using namespace USBDM;

/**
 * This example uses TPM interrupts.
 *
 * It is necessary to enable these in Configure.usbdmProject
 * under the "Peripheral Parameters"->TPM tab.
 * Select irqHandlingMethod option (Class Method - Software ...)
 */

/**
 * Timer being used - change as required
 * Could also access as TimerChannel::Tpm
 */
using Timer = Tpm0;

/// Timer channel for output - change as required
using TimerChannel = Timer::Channel<1>;

/**
 * Half-period for timer in ticks.
 * This variable is shared with the interrupt routine
 */
static volatile uint16_t timerHalfPeriodInTicks;

/// Waveform period to generate
static constexpr float WAVEFORM_PERIOD = 100*ms;

/// Maximum OC interval - the OC interval should not exceed this value.
static constexpr float MAX_OC_INTERVAL = (1.1 * WAVEFORM_PERIOD)/2;

/**
 * Interrupt handler for Timer interrupts
 * This sets the next interrupt/pin toggle for a half-period from the last event
 *
 * @param[in] status Flags indicating interrupt source channel(s)
 */
static void tpmCallback(uint8_t status) {

   // Check channel
   if (status & TimerChannel::CHANNEL_MASK) {
      // Note: The pin is toggled directly by hardware
      // Re-trigger at last interrupt time + timerHalfPeriodInTicks
      TimerChannel::setDeltaEventTime(timerHalfPeriodInTicks);
   }
}

/**
 * Demonstration main-line
 *
 * @return Not used.
 */
int main() {
   /**
    * TPM channel set as Output compare with pin Toggle mode and using a callback function
    */
   // Configure base TPM (affects all channels)
   SimInfo::setTpmClock(SimTpmClockSource_Peripheral);
   Timer::configure(
         TpmMode_LeftAlign,        // Left-aligned is required for OC/IC
         TpmClockSource_Internal); // Bus clock usually

   // Set IC/OC measurement interval to longest interval needed.
   // This adjusts the prescaler value but does not change the clock source
   Timer::setMaximumInterval(MAX_OC_INTERVAL, true);

#if 1
   uint16_t modulo = Timer::getCounterMaximumValue();
   Timer::setPeriodInTicks(1000, false);
   Timer::setPeriodInTicks(2000, false);
   Timer::setPeriodInTicks(3000, false);
   Timer::setPeriodInTicks(1000, true);
   Timer::setPeriodInTicks(2000, true);
   Timer::setPeriodInTicks(3000, true);
   Timer::setCounterMaximumValue(modulo, true);
   Timer::setClockSource(TpmClockSource_External);
   Timer::setClockSource(TpmClockSource_Internal);
   Timer::setClockSource(TpmClockSource_External);
   Timer::setClockSource(TpmClockSource_Internal);
   Timer::enableTimerOverflowInterrupts(true);
   Timer::enableTimerOverflowInterrupts(false);
   Timer::enableTimerOverflowInterrupts(true);
   Timer::enableTimerOverflowInterrupts(false);
   Timer::enableTimerOverflowDma(true);
   Timer::enableTimerOverflowDma(false);
   Timer::enableTimerOverflowDma(true);
   Timer::enableTimerOverflowDma(false);
   Timer::setMode(TpmMode_CentreAlign);
   Timer::setMode(TpmMode_LeftAlign);
   Timer::setMode(TpmMode_CentreAlign);
   Timer::setMode(TpmMode_LeftAlign);
#endif

   // Calculate half-period in timer ticks
   // Must be done after timer clock configuration (above)
   timerHalfPeriodInTicks = Timer::convertSecondsToTicks(WAVEFORM_PERIOD/2.0);

   // Set callback function
   Timer::setChannelCallback(tpmCallback);

   // Enable interrupts for entire timer
   Timer::enableNvicInterrupts(NvicPriority_Normal);

   // Configure pin associated with channel
   TimerChannel::setOutput(
         PinDriveStrength_High,
         PinDriveMode_PushPull,
         PinSlewRate_Slow);
   // or change individual attributes
   //  TimerChannel::setDriveStrength(PinDriveStrength_High);
   //  TimerChannel::setDriveMode(PinDriveMode_PushPull);

   // Trigger 1st interrupt at now+100
   TimerChannel::setRelativeEventTime(100);

   // Configure the channel
   TimerChannel::configure(
         TpmChMode_OutputCompareToggle, //  Output Compare with pin toggle
         TpmChannelAction_Irq);         //  + interrupts on events

   // Check if configuration failed
   USBDM::checkError();

   // Wait here forever (sleeping between interrupts)
   for(;;) {
      Smc::enterWaitMode();
   }
   return 0;
}

