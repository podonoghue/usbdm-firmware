/**
 ============================================================================
 * @file    ftm-oc-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Demo using Ftm class to implement a basic Output Compare system
 *
 *  An FTM output generates a square wave with 100ms period
 *
 *  Created on: 3/7/2017
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "smc.h"

using namespace USBDM;

/**
 * This example uses FTM interrupts.
 *
 * It is necessary to enable these in Configure.usbdmProject
 * under the "Peripheral Parameters"->FTM tab.
 * Select irqHandlingMethod option (Software (Use setCallback() or override class method)
 */

/**
 * Timer being used - change as required
 * Could also access as TimerChannel::Ftm
 */
using Timer = Ftm0;

/// Timer channel for output - change as required
using TimerChannel = Timer::Channel<7>;

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
static void ftmCallback(uint8_t status) {

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
    * FTM channel set as Output compare with pin Toggle mode and using a callback function
    */
   // Configure base FTM (affects all channels)
   Timer::configure(
         FtmMode_LeftAlign,       // Left-aligned is required for OC/IC
         FtmClockSource_System);  // Bus clock usually

   // Set IC/OC measurement interval to longest interval needed.
   // This adjusts the prescaler value but does not change the clock source
   Timer::setMaximumInterval(MAX_OC_INTERVAL);

   // Calculate half-period in timer ticks
   // Must be done after timer clock configuration (above)
   timerHalfPeriodInTicks = Timer::convertSecondsToTicks(WAVEFORM_PERIOD/2.0);

   // Enable interrupts for entire timer
   Timer::enableNvicInterrupts(NvicPriority_Normal);

   // Configure pin associated with channel
   TimerChannel::setOutput(
         PinDriveStrength_High,
         PinDriveMode_PushPull,
         PinSlewRate_Slow);

   // Trigger 1st interrupt at now+100
   TimerChannel::setRelativeEventTime(100);

   // Set callback function (may be shared by multiple channels)
   TimerChannel::setChannelCallback(ftmCallback);

   // Configure the channel
   TimerChannel::configure(
         FtmChMode_OutputCompareToggle, //  Output Compare with pin toggle
         FtmChannelAction_Irq);         //  + interrupts on events

   // Check if configuration failed
   USBDM::checkError();

   // Wait here forever (sleeping between interrupts)
   for(;;) {
      Smc::enterWaitMode();
   }
   return 0;
}

