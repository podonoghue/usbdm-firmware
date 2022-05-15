/*
 ================================================================================
 * @file    ftm-ic-example.cpp
 * @brief   Demo using Ftm class to implement a basic Input Capture system
 *
 * An FTM input channel is used to measure the period of a waveform.
 * This example uses floating point calculations.
 *
 *  Created on: 3/7/2017
 *      Author: podonoghue
 ================================================================================
 */
#include "hardware.h"
#include "ftm.h"

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

/// Timer channel for measurement - change as required
using TimerChannel = Timer::Channel<0>;

/**
 * Period between input edges in ticks.
 * This variable is shared with the interrupt routine
 */
static volatile unsigned periodInTicks;

// Maximum measurement time
static constexpr Seconds MEASUREMENT_TIME = 100_ms;

// Maximum IC interval - the IC interval between measurement events should not exceed this value.
static constexpr Seconds MAX_IC_INTERVAL = (1.1f * MEASUREMENT_TIME);

using Debug = GpioA<12>;

/**
 * Interrupt handler for Timer interrupts
 * This calculates the time between events (rising edges)
 *
 * @param[in] status Flags indicating interrupt source channel(s)
 */
static void ftmCallback(uint8_t status) {
   static unsigned risingEdgeEventTime;

   Debug::set();
   // Check channel
   if (status & TimerChannel::CHANNEL_MASK) {
      unsigned currentEventTime = (unsigned)TimerChannel::getEventTime();
      periodInTicks = currentEventTime-risingEdgeEventTime;
      risingEdgeEventTime = currentEventTime;
   }
   Debug::clear();
}

/**
 * Demonstration main-line
 *
 * @return Not used.
 */
int main() {
   console.writeln("Starting");

   Debug::setOutput(PinDriveStrength_High);

   /**
    * FTM channel set as Input Capture using a callback function
    */
   // Configure base FTM (affects all channels)
   Timer::configure(
         FtmMode_LeftAlign,      // Left-aligned is required for OC/IC
         FtmClockSource_System,  // Bus clock usually
         FtmPrescale_1);         // The prescaler will be re-calculated later

   // Set IC/OC measurement interval to accommodate maximum measurement needed.
   // This adjusts the prescaler value but does not change the clock source.
   Timer::setMaximumInterval(MAX_IC_INTERVAL);

   // Enable interrupts for entire timer
   Timer::enableNvicInterrupts(NvicPriority_Normal);

   // Configure pin associated with channel
   TimerChannel::setInput(
         PinPull_None,
         PinAction_None,
         PinFilter_Passive);
   // or change individual attributes
   //  TimerChannel::setPullDevice(PinPull_Up);
   //  TimerChannel::setFilter(PinFilter_Passive);

   // Set callback function (may be shared by multiple channels)
   TimerChannel::setChannelCallback(ftmCallback);

   // Configure the channel
   TimerChannel::configure(
         FtmChMode_InputCaptureRisingEdge, // Input capture rising edge
         FtmChannelAction_Irq);            //  + interrupts on events

   // Check if configuration failed
   USBDM::checkError();

   // Loop here forever reporting values from ISR (call-back)
   for(;;) {
      Ticks tPeriodInTicks;
      // Access shared data in protected fashion
      // Not necessary on Cortex-M4 as reading a simple variable like this is atomic.
      {
         CriticalSection cs;
         tPeriodInTicks = periodInTicks;
      }
      console.write("Period = ", Timer::convertTicksToSeconds(tPeriodInTicks));
   }
   return 0;
}

