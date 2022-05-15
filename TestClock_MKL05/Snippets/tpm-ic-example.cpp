/*
 ================================================================================
 * @file    tpm-ic-example.cpp
 * @brief   Demo using Tpm class to implement a basic Input Capture system
 *
 * An TPM input channel is used to measure the period of a waveform.
 * This example uses floating point calculations.
 *
 *  Created on: 3/7/2017
 *      Author: podonoghue
 ================================================================================
 */
#include "hardware.h"

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

/// Timer channel for measurement - change as required
using TimerChannel = Timer::Channel<1>;

/**
 * Period between input edges in ticks.
 * This variable is shared with the interrupt routine
 */
static volatile uint16_t periodInTicks = 0;

// Maximum measurement time
static constexpr float MEASUREMENT_TIME = 100_ms;

// Maximum IC interval - the IC interval between measurement events should not exceed this value.
static constexpr float MAX_IC_INTERVAL = (1.1 * MEASUREMENT_TIME);

using Debug = GpioA<12>;

/**
 * Interrupt handler for Timer interrupts
 * This calculates the time between events (rising edges)
 *
 * @param[in] status Flags indicating interrupt source channel(s)
 */
static void ftmCallback(uint8_t status) {
   static uint16_t risingEdgeEventTime;

   Debug::set();
   // Check channel
   if (status & TimerChannel::CHANNEL_MASK) {
      uint16_t currentEventTime = TimerChannel::getEventTime();
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
    * TPM channel set as Input Capture using a callback function
    */
   // Configure base TPM (affects all channels)
   Timer::configure(
         TpmMode_LeftAlign,       // Left-aligned is required for OC/IC
         TpmClockSource_Internal, // Bus clock usually
         TpmPrescale_1);          // The prescaler will be re-calculated later

   // Set IC/OC measurement interval to accommodate maximum measurement needed.
   // This adjusts the prescaler value but does not change the clock source.
   Timer::setMaximumInterval(MAX_IC_INTERVAL);

   // Set callback function shared by all channels
   Timer::setChannelCallback(ftmCallback);

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

   // Configure the channel
   TimerChannel::configure(
         TpmChMode_InputCaptureRisingEdge, // Input capture rising edge
         TpmChannelAction_Irq);            //  + interrupts on events

   // Check if configuration failed
   USBDM::checkError();

   // Loop here forever reporting values from ISR (call-back)
   for(;;) {
      uint16_t tPeriodInTicks;
      // Access shared data in protected fashion
      // Not necessary on Cortex-M as reading a simple variable like this is atomic.
      {
         CriticalSection cs;
         tPeriodInTicks = periodInTicks;
      }
      int intervalInMilliseconds = (int)(1000*Timer::convertTicksToSeconds(tPeriodInTicks));
      console.write("Period = ").write(intervalInMilliseconds).writeln(" ms");
   }
   return 0;
}

