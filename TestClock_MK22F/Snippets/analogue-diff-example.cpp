/**
 ============================================================================
 * @file analogue-diff-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief Example showing use of a differential ADC channel
 *
 *  Created on: 10/6/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "adc.h"

using namespace USBDM;

/*
 * Demonstrates differential conversion on a channel
 */

// Connection mapping - change as required
// Note - many actions on the channel affect the entire ADC

// Shared ADC to use
using MyAdc        = Adc0;

// ADC channel to use
using MyAdcChannel = MyAdc::DiffChannel<0>;

// Resolution to use for ADC
constexpr AdcResolution adcResolution = AdcResolution_9bit_diff;

int main(void) {

   // Enable and configure ADC
   MyAdc::configure(AdcResolution_11bit_diff);

   // Calibrate before first use
   MyAdc::calibrate();

   // May change current resolution as needed e.g.
   MyAdc::setResolution(adcResolution);

   // Connect ADC channel to pin
   MyAdcChannel::setInput();

   for(;;) {
      // Do next conversion
      int32_t value = MyAdcChannel::readAnalogue();
      console.writeln("Difference  = ", value*3.3/MyAdc::getDifferentialMaximum(adcResolution), " volts");
   }
}
