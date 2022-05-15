/**
 ============================================================================
 * @file analogue-interrupt-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief Example showing use of a interrupts with an ADC channel
 *
 *  Created on: 10/6/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "adc.h"

using namespace USBDM;

/*
 * Demonstrates conversion on a single channel with interrupts
 */

// Connection mapping - change as required
// (ADC Ch(19) = light sensor on FRDM-K20
using MyAdcChannel = Adc0::Channel<8>;
using MyAdc        = MyAdcChannel::OwningAdc;

/**
 * NOTE:  This is not a sensible approach
 *        Using serial I/O in a ISR is very silly!!!!
 */
void handler(uint32_t result, int) {
   result = result/10;
   for (unsigned i=0; i<result; i++) {
      console.write('X');
   }
   console.writeln();
}

int main(void) {
   console.writeln("Starting");

   // Enable and configure ADC
   MyAdc::configure(AdcResolution_8bit_se);

   // Calibrate before use
   MyAdc::calibrate();

   // Note: Setting callback affects all channels on the ADC
   MyAdc::setCallback(handler);
   MyAdc::enableNvicInterrupts(NvicPriority_Normal);

   // Connect ADC channel to pin
   MyAdcChannel::setInput();

   // Check for error so far
   checkError();

   for(;;) {
      // Start a conversion with interrupt on completion
      MyAdcChannel::startConversion(AdcInterrupt_Enabled);
      waitMS(100);
   }
}
