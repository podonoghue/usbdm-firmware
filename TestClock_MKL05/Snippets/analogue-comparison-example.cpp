/**
 ============================================================================
 * @file    analogue-comparison-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo of using ADC comparison hardware with interrupts
 *
 *  Created on: 10/6/2017
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "adc.h"
#include "smc.h"

using namespace USBDM;

/*
 * Demonstrates ADC comparison hardware with interrupts
 */

// Connection - change as required
using Led         = GpioA<1,  ActiveLow>;

// ADC channel to use
using MyAdcChannel  = Adc0::Channel<8>;

// Shared ADC to use
using MyAdc         = MyAdcChannel::OwningAdc;

// Resolution to use for ADC
constexpr AdcResolution adcResolution = AdcResolution_10bit_se;

// Lower window threshold for comparison 20%
constexpr int LOWER_THRESHOLD = MyAdc::getSingleEndedMaximum(adcResolution)*0.2;

// Upper window threshold for comparison 60%
constexpr int UPPER_THRESHOLD = MyAdc::getSingleEndedMaximum(adcResolution)*0.6;

/**
 * ADC callback
 *
 * Will toggle LED while comparison is true
 */
void adcComparisonCallback(uint32_t, int) {
   Led::toggle();
}

int main() {
   // Enable LED
   Led::setOutput();

#ifdef USBDM_PCC_IS_DEFINED
   // Enable and configure ADC
   PccInfo::setAdc0ClockSource(PccDiv2Clock_Sirc);
#endif
   MyAdc::configure(adcResolution);

   // Calibrate before first use
   MyAdc::calibrate();

   // Set up comparison range
   MyAdc::enableComparison(AdcCompare_OutsideRangeExclusive, LOWER_THRESHOLD, UPPER_THRESHOLD);

   /**
    * Set callback
    * The callback is executed each time the Conversion Complete (COCO) flag sets.
    * In comparison mode this only occurs when the converted value matches the comparison set.
    */
   MyAdc::setCallback(adcComparisonCallback);
   MyAdc::enableNvicInterrupts(NvicPriority_Normal);

   // Connect ADC channel to pin
   MyAdcChannel::setInput();

   /**
    * Start continuous conversions with interrupts on comparison true.
    * A bit wasteful of power - should throttle.
    */
   MyAdc::enableContinuousConversions(AdcContinuous_Enabled);
   MyAdcChannel::startConversion(AdcInterrupt_Enabled);

   for(;;) {
      Smc::enterWaitMode();
   }
}
