/**
 ============================================================================
 * @file analogue-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief Example showing use of a single-ended ADC channel
 *
 *  Created on: 10/6/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "adc.h"
#include "pmc.h"

using namespace USBDM;

/*
 * Demonstrates single-ended conversion on a channel
 */

// Connection mapping - change as required
// Note - many actions on the channel affect the entire ADC

// Shared ADC to use
using MyAdc        = Adc0;

// ADC channel to use
using BandgapAdcChannel  = Adc0::Channel<27>;  // Internal bandgap
using MyAdcChannel       = Adc0::Channel<26>;  // Internal chip temperature

// Resolution to use for ADC
constexpr AdcResolution adcResolution = AdcResolution_16bit_se;

void reportChipTemperature() {
   using TemperatureChannel    = Adc0::Channel<0b11010>;  // Internal temp sensor
   constexpr float VREF_H      = 3.3;                     // External Vref voltage ~ Vcc

   unsigned tMeasure        = TemperatureChannel::readAnalogue();
   float    tVoltage        = tMeasure*(VREF_H/Adc0::getSingleEndedMaximum(adcResolution));
   // Formula from data sheets
   float    chipTemperature = 25 - (tVoltage-0.719)/.001715;

   console.setFloatFormat(1, Padding_LeadingSpaces, 2);
   console.writeln("Temp = ", chipTemperature, " degrees");
   console.resetFormat();
}

int main(void) {
   // Enable and configure ADC
   MyAdc::configure(adcResolution);

   // Calibrate before first use
   MyAdc::calibrate();

   // Connect ADC channel to pin
   MyAdcChannel::setInput();

   // Enable band-gap voltage reference buffer in PMC
   Pmc::configureBandgapOperation(PmcBandgapBuffer_On, PmcBandgapLowPowerEnable_Off);

   console.setFloatFormat(6, Padding_LeadingSpaces, 2);

   for(;;) {
//      reportChipTemperature();

      // Do next conversion
      uint32_t value = BandgapAdcChannel::readAnalogue();

      // Scale value for input voltage range (Assumes Vrh=3.3V, Vrl=0.0V)
      float voltage = value*3.3/MyAdc::getSingleEndedMaximum(adcResolution);

      // Report
      console.writeln("Value = ", voltage, " volts");
      waitMS(200);
   }
}
