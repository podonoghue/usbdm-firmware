/**
 ============================================================================
 * @file    dac-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo using DAC class
 *
 *  Created on: 30/10/2018
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "dac.h"

using namespace USBDM;

// 100 samples
static constexpr unsigned NUM_SAMPLES = 100;

// Table of values for a sine wave
static uint16_t sineTable[NUM_SAMPLES];

using Dac = Dac0;

constexpr float PI = 3.14159265358979323846;

/**
 * Fill sineTable with scaled sine waveform
 */
void initSineTable() {

   for (unsigned index = 0; index<(sizeof(sineTable)/sizeof(sineTable[0])); index++) {
      sineTable[index] = roundf(Dac::getRange() * (1 + sin(index*2*PI/NUM_SAMPLES))/2);
   }
}

/**
 * Configure DAC for simple software use
 */
static void configureDac() {

   // Basic configuration
   Dac::configure(
         DacReferenceSelect_Vdda,
         DacPower_High,
         DacTriggerSelect_Software);

   // No buffer
   Dac::configureBuffer(
         DacBufferMode_Disabled);

   // Connect output to pin (if necessary)
   Dac::setOutput();
}

int main() {
   console.writeln("Starting");

   initSineTable();
   configureDac();

   // Loop through DAC values
   for(;;) {
      for (unsigned i=0; i<(sizeof(sineTable)/sizeof(sineTable[0])); i++) {
         Dac::writeValue(sineTable[i]);
         waitUS(10);
      }
   }
   return 0;
}
