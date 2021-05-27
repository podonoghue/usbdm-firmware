/**
 ============================================================================
 * @file    dac-softwareTrigger-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   DAC example using buffer and software trigger
 *
 *  Created on: 30/10/2018
 *      Author: podonoghue
 ============================================================================
 */
/**
 * This example is only useful if the DAC has a reasonable size
 * buffer e.g. 16 entries.
 */
#include "hardware.h"
#include "dac.h"

using namespace USBDM;

using Dac   = Dac0;

/**
 * Configure DAC and initialise DAC buffer
 */
static void configureDac() {

   // Basic configuration - Software trigger
   Dac::configure(
         DacReferenceSelect_Vdda,
         DacPower_High,
         DacTriggerSelect_Software);

   // Normal buffer
   Dac::configureBuffer(
         DacBufferMode_Normal);

   // Connect output to pin (if necessary)
   Dac::setOutput();

   // Fill DAC buffer with values for ramp waveform
   Dac::setBufferLimit(Dac::getBufferSize()-1);
   for (unsigned index=0; index<Dac::getBufferSize(); index++) {
      uint16_t value = round(index*Dac::getRange())/(Dac::getBufferSize()-1);
      Dac::writeValue(index, value);
      console.write("Value[").write(index).write("] = ").writeln(value);
   }
}

int main() {
   console.writeln("Starting");
   
   configureDac();

   // Loop through DAC values using software trigger
   for(;;) {
      Dac::softwareTrigger();
      waitUS(20);
   }

   return 0;
}
