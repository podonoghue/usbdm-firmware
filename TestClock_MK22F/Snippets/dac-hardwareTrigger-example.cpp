/**
 ============================================================================
 * @file    dac-hardwareTrigger-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   DAC example using buffer and hardware trigger
 *
 *  Created on: 20/10/2018
 *      Author: podonoghue
 ============================================================================
 */
/**
 * This example is only useful if the DAC has a reasonable size
 * buffer e.g. 16 entries.
 */
#include "hardware.h"
#include "dac.h"
#include "pdb.h"
#include "smc.h"

using namespace USBDM;

using Dac   = Dac0;

/**
 * Configure PDB to trigger DAC
 */
static void configurePdb() {
    // Configure PDB.
   Pdb0::configure(
         PdbMode_Continuous,
         PdbTrigger_Software);
//   Pdb0::setPeriod(1*ms);
   Pdb0::configureDacTrigger(0, PdbDacTriggerMode_Delayed, 20*us);

   // Registers load on next event
   Pdb0::confirmRegisterLoad(PdbLoadMode_Event);

   // Start timer and load values
   Pdb0::softwareTrigger();
}

/**
 * Configure DAC and initialise DAC buffer
 */
static void configureDac() {

   // Basic configuration - Hardware trigger
   Dac::configure(
         DacReferenceSelect_Vdda,
         DacPower_High,
         DacTriggerSelect_Hardware);

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
   configurePdb();

   // PDB loops through DAC values using hardware trigger
   for(;;) {
      Smc::enterWaitMode();
   }

   return 0;
}
