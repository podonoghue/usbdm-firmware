/**
 ============================================================================
 * @file   pdb-example.cpp (180.ARM_Peripherals/Snippets/)
 * @brief  Basic C++ Programmable Delay Block (PDB) Example
 *
 *  Created on: 10/7/2017
 *      Author: podonoghue
 ============================================================================
 */
/*
 * This example uses PDB software trigger to schedule an ADC conversion
 * Uses an LED for debug timing check.
 *
 * Note - Requires ADC and PDB interrupt handlers to be installed.
 */
#include "hardware.h"
#include "pdb.h"

using namespace USBDM;

// LED connection - change as required
using Led         = GpioA<2, ActiveLow>;
using Pdb         = Pdb0;
using Adc         = Adc0;
using AdcChannel  = Adc0Channel<19>;

// Length of PDB sequence
static constexpr float SEQ_LENGTH    = 10*ms;

// When to take ADC sample
static constexpr float TRIGGER_TIME  =  9*ms;

static uint32_t result;
static bool     complete=false;

static void pdbCallback() {
   complete = true;
   Led::clear();
}

static void pdbErrorCallback() {
   complete = true;
   __BKPT();
}

static void configurePdb() {

   // Note: Can work in timer ticks and avoid floating point if desired
   //   Pdb::setClock(PdbPrescale_128, PdbMultiplier_10);
   //   Pdb::setModulo(1000);
   //   Pdb::setInterruptDelayInTicks(900);
   //   Pdb::setPretriggersInTicks(0, PdbPretrigger0_Delayed, 800);

   Pdb::enable();
   // Software Trigger
   Pdb::setTriggerSource(PdbTrigger_Software);
   // Set call-backs
   Pdb::setErrorCallback(pdbErrorCallback);
   Pdb::setCallback(pdbCallback);
   // Interrupts during sequence or error
   Pdb::setActions(PdbAction_Interrupt, PdbErrorInterrupt_Enabled);

   // Set period of sequence
   Pdb::setPeriod(SEQ_LENGTH);
   // Generate interrupt at end of sequence
   Pdb::setInterruptDelay(SEQ_LENGTH);
   // Take single ADC sample at TRIGGER_TIME
   Pdb::configureAdcPretrigger(0, 0, PdbPretrigger_Delayed, TRIGGER_TIME);
   // Update registers
   Pdb::configureRegisterLoad(PdbLoadMode_Immediate);
   while (!Pdb::isRegisterLoadComplete()) {
      __asm__("nop");
   }
   Pdb::enableNvicInterrupts(NvicPriority_Normal);
}

static void adcCallback(uint32_t value, int) {
   result = value;
   Led::set();
}

static void configureAdc() {

   SimInfo::setAdc0Triggers(SimAdc0TriggerMode_Pdb);

   Adc::enable();
   Adc::setResolution(AdcResolution_8bit_se);
   Adc::calibrate();
   Adc::setCallback(adcCallback);
   Adc::enableNvicInterrupts(NvicPriority_Normal);

   AdcChannel::enableHardwareConversion(AdcPretrigger_0, AdcInterrupt_Enabled);
}

int main() {
   console.writeln("Starting");

   Led::setOutput();

   configureAdc();
   configurePdb();

   for(;;) {
      complete = false;
      Pdb::softwareTrigger();
      while (!complete) {
         __WFI();
      }
      console.write("ch1 = ").writeln(result);
   }
   for(;;) {
      __asm__("nop");
   }
}
