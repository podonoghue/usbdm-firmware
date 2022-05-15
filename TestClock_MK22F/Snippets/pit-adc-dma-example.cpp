/*
 ============================================================================
 * @file    pit-adc-dma-example.cpp (180.ARM_Peripherals/Snippets/)
 * @brief   PIT trigger ADC triggers DMA
 *
 *  Created on: 10/7/2017
 *      Author: podonoghue
 ============================================================================
 */
#include <string.h> // memset()
#include "hardware.h"
#include "pit.h"
#include "dma.h"
#include "adc.h"

using namespace USBDM;

// Connection - change as required
using Led          = GpioA<2,  ActiveLow>;
using Pwm          = Ftm0::Channel<0>;
using MyAdcChannel = Adc0::Channel<8>;
using MyAdc        = MyAdcChannel::OwningAdc;
using MyTmrChannel = Pit::Channel<0>;

/**
 * This example used the PIT to trigger 100 ADC conversions @1ms interval.
 * Each conversion result is captured by the DMAC and transferred to memory.
 *
 * +------------+     +-----------+     +-----------+     +----------+
 * |            |     |           |     |           |     |          |
 * |    PIT     | ==> |    ADC    | ==> |    DMA    | ==> |  Memory  |
 * |            |     |           |     |           |     |          |
 * +------------+     +-----------+     +-----------+     +----------+
 *
 * Requires:
 *  DMAx_IRQHandler
 *  PITx_IRQHandler
 *
 *  Use loop-back from PWM (FTM output) to ADC channel
 */

/**
 * Configure the ADC
 * - Triggered by the PIT
 * - Results retrieved by the DMAC
 */
static void configureAdc() {

   // Enable and configure ADC with mostly default settings
   MyAdc::configure(AdcResolution_16bit_se);

   // Calibrate before use
   MyAdc::calibrate();

   // Set averaging to reduce noise
   MyAdc::setAveraging(AdcAveraging_16);

   // Configure the ADC to use hardware with trigger 0 + DMA
   MyAdcChannel::enableHardwareConversion(AdcPretrigger_0, AdcInterrupt_Disabled, AdcDma_Enabled);

   // Connect channel to pin
   MyAdcChannel::setInput();

   // Connect ADC trigger 0 to PIT
   SimInfo::setAdc0Triggers(SimAdc0TriggerMode_Alt_PreTrigger_0, SimAdc0Trigger_PitCh0);

   // Check for errors so far
   checkError();
}

// Flag to indicate transfer complete
bool complete;

/**
 * DMA complete callback
 *
 * Sets flag to indicate sequence complete.
 * Stops PIT generating events
 * Clears DMA IRQ
 */
void dmaCallback(DmaChannelNum channel) {

   // Only one channel operating - don't bother checking which channel.

   // Clear status
   Dma0::clearInterruptRequest(channel);

   // Clear LED for debug
   Led::off();

   // Disable PIT->ADC requests
   MyTmrChannel::disable();

   // Flag complete to main-line
   complete = true;
}

uint16_t buffer[100] = {1,2,3,4,5,6,7,8,9};

/**
 * Configure the DMA
 * - Triggered by the ADC
 * - Transfers results from the ADC to memory buffer
 */
static void configureDma() {

   // DMA channel number to use
   static constexpr DmaChannelNum DMA_CHANNEL = DmaChannelNum_1;

   /*
    * Structure to define a DMA transfer
    *
    * Each DMA request trigger causes a minor-loop transfer sequence.
    * The minor loops are counted in the major-loop.
    *
    * The following are used during a minor loop:
    *  - SADDR Source address
    *  - SOFF  Adjustment applied to SADDR after each transfer
    *  - DADDR Destination address
    *  - DOFF  Adjustment applied to DADDR after each transfer
    *  - NBYTES Number of bytes to transfer
    *  - Attributes
    *    - ATTR_SSIZE, ATTR_DSIZE Source and destination transfer sizes
    *    - ATTR_SMOD, ATTR_DMOD Modulo
    *
    * The following are used by the major loop
    *  - SLAST Adjustment applied to SADDR after each major loop - can be used to reset the SADDR for next major loop
    *  - DLAST Adjustment applied to DADDR after each major loop - can be used to reset the DADDR for next major loop
    *  - CITER Major loop counter - counts major loops
    */
   /**
    * Structure to define the Transmit DMA transfer
    *
    * Note: This uses a 32-bit transfer even though the transmit data is only 8-bit
    */
   static constexpr DmaTcd tcd = DmaTcd (
      /* Source address                 */ MyAdc::adcR(0),                 // ADC result register
      /* Source offset                  */ 0,                              // SADDR does not change
      /* Source size                    */ DmaSize_16bit,                  // 16-bit read from ADR->R (ignores MSBs)
      /* Source modulo                  */ DmaModulo_Disabled,             // Disabled
      /* Last source adjustment         */ 0,                              // No adjustment as SADDR was unchanged

      /* Destination address            */ (uint32_t)(buffer),             // Start of array for result
      /* Destination offset             */ sizeof(buffer[0]),              // DADDR advances 2 bytes for each request
      /* Destination size               */ DmaSize_16bit,                  // 16-bit write to array
      /* Destination modulo             */ DmaModulo_Disabled,             // Disabled
      /* Last destination adjustment    */ -sizeof(buffer[0]),             // Reset DADDR to start of array on completion

      /* Minor loop byte count          */ dmaNBytes(sizeof(buffer[0])),   // 2-bytes for each ADC DMA request
      /* Major loop count               */ dmaCiter(sizeofArray(buffer)),  // Number of requests to do for entire buffer

      /* Start channel                  */ false,                          // Don't start (triggered by hardware)
      /* Disable Req. on major complete */ true,                           // Clear hardware request when major loop completed
      /* Interrupt on major complete    */ true,                           // Interrupt on completion
      /* Interrupt on half complete     */ false,                          // No interrupt
      /* Bandwidth (speed) Control      */ DmaSpeed_NoStalls               // Full speed (throttled by PIT->ADC)
   );

   // Sequence not complete yet
   complete = false;

   // Enable DMAC with default settings
   Dma0::configure();

   // Set callback for end of transfer
   Dma0::setCallback(DMA_CHANNEL, dmaCallback);

   // Enable interrupts in NVIC
   Dma0::enableNvicInterrupts(DMA_CHANNEL);

   // Configure the transfer
   Dma0::configureTransfer(DMA_CHANNEL, tcd);

   // Enable requests from the channel
   Dma0::enableRequests(DMA_CHANNEL);

   // DMA triggered by ADC requests (continuous = whenever ADC wants)
   DmaMux0::configure(DMA_CHANNEL, Dma0Slot_ADC0, DmaMuxEnable_Continuous);

   // Check for errors so far
   checkError();
}

/**
 * PIT callback
 *
 * Used for debug timing checks.
 * LED toggles on each PIT event
 */
void pitCallback() {
   Led::toggle();
}

/*
 * Configure the PIT
 * - Generates regular events at 1ms interval. Each event is used to initiate an ADC conversions.
 */
void configurePit() {
   // Configure base PIT
   Pit::configure(PitDebugMode_Stop);

   MyTmrChannel::setCallback(pitCallback);

   // Configure channel
   MyTmrChannel::configure(1_ms, PitChannelIrq_Enabled);
   MyTmrChannel::enableNvicInterrupts(NvicPriority_Normal);

   // Check for errors so far
   checkError();
}

/**
 * Configures a timer as a 10ms Square wave output to
 * use as measurement input for the ADC.
 */
void createWaveform() {
   Pwm::Ftm::enable();
   Pwm::Ftm::configure(FtmMode_LeftAlign);
   Pwm::Ftm::setPeriod(10.0_ms);

   Pwm::configure(FtmChMode_PwmHighTruePulses);
   Pwm::setDutyCycle(50);
   Pwm::setOutput();
}

/**
 * Do 100 ADC conversions @1ms interval into memory buffer
 */
void testHardwareConversions() {
   // Clear buffer initially
   memset(buffer, 0, sizeof(buffer));

   configureAdc();
   configureDma();
   configurePit();

   while (!complete) {
      __asm__("nop");
   }

   console.writeln("Completed Transfer\nResults:");
   console.setPadding(Padding_LeadingSpaces).setWidth(5);
   for (unsigned index=0; index<(sizeof(buffer)/sizeof(buffer[0]));) {
      console.write(index).write(": ");
      for (int row=0; row<10; row++, index++) {
         console.write(buffer[index]).write(", ");
      }
      console.writeln();
   }
   console.resetFormat();
}

int main() {
   console.writeln("\n\nStarting");

   // Debug LED
   Led::setOutput();

   createWaveform();
   testHardwareConversions();

   for(;;) {
      __asm__("nop");
   }
   return 0;
}
