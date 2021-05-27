/**
 ============================================================================
 * @file    dac-dma-pitThrottled-example.cpp (180.ARM_Peripherals/Snippets/)
 * @brief   Basic C++ demo using DAC, DMA and PIT classes
 *
 *  Created on: 30/10/2018
 *      Author: podonoghue
 ============================================================================
 */
/**
 * This example uses DMA to transfer characters from an array to the DAC for conversion.
 * Uses PIT throttled DMA to DAC to produce a 1 kHz sine waveform.
 * The DAC is operating is software mode as a dumb peripheral.
 */
/**
 * This example uses DMA interrupts.
 *
 * It is necessary to enable these in Configure.usbdmProject
 * under the "Peripheral Parameters"->DMA tab.
 * Select irqHandlers option (Class Method - Software ...)
 */
#include "math.h"
#include "hardware.h"
#include "dma.h"
#include "pit.h"
#include "smc.h"
#include "dac.h"

using namespace USBDM;

// 100 samples => Sine period = 100 * PIT period
static constexpr unsigned NUM_SAMPLES = 100;
static constexpr float PIT_PERIOD = 10 * us;

// Table of values for a sine wave
static uint16_t sineTable[NUM_SAMPLES];

using Dac = Dac0;

constexpr float PI = 3.14159265358979323846;

/**
 * @verbatim
 * +------------------------------+            Simple DMA mode (MLNO = Minor Loop Mapping Disabled)
 * | Major Loop =                 |            ==================================================
 * |    CITER x Minor Loop        |
 * |                              |            Each DMA request triggers a minor-loop transfer sequence.
 * | +--------------------------+ |<-DMA Req.  The minor loops are counted in the major-loop.
 * | | Minor Loop               | |
 * | | Each transfer            | |            The following are used during a minor loop:
 * | |   SADDR->DADDR           | |             - SADDR Source address
 * | |   SADDR += SOFF          | |             - SOFF  Adjustment applied to SADDR after each transfer
 * | |   DADDR += DOFF          | |             - DADDR Destination address
 * | | Total transfer is NBYTES | |             - DOFF  Adjustment applied to DADDR after each transfer
 * | +--------------------------+ |             - NBYTES Number of bytes to transfer
 * | +--------------------------+ |<-DMA Req.   - Attributes
 * | | Minor Loop               | |               - ATTR_SSIZE, ATTR_DSIZE Source and destination transfer sizes
 * |..............................|               - ATTR_SMOD, ATTR_DMOD Modulo
 * | |                          | |
 * | +--------------------------+ |             The number of reads and writes done will depend on NBYTES, SSIZE and DSIZE
 * | +--------------------------+ |<-DMA Req.   For example: NBYTES=12, SSIZE=16-bits, DSIZE=32-bits => 6 reads, 3 writes
 * | | Minor Loop               | |             NBYTES must be an even multiple of SSIZE and DSIZE in bytes.
 * | | Each transfer            | |
 * | |   SADDR->DADDR           | |            The following are used by the major loop
 * | |   SADDR += SOFF          | |             - SLAST Adjustment applied to SADDR at the end of each major loop
 * | |   DADDR += DOFF          | |             - DLAST Adjustment applied to DADDR at the end of each major loop
 * | | Total transfer is NBYTES | |             - CITER Major loop counter - counts how many completed major loops
 * | +--------------------------+ |
 * |                              |            SLAST and DLAST may be used to reset the addresses to the initial value or
 * | At end of Major Loop         |            link to the next transfer.
 * |    SADDR += SLAST            |            The total transferred for the entire sequence is CITER x NBYTES.
 * |    DADDR += DLAST            |
 * |                              |            Important options in the CSR:
 * | Total transfer =             |              - DMA_CSR_INTMAJOR = Generate interrupt at end of Major-loop
 * |    CITER*NBYTES              |              - DMA_CSR_DREQ     = Clear hardware request at end of Major-loop
 * +------------------------------+              - DMA_CSR_START    = Start transfer. Used for software transfers. Automatically cleared.
 * @endverbatim
 *
 * Structure to define the DMA transfer
 */
static constexpr DmaTcd tcd (
   /* Source address                 */ (uint32_t)sineTable,              // Source array
   /* Source offset                  */ sizeof(sineTable[0]),             // Source address advances source element size for each transfer
   /* Source size                    */ dmaSize(sineTable[0]),            // 16-bit read from source address
   /* Source modulo                  */ DmaModulo_Disabled,               // Disabled
   /* Last source adjustment         */ -(int)sizeof(sineTable),          // Reset Source address to start of array on completion

   /* Destination address            */ Dac::dacData(),                   // Destination
   /* Destination offset             */ 0,                                // No adjustment
   /* Destination size               */ DmaSize_16bit,                    // 16-bit write to destination address
   /* Destination modulo             */ DmaModulo_Disabled,               // Disabled
   /* Last destination adjustment    */ 0,                                // No adjustment

   /* Minor loop byte count          */ dmaNBytes(sizeof(sineTable[0])),  // 1 value transfered each minor-loop (triggered request)
   /* Major loop count               */ dmaCiter(sizeof(sineTable)/       // Transfer entire buffer in major loop
   /*                                */           sizeof(sineTable[0])),

   /* Start channel                  */ false,                            // Don't start (triggered by hardware)
   /* Disable Req. on major complete */ false,                            // Don't clear hardware request when major loop completed
   /* Interrupt on major complete    */ false,                            // No interrupt
   /* Interrupt on half complete     */ false,                            // No interrupt
   /* Bandwidth (speed) Control      */ DmaSpeed_NoStalls                 // Full speed
);

/**
 * Fill sineTable with scaled sine waveform
 */
void initSineTable() {

   for (unsigned index = 0; index<(sizeof(sineTable)/sizeof(sineTable[0])); index++) {
      sineTable[index] = roundf(Dac::getRange() * (1 + sin(index*2*PI/NUM_SAMPLES))/2);
   }
}

/**
 * DMA error call back
 *
 * @param errorFlags Channel error information (DMA_ES)
 */
void dmaErrorCallbackFunction(uint32_t errorFlags) {
   console.write("DMA error DMA_ES = 0b").writeln(errorFlags, Radix_2);
   __BKPT();
}

/**
 * Configure DMA from Memory-to-DAC
 *
 * @param dmaChannel  Pre-allocated DMA channel to use.
 */
static void configureDma(DmaChannelNum dmaChannel) {

   Dma0::setErrorCallback(dmaErrorCallbackFunction);
   Dma0::enableErrorInterrupts(dmaChannel);
   Dma0::enableNvicErrorInterrupt();

   // Connect DMA channel to continuous request
   DmaMux0::configure(dmaChannel, Dma0Slot_AlwaysEnabled60, DmaMuxEnable_Triggered);

   // Configure the transfer
   Dma0::configureTransfer(dmaChannel, tcd);

   // Enable hardware requests
   Dma0::enableRequests(dmaChannel);

#ifdef DMA_EARS_EDREQ_0_MASK
   // Enable asynchronous requests (if available)
   Dma0::enableAsynchronousRequests(dmaChannel);
#endif

}

/**
 * Configure the PIT
 * - Generates regular events which throttles the DMA transfer
 *
 * @param dmaChannel  PIT channel being used.  Must be associated with DMA channel.
 */
static void configurePit(PitChannelNum pitChannel) {
   // Configure base PIT
   Pit::configure(PitDebugMode_Stop);

   // Configure channel for 100ms + interrupts
   Pit::configureChannel(pitChannel, PIT_PERIOD, PitChannelIrq_Disabled);
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

   // Enable DMAC0
   // This also clears the channel reservations
   Dma0::configure();

   // Enable PIT
   // This also clears the channel reservations
   Pit::configure(PitDebugMode_Stop);

   // DMA channel number to use (determines which PIT channel used)
   static const DmaChannelNum dmaChannel = Dma0::allocatePeriodicChannel();
   if (dmaChannel == DmaChannelNum_None) {
      console.write("Failed to allocate DMA channel, rc= ").writeln(E_NO_RESOURCE);
      __BKPT();
   }
   console.write("Allocated DMA channel  #").writeln(dmaChannel);

   // Get Pit channel associated with DMA channel
   PitChannelNum pitChannel = Pit::allocateDmaAssociatedChannel(dmaChannel);
   if (pitChannel == PitChannelNum_None) {
      console.write("Failed to allocate PIT channel, rc= ").writeln(E_NO_RESOURCE);
      __BKPT();
   }
   console.write("Allocated PIT channel  #").writeln(pitChannel);

   configureDac();

   // Set up throttled DMA transfer
   configurePit(pitChannel);
   configureDma(dmaChannel);

   for(;;) {
      Smc::enterWaitMode();
   }
   return 0;
}
