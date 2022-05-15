/**
 ============================================================================
 * @file    dma-uart-example-mkl25.cpp (180.ARM_Peripherals/Snippets)
 * @brief   DMA example using UART and PIT throttling
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
/**
 * This example uses DMA to transfer characters from a string to the UART for transmission.
 * The speed of transmission is throttled by the use of PIT triggering the DMA channel (DmaMux).
 * The transmission is made continuous by setting up the TCD appropriately:
 * - Not clearing the DREQ on transfer complete
 * - Arranging SLAST to return the transfer addresses to starting value after each major-loop.
 *
 * This example requires interrupts to be enabled in the USBDM configuration for the following:
 * - DMA
 * - PIT
 *
 * The LED should be assigned to a suitable GPIO
 *
 * It may also be necessary to adjust DMA_SLOT for the console UART.
 *    DmaSlot_UART0_Transmit => DmaSlot_UART?_Transmit
 * 
 * If the console uses a LPUART then other changes are necessary:
 *    DmaSlot_UART0_Transmit => DmaSlot_LPUART?_Transmit
 *    UartDma_TxHoldingEmpty => LpuartDma_TxHoldingEmpty
 *    console.uart->D        => console.lpuart->DATA
 */
#include "hardware.h"
#include "dma.h"
#include "pit.h"
#include "smc.h"
#include "mcg.h"

using namespace USBDM;

// Slot number to use (must agree with console UART)
static constexpr DmaSlot DMA_SLOT = Dma0Slot_UART0_Tx;

// MCG clocks for various run modes
static constexpr ClockConfig VLPR_MODE  = ClockConfig_BLPE_4MHz;
static constexpr ClockConfig RUN_MODE   = ClockConfig_PEE_48MHz;

// Used to indicate complete transfer
static volatile bool complete;

static const char message[]=
      "=================================\n\r"
      " Hello world from DMA controller \n\r"
      "=================================\n\r";

/**
 * @verbatim
 * +------------------------------+  DMA mode
 * | Loop =                       |  ==================================================
 * | +--------------------------+ |
 * | | Each transfer            | |  The following are used during a loop:
 * | |   SADDR->DADDR           | |   - SADDR      Source address
 * | |   SADDR += DCR.SSIZE     | |   - DCR.SSIZE  Adjustment applied to SADDR after each transfer
 * | |   DADDR += DCR.DSIZE     | |   - DADDR      Destination address
 * | +--------------------------+ |   - DCR.DSIZE  Adjustment applied to DADDR after each transfer
 * |   Total transfer is BCR      |   - BCR        Number of bytes to transfer
 * +------------------------------+
 * @endverbatim
 *
 * Structure to define the DMA transfer
 */
    static const DmaTcd tcd (
      /* Transfer size                          */ sizeof(message),         // Total transfer in bytes
      /* Source address                         */ (uint32_t)(message),     // Source array
      /* Source size                            */ dmaSize(message[0]),     // 8-bit destination
      /* Source modulo                          */ DmaModulo_Disabled,
      /* Source increment                       */ true,                    // Increment source address
      /* Destination address                    */ console.uartD(),         // UART Data register
      /* Destination size                       */ dmaSize(message[0]),     // 8-bit source
      /* Destination modulo                     */ DmaModulo_Disabled,
      /* Destination increment                  */ false,
      /* DMA mode                               */ DmaMode_CycleSteal,      // Single transfer for each request
      /* Auto align                             */ false,
      /* Start transfer                         */ false,
      /* Enable asynchronous requests           */ true,                    // Asynchronous DMA
      /* Enable peripheral requests             */ true,                    // Requests from UART
      /* Disable peripheral request on complete */ true,                    // Disable peripheral requests (ERQ) at end
      /* Enable interrupts                      */ true                     // Interrupt when complete
   );


/**
 * DMA complete callback
 *
 * Sets flag to indicate sequence complete.
 */
static void dmaCallback(DmaChannelNum channel) {
   // Clear status
   Dma0::clearInterruptRequest(channel);
   // Stop UART requests
   console.enableDma(UartDma_TxHoldingEmpty, false);
   // Reconfigure for next transfer
   Dma0::configureTransfer(channel, tcd);
   // Flag complete to main-line
   complete = true;
}

/**
 * Configure DMA from Memory-to-UART
 *
 * @param dmaChannel  Pre-allocated DMA channel to use.
 */
static void configureDma(DmaChannelNum dmaChannel) {

   // Sequence not complete yet
   complete = false;

   // Enable DMAC with default settings
   Dma0::configure();

   // Set callback (Interrupts are enabled in TCD)
   Dma0::setCallback(dmaChannel, dmaCallback);
   Dma0::enableNvicInterrupts(dmaChannel, NvicPriority_Normal);

   // Connect DMA channel to UART but throttle by PIT Channel 1 (matches DMA channel 1)
   DmaMux0::configure(dmaChannel, DMA_SLOT, DmaMuxEnable_Triggered);

   // Configure the transfer
   Dma0::configureTransfer(dmaChannel, tcd);
}

/**
 * Configure the PIT
 * - Generates regular events which throttles the DMA -> UART Tx.
 *
 * @param dmaChannel  DMA channel being used, determines PIT
 */
static void configurePit(DmaChannelNum dmaChannel) {
   // Configure base PIT
   Pit::configure(PitDebugMode_Stop);

   // Configure channel for 100ms + interrupts
   Pit::configureChannel(dmaChannel, 100_ms, PitChannelIrq_Enabled);
}

/**
 * Change run mode
 *
 * @param[in] smcRunMode Run mode to enter
 */
void changeRunMode(SmcRunMode smcRunMode) {
   // Get current run mode
   SmcStatus smcStatus = Smc::getStatus();

   // Check if transition needed
   if (((smcStatus == SmcStatus_RUN) && (smcRunMode == SmcRunMode_Normal)) ||
       ((smcStatus == SmcStatus_VLPR) && (smcRunMode == SmcRunMode_VeryLowPower))) {
      return;
   }
   if (smcStatus == SmcStatus_VLPR) {
      // Do VLPR->RUN mode
      Smc::enterRunMode(SmcRunMode_Normal);
      Mcg::configure(RUN_MODE);
      console.setBaudRate(defaultBaudRate);
      console.write("Changed to RUN mode, ").flushOutput();
   }

   // Now in RUN mode
   switch(smcRunMode) {
      case SmcRunMode_Normal:
         // Complete
         break;

      case SmcRunMode_VeryLowPower:
         // RUN->VLPR
         Mcg::configure(VLPR_MODE);
         Smc::enterRunMode(SmcRunMode_VeryLowPower);
         console.setBaudRate(defaultBaudRate);
         console.write("Changed to VLPR mode, ").flushOutput();
         break;
   }

   console.writeln(Smc::getSmcStatusName(), ":", Mcg::getClockModeName(), "@", ::SystemCoreClock);
}

int main() {

   SimInfo::setUart0Clock(SimUart0ClockSource_OscerClk);
   console.setBaudRate(defaultBaudRate);

   console.writeln("\nStarting\n").flushOutput();

   // Allow entry to other RUN modes
   Smc::enablePowerModes(
         SmcVeryLowPower_Enabled,
         SmcLowLeakageStop_Enabled,
         SmcVeryLowLeakageStop_Enabled);

   // DMA channel number to use (determines which PIT channel used)
   static const DmaChannelNum dmaChannel = Dma0::allocatePeriodicChannel();
   if (dmaChannel == DmaChannelNum_None) {
      console.writeln("Failed to allocate DMA channel, rc= ", E_NO_RESOURCE);
      __asm__("bkpt");
   }
   console.writeln("Allocated DMA channel  #", dmaChannel);

   // Set up throttled DMA transfer from memory -> UART
   configureDma(dmaChannel);

   // Get Pit channel associated with DMA channel
   PitChannelNum pitChannel = Pit::allocateDmaAssociatedChannel(dmaChannel);
   if (pitChannel == PitChannelNum_None) {
      console.writeln("Failed to allocate PIT channel, rc= ", E_NO_RESOURCE);
      __asm__("bkpt");
   }
   console.writeln("Allocated PIT channel  #", pitChannel);
   configurePit(pitChannel);

   // Start the UART DMA requests
   console.writeln("Doing 1 DMA transfer while in RUN").flushOutput();
   console.enableDma(UartDma_TxHoldingEmpty);

   // Wait for completion of 1 message
   while (!complete) {
      __asm__("nop");
   }
   console.writeln("Done 1st transfer");
   waitMS(500);

   // RUN->VLPR
   changeRunMode(SmcRunMode_VeryLowPower);

   // Re-configure PIT as bus clock may have changed
   configurePit(pitChannel);

   // Start the UART DMA requests again
   complete = false;
   console.writeln("\nDoing DMA while in VLPR....").flushOutput();
   console.enableDma(UartDma_TxHoldingEmpty);

   // Wait for completion of 1 message
   while (!complete) {
      __asm__("nop");
   }
   console.writeln("Done 2nd transfer");
   waitMS(500);

   Smc::setStopOptions(
         SmcLowLeakageStopMode_VLLS3,   // Retains RAM
         SmcPowerOnReset_Enabled,        // Brown-out detection
         SmcPartialStopMode_Normal,     // No bus clock in stop!
         // SmcPartialStopMode_Partial1 - Bus clock active (for DMAC)
         SmcLpoInLowLeakage_Disabled);   // LPO stops in LLS/VLLS

   console.writeln("\nDoing DMA while sleeping....").flushOutput();

   for(;;) {
      // Enable UART Tx DMA requests
      console.enableDma(UartDma_TxHoldingEmpty);

      Smc::enterWaitMode();
//      Smc::enterStopMode(SmcStopMode_NormalStop); // Only if chip supports SmcPartialStopMode_Partial1

      // Will wake up after each complete transfer due to DMA complete interrupt
      console.writeln("Woke up!").flushOutput();
   }
   return 0;
}
