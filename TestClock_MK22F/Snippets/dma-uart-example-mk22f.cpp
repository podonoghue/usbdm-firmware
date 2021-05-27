/**
 ============================================================================
 * @file    dma-uart-example-mk22f.cpp (180.ARM_Peripherals/Snippets)
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
 *
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
static constexpr DmaSlot DMA_SLOT = Dma0Slot_UART1_Tx;

// MCG clocks for various run modes
static constexpr ClockConfig VLPR_MODE  = ClockConfig_BLPE_4MHz;
static constexpr ClockConfig RUN_MODE   = ClockConfig_PEE_80MHz;
static constexpr ClockConfig HSRUN_MODE = ClockConfig_PEE_120MHz;

// Used to indicate complete transfer
static volatile bool complete;

static const char message[]=
      "=================================\n\r"
      " Hello world from DMA controller \n\r"
      "=================================\n\r";

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
static constexpr DmaTcd tcd = DmaTcd (
   /* Source address                 */ (uint32_t)(message),           // Source array
   /* Source offset                  */ sizeof(message[0]),            // Source address advances 1 element for each request
   /* Source size                    */ dmaSize(message[0]),           // 8-bit read from source address
   /* Source modulo                  */ DmaModulo_Disabled,            // Disabled
   /* Last source adjustment         */ -(int)sizeof(message),         // Reset source address to start of array on completion

   /* Destination address            */ console.uartD(),               // Destination is UART data register
   /* Destination offset             */ 0,                             // Destination address doesn't change
   /* Destination size               */ DmaSize_8bit,                  // 8-bit write to destination address
   /* Destination modulo             */ DmaModulo_Disabled,            // Disabled
   /* Last destination adjustment    */ 0,                             // Destination address doesn't change

   /* Minor loop byte count          */ dmaNBytes(sizeof(message[0])), // Total transfer in one minor-loop
   /* Major loop count               */ dmaCiter(sizeof(message)/      // Transfer entire buffer
   /*                                */           sizeof(message[0])),

   /* Start channel                  */ false,                         // Don't start (triggered by hardware)
   /* Disable Req. on major complete */ false,                         // Don't clear hardware request when major loop completed
   /* Interrupt on major complete    */ true,                          // Generate interrupt on completion of Major-loop
   /* Interrupt on half complete     */ false,                         // No interrupt
   /* Bandwidth (speed) Control      */ DmaSpeed_NoStalls              // Full speed
);

/**
 * DMA error call back
 *
 * @param errorFlags Channel error information (DMA_ES)
 */
void dmaErrorCallbackFunction(uint32_t errorFlags) {
   console.write("DMA error DMA_ES = 0x").writeln(errorFlags, Radix_2);
   __BKPT();
}

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

   // Enable DMAC
   // Note: These settings affect all DMA channels
   Dma0::configure(
         DmaOnError_Halt,
         DmaMinorLoopMapping_Disabled,
         DmaContinuousLink_Disabled,
         DmaArbitration_Fixed);

   // Set callback (Interrupts are enabled in TCD)
   Dma0::setCallback(dmaChannel, dmaCallback);
   Dma0::setErrorCallback(dmaErrorCallbackFunction);
   Dma0::enableNvicInterrupts(dmaChannel, NvicPriority_Normal);
   Dma0::enableNvicErrorInterrupt();

   // Connect DMA channel to UART but throttle by PIT Channel N (matches DMA channel N)
   DmaMux0::configure(dmaChannel, DMA_SLOT, DmaMuxEnable_Triggered);

   // Configure the transfer
   Dma0::configureTransfer(dmaChannel, tcd);

   // Enable hardware requests
   Dma0::enableRequests(dmaChannel);

#ifdef DMA_EARS_EDREQ_0_MASK
   // Enable asynchronous requests (if available)
   Dma0::enableAsynchronousRequests(dmaChannel);
#endif

   // Enable channel interrupt requests
   Dma0::enableErrorInterrupts(dmaChannel);
}

/**
 * Configure the PIT
 * - Generates regular events which throttles the DMA -> UART Tx.
 *
 * @param dmaChannel  PIT channel being used.  Must be associated with DMA channel.
 */
static void configurePit(PitChannelNum pitChannel) {
   // Configure base PIT
   Pit::configure(PitDebugMode_Stop);

   // Configure channel for 100ms + interrupts
   Pit::configureChannel(pitChannel, 100*ms, PitChannelIrq_Enabled);
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
   if (((smcStatus == SmcStatus_hsrun) && (smcRunMode == SmcRunMode_HighSpeed)) ||
       ((smcStatus == SmcStatus_RUN) && (smcRunMode == SmcRunMode_Normal)) ||
       ((smcStatus == SmcStatus_VLPR) && (smcRunMode == SmcRunMode_VeryLowPower))) {
      return;
   }
   // If changing go via RUN
   if (smcStatus == SmcStatus_hsrun) {
      // Do HSRUN->RUN
      Mcg::configure(RUN_MODE);
      Smc::enterRunMode(SmcRunMode_Normal);
      console.setBaudRate(defaultBaudRate);
      console.write("Changed to RUN mode, ").flushOutput();
   }
   else if (smcStatus == SmcStatus_VLPR) {
      // Do VLPR->RUN mode
      Smc::enterRunMode(SmcRunMode_Normal);
      Mcg::configure(RUN_MODE);
      console.setBaudRate(defaultBaudRate);
      console.write("Changed to RUN mode, ").flushOutput();
   }

   // Now in RUN mode
   switch(smcRunMode) {
      case SmcRunMode_HighSpeed:
         // RUN->HSRUN
         Smc::enterRunMode(SmcRunMode_HighSpeed);
         Mcg::configure(HSRUN_MODE);
         console.setBaudRate(defaultBaudRate);
         console.write("Changed to HSRUN mode, ").flushOutput();
         break;

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

   console.write(Smc::getSmcStatusName()).
         write(":").
         write(Mcg::getClockModeName()).
         write("@").writeln(::SystemCoreClock);
}

int main() {

   console.writeln("\nStarting\n").flushOutput();

   // Allow entry to other RUN modes
   Smc::enablePowerModes(
         SmcVeryLowPower_Enabled,
         SmcLowLeakageStop_Enabled,
         SmcVeryLowLeakageStop_Enabled,
         SmcHighSpeedRun_Enabled);

   // DMA channel number to use (determines which PIT channel used)
   static const DmaChannelNum dmaChannel = Dma0::allocatePeriodicChannel();
   if (dmaChannel == DmaChannelNum_None) {
      console.write("Failed to allocate DMA channel, rc= ").writeln(E_NO_RESOURCE);
      __asm__("bkpt");
   }
   console.write("Allocated DMA channel  #").writeln(dmaChannel);

   // Set up throttled DMA transfer from memory -> UART
   configureDma(dmaChannel);

   // Get Pit channel associated with DMA channel
   PitChannelNum pitChannel = Pit::allocateDmaAssociatedChannel(dmaChannel);
   if (pitChannel == PitChannelNum_None) {
      console.write("Failed to allocate PIT channel, rc= ").writeln(E_NO_RESOURCE);
      __asm__("bkpt");
   }
   console.write("Allocated PIT channel  #").writeln(pitChannel);
   configurePit(pitChannel);

   // Start the UART DMA requests
   console.writeln("Doing 1 DMA transfer while in RUN").flushOutput();
   console.enableDma(UartDma_TxHoldingEmpty);

   // Wait for completion of 1 Major-loop = 1 message
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

   // Wait for completion of 1 Major-loop = 1 message
   while (!complete) {
      __asm__("nop");
   }
   console.writeln("Done 2nd transfer");
   waitMS(500);

   Smc::setStopOptions(
         SmcLowLeakageStopMode_LLS3,   // Retains RAM
         SmcPowerOnReset_Enabled,       // Brown-out detection
         SmcPartialStopMode_Partial2,  // Bus clock active (for DMAC)
         SmcLpoInLowLeakage_Disabled);  // LPO stops in LLS/VLLS

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
