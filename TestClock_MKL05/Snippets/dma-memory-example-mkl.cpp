/**
 ============================================================================
 * @file    dma-memory-example-mkl.cpp (180.ARM_Peripherals/Sources/)
 * @brief   Basic C++ demo using GPIO class
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
/**
 * This example uses DMA interrupts.
 *
 * It is necessary to enable these in Configure.usbdmProject
 * under the "Peripheral Parameters"->DMA tab.
 * Select irqHandlers option (Class Method - Software ...)
 */
#include "hardware.h"
#include "dma.h"

#include "stdlib.h"
#include "string.h"

using namespace USBDM;

// Used to indicate complete transfer
static volatile bool complete;

/**
 * DMA complete callback
 *
 * Sets flag to indicate sequence complete.
 */
static void dmaCallback(DmaChannelNum channel) {
   Dma0::clearInterruptRequest(channel);
   complete = true;
}

/**
 * DMA Memory-to-memory transfer
 *
 * @param[in]  source         Source location
 * @param[out] destination    Destination location
 * @param[in]  size           Number of bytes to transfer - must be multiple of uint32_t size
 */
static ErrorCode dmaTransfer(uint32_t *source, uint32_t *destination, uint32_t size) {

   usbdm_assert(size%sizeof(uint32_t) == 0, "size must be a multiple of sizeof(uint32_t)");

   // DMA channel number to use
   const DmaChannelNum dmaChannelNum = Dma0::allocateChannel();

   if (dmaChannelNum == DmaChannelNum_None) {
      return E_NO_RESOURCE;
   }

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
    * +------------------------------+   - DCR.START  Start transfer. Used for software transfers.
    * @endverbatim
    *
    * Structure to define the DMA transfer
    */
   static const DmaTcd tcd (
      /* Transfer size                          */ size,                    // Total transfer size in bytes
      /* Source address                         */ (uint32_t)(source),      // Source array
      /* Source size                            */ dmaSize(*source),        // 32-bit source
      /* Source modulo                          */ DmaModulo_Disabled,
      /* Source increment                       */ true,                    // Increment source address
      /* Destination address                    */ (uint32_t)(destination), // Start of array for result
      /* Destination size                       */ dmaSize(*destination),   // 32-bit destination
      /* Destination modulo                     */ DmaModulo_Disabled,
      /* Destination increment                  */ true,                    // Increment destination address
      /* DMA mode                               */ DmaMode_Continuous,      // All data for each request
      /* Auto align                             */ false,
      /* Start transfer                         */ true,                    // Start transfer immediately
      /* Enable asynchronous requests           */ true,                    // Asynchronous DMA
      /* Enable peripheral requests             */ false,
      /* Disable peripheral request on complete */ false,
      /* Enable interrupts                      */ true                     // Interrupt when complete
   );

   // Sequence not complete yet
   complete = false;

   // Enable DMAC with default settings
   Dma0::configure();

   // Set callback (Interrupts are enabled in TCD)
   Dma0::setCallback(dmaChannelNum, dmaCallback);
   Dma0::enableNvicInterrupts(dmaChannelNum, NvicPriority_Normal);

   // Configure the transfer
   Dma0::configureTransfer(dmaChannelNum, tcd);

   while (!complete) {
      __asm__("nop");
   }
   return E_NO_ERROR;
}

constexpr int DataSize = 4*((1<<10)/sizeof(uint32_t));  // 4KiB
uint32_t source[DataSize];
uint32_t destination[DataSize];

int main() {
   console.writeln("Starting");

#if 0
   // For testing channel allocation
   for(;;) {
      DmaChannelNum ch = Dma0::allocateChannel();
      if (ch == DmaChannelNum_None) {
         break;
      }
      console.writeln("Channel allocated = ", ch);
   }
#endif

   for(uint32_t *p=source; p<(source+DataSize); p++) {
      *p = (uint32_t)rand();
   }
   console.writeln("Source buffer contents");
   console.writeArray(source, sizeof(source)/sizeof(source[0]));

   console.writeln("Starting Transfer");
   ErrorCode rc = dmaTransfer(source, destination, sizeof(source));
   console.writeln("Completed Transfer rc = ", getErrorMessage(rc));

   if (rc == E_NO_ERROR) {
      console.writeln("Destination buffer contents");
      console.writeArray(destination, sizeof(destination)/sizeof(destination[0]));

      if (memcmp(source,destination,sizeof(source)) == 0) {
         console.writeln("Contents verify passed");
      }
      else {
         console.writeln("Contents verify failed");
      }
   }
   for(;;) {
      __asm__("nop");
   }
   return 0;
}
