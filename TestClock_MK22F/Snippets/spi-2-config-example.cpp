/*
 ==============================================================================
 * @file    spi-2-config-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   C++ demo using SPI interface
 *
 * This example sets up two hardware configurations and switches between them.
 * This has low overhead but is limited by the hardware.
 *   - Two configurations (assuming 2 SPI CTARs)
 *   - These configurations may be used with multiple devices (limited by PCSs)
 *
 *  Created on: 10/12/2021
 *      Author: podonoghue
 ==============================================================================
 */
/*
 * This example requires a loop-back between SPI_MOSI and SPI_MISO.
 * It may be necessary to adjust the peripheral selection to an available pin.
 *
 * The SPI signals used must be mapped to pins in Configure.usbdmProject
 *  - SPI0_SCK
 *  - SPI0_SIN
 *  - SPI0_SOUT
 *  - SPI0_PCS0
 *  - SPI0_PCS1
 */
#include <string.h>
#include "hardware.h"
#include "spi.h"

using namespace USBDM;

static const SpiConfigurations configurations {
   //                              Speed      Mode        Bit Order           Frame Size
   /* Configuration 0 (CTAR0) */ { 1'000'000, SpiMode_0, SpiOrder_MsbFirst, SpiFrameSize_8},
   /* Configuration 1 (CTAR1) */ {10'000'000, SpiMode_0, SpiOrder_MsbFirst, SpiFrameSize_12},
   /* PCS idle levels         */ SpiPeripheralSelect_None, // All PCSs idle low
};

int main() {
   Spi0 spi{};

   // Configure two configurations (CTARs) and the idle levels for PCSs
   spi.setConfigurations(configurations);

   for(;;) {
      {
         /*
          * Odd transmission
          *
          * Transmit with configuration 0
          * 8-bit transfers @ 1 MHz using PCS0
          */
         static const uint8_t txDataA[] = { 0xA1,0xB2,0xC3,0xD4,0x55, };
         uint8_t rxData1[sizeof(txDataA)/sizeof(txDataA[0])] = {0};
         uint8_t rxData2[sizeof(txDataA)/sizeof(txDataA[0])] = {0};
         uint8_t rxData3 = 0;
         uint8_t rxData4 = 0;

         spi.startTransaction(SpiPeripheralSelect_0, SpiCtarSelect_0);
         spi.txRx(txDataA, rxData1); // 5 bytes tx-rx
         spi.txRx(txDataA, rxData2); // 5 bytes tx-rx
         rxData3 = spi.txRx(txDataA[0]); // 1 byte tx-rx
         rxData4 = spi.txRx(txDataA[1]); // 1 byte tx-rx
         spi.endTransaction();

         if ((memcmp(txDataA, rxData1, sizeof(txDataA)/sizeof(txDataA[0])) != 0) ||
             (memcmp(txDataA, rxData2, sizeof(txDataA)/sizeof(txDataA[0])) != 0) ||
             (rxData3 != txDataA[0]) ||
             (rxData4 != txDataA[1])) {
            console.writeln("Failed read-back");
            __asm__("bkpt");
         }
      }
      {
         /*
          * Even transmission
          *
          * Transmit with configuration 1
          * 12-bit transfers @ 10 MHz using PCS2
          */
         static const uint16_t txDataB[] = { 0xA01,0xB02,0xC03,0xD04,0x555, };
         uint16_t rxData1[sizeof(txDataB)/sizeof(txDataB[0])] = {0};
         uint16_t rxData2[sizeof(txDataB)/sizeof(txDataB[0])] = {0};
         uint16_t rxData3 = 0;;
         uint16_t rxData4 = 0;

         spi.startTransaction(SpiPeripheralSelect_1, SpiCtarSelect_1);
         spi.txRx(txDataB, rxData1); // 5 bytes tx-rx
         spi.txRx(txDataB, rxData2); // 5 bytes tx-rx
         rxData3 = spi.txRx(txDataB[0]); // 1 byte tx-rx
         rxData4 = spi.txRx(txDataB[1]); // 1 byte tx-rx
         spi.endTransaction();

         if ((memcmp(txDataB, rxData1, sizeof(txDataB)/sizeof(txDataB[0])) != 0) ||
             (memcmp(txDataB, rxData2, sizeof(txDataB)/sizeof(txDataB[0])) != 0) ||
             (rxData3 != txDataB[0]) ||
             (rxData4 != txDataB[1])) {
            console.writeln("Failed read-back");
            __asm__("bkpt");
         }
      }
      wait(10_ms);
   }

}
