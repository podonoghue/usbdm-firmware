/*
 ============================================================================
 * @file    spi-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo of using SPI interface
 *
 *  Created on: 10/6/2017
 *      Author: podonoghue
 ============================================================================
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
 *  - SPI0_PCS2
 */
#include <string.h>
#include "spi.h"

using namespace USBDM;

int main() {
   Spi0 spi{};

   // Configure SPI parameters for odd transmissions
   spi.setSpeed(1*MHz);
   spi.setMode(SpiMode_0);
   spi.setPeripheralSelect(SpiPeripheralSelect_0, ActiveLow, SpiSelectMode_Idle);
   spi.setFrameSize(8);

   // Save configuration
   SpiConfig configurationOdd = spi.getConfiguration();

   // Configure SPI parameters for even transmissions
   spi.setSpeed(10*MHz);
   spi.setMode(SpiMode_0);
   spi.setPeripheralSelect(SpiPeripheralSelect_2, ActiveLow, SpiSelectMode_Idle);
   spi.setFrameSize(12);

   // Save configuration
   SpiConfig configurationEven = spi.getConfiguration();

   for(;;) {
      {
         /*
          * Odd transmission
          *
          * Transmit with configuration 1
          * 8-bit transfers @ 1 MHz using CS0
          */
         static const uint8_t txDataA[] = { 0xA1,0xB2,0xC3,0xD4,0x55, };
         uint8_t rxData1[sizeof(txDataA)/sizeof(txDataA[0])] = {0};
         uint8_t rxData2[sizeof(txDataA)/sizeof(txDataA[0])] = {0};
         uint8_t rxData3 = 0;
         uint8_t rxData4 = 0;

         spi.startTransaction(configurationOdd);
         spi.txRx(sizeof(txDataA)/sizeof(txDataA[0]), txDataA, rxData1); // 5 bytes
         spi.txRx(sizeof(txDataA)/sizeof(txDataA[0]), txDataA, rxData2); // 5 bytes
         rxData3 = spi.txRx(txDataA[0]); // 1 byte
         rxData4 = spi.txRx(txDataA[1]); // 1 byte
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
          * Transmit with configuration 2
          * 12-bit transfers @ 10 MHz using CS2
          */
         static const uint16_t txDataB[] = { 0xA01,0xB02,0xC03,0xD04,0x555, };
         uint16_t rxData1[sizeof(txDataB)/sizeof(txDataB[0])] = {0};
         uint16_t rxData2[sizeof(txDataB)/sizeof(txDataB[0])] = {0};
         uint16_t rxData3 = 0;;
         uint16_t rxData4 = 0;

         spi.startTransaction(configurationEven);
         spi.txRx(sizeof(txDataB)/sizeof(txDataB[0]), txDataB, rxData1); // 5 bytes
         spi.txRx(sizeof(txDataB)/sizeof(txDataB[0]), txDataB, rxData2); // 5 bytes
         rxData3 = spi.txRx(txDataB[0]); // 1 byte
         rxData4 = spi.txRx(txDataB[1]); // 1 byte
         spi.endTransaction();

         if ((memcmp(txDataB, rxData1, sizeof(txDataB)/sizeof(txDataB[0])) != 0) ||
             (memcmp(txDataB, rxData2, sizeof(txDataB)/sizeof(txDataB[0])) != 0) ||
             (rxData3 != txDataB[0]) ||
             (rxData4 != txDataB[1])) {
            console.writeln("Failed read-back");
            __asm__("bkpt");
         }
      }
      wait(10*ms);
   }

}
