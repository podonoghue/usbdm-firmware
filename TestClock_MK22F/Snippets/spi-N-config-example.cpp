/*
 ================================================================================
 * @file    spi-N-config-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo of using SPI interface
 *
 *  This example re-uses configuration 0 with different calculated configurations
 *  This approach is useful if more configuration are needed that the number
 *  supported by the hardware (usually 2 CTARs).
 *
 *  Created on: 10/6/2017
 *      Author: podonoghue
 =================================================================================
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

// Configurations to use
// These are converted to calculated configurations to reduce overhead
static const SpiConfiguration configuration1{ 1'000'000, SpiMode_0, SpiOrder_MsbFirst, SpiFrameSize_8};
static const SpiConfiguration configuration2{ 1'000'000, SpiMode_0, SpiOrder_MsbFirst, SpiFrameSize_12};

int main() {
   Spi0 spi{};

   // Configure SPI parameters for odd transmissions

   // This method is time consuming as it calculates several derived parameters
   spi.setConfiguration(configuration1);

   // Peripheral settings to use with above configuration
   spi.setPeripheralSelect(SpiPeripheralSelect_0, ActiveLow, SpiSelectMode_Idle, SpiCtarSelect_0);

   // Save the derived configuration
   SpiCalculatedConfiguration configurationOdd = spi.getConfiguration();

   // Configure SPI parameters for even transmissions

   // This method is time consuming as it calculates several derived parameters
   spi.setConfiguration(configuration2);

   // Peripheral settings to use with above configuration
   spi.setPeripheralSelect(SpiPeripheralSelect_1, ActiveLow, SpiSelectMode_Idle, SpiCtarSelect_0);

   // Save the derived configuration
   SpiCalculatedConfiguration configurationEven = spi.getConfiguration();

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
          * Transmit with configuration 2
          * 12-bit transfers @ 10 MHz using CS2
          */
         static const uint16_t txDataB[] = { 0xA01,0xB02,0xC03,0xD04,0x555, };
         uint16_t rxData1[sizeof(txDataB)/sizeof(txDataB[0])] = {0};
         uint16_t rxData2[sizeof(txDataB)/sizeof(txDataB[0])] = {0};
         uint16_t rxData3 = 0;;
         uint16_t rxData4 = 0;

         spi.startTransaction(configurationEven);
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
