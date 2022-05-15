/*
 ============================================================================
 * @file    spi-example-MKL.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo of using SPI interface
 *
 *  Created on: 10/6/2017
 *      Author: podonoghue
 ============================================================================
 */
/*
 * This example requires a loop-back between SPI_MOSI and SPI_MISO.
 * It may be necessary to adjust the peripheral selection to an available pin.
 */
#include <string.h>
#include "spi.h"

using namespace USBDM;

int main() {

   static Spi0 spi{};

   // Configure SPI
   spi.setSpeed(10*MHz);
   spi.setMode(SpiMode_0);
   spi.setSlaveSelect(SpiPinSelect_Enable);

   // Save configuration
   SpiConfig configuration1 = spi.getConfiguration();

   // Configure SPI
   spi.setSpeed(24.0*MHz);
   spi.setMode(SpiMode_3);
   spi.setSlaveSelect(SpiPinSelect_Enable);

   static bool complete;
   static auto cb = [](uint32_t status) {
      if (status != 0) {
         console.write("TxRx failed, status=").writeln(status, Radix_2);
      }
      complete = true;
   };
   spi.setCallback(cb);

   // Save configuration
   SpiConfig configuration2 = spi.getConfiguration();

   for(;;) {
      {
         /*
          * Transmit with configuration 1
          * 8-bit transfers @ 10 MHz
          */
         static const uint8_t txDataA[] = { 0xA1,0xB2,0xC3,0xD4,0xE5, };
         uint8_t rxData1[sizeof(txDataA)/sizeof(txDataA[0])] = {0};
         uint8_t rxData2[sizeof(txDataA)/sizeof(txDataA[0])] = {0};

         spi.startTransaction(configuration1);
         spi.txRx(sizeof(txDataA)/sizeof(txDataA[0]), txDataA, rxData1);
         spi.txRx(sizeof(txDataA)/sizeof(txDataA[0]), txDataA, rxData2);
         spi.endTransaction();

         if ((memcmp(txDataA, rxData1, sizeof(txDataA)/sizeof(txDataA[0])) != 0) ||
             (memcmp(txDataA, rxData2, sizeof(txDataA)/sizeof(txDataA[0])) != 0)) {
            console.writeln("Failed read-back");
//            __asm__("bkpt");
         }
         else {
            console.writeln("Read-back OK");
         }
      }
      {
         /*
          * Transmit with configuration 1
          * 8-bit transfers @ 24 MHz
          * Using call-back on completion (silly example!)
          */
         static const uint8_t txDataB[] = { 0x01,0x02,0x03,0x04,0x05, };
         uint8_t rxData1[sizeof(txDataB)/sizeof(txDataB[0])] = {0};
         uint8_t rxData2[sizeof(txDataB)/sizeof(txDataB[0])] = {0};

         spi.startTransaction(configuration2);
         complete = false;
         spi.txRx(sizeof(txDataB)/sizeof(txDataB[0]), txDataB, rxData1);
         while(!complete) {
         }
         complete = false;
         spi.txRx(sizeof(txDataB)/sizeof(txDataB[0]), txDataB, rxData2);
         while(!complete) {
         }
         spi.endTransaction();

         if ((memcmp(txDataB, rxData1, sizeof(txDataB)/sizeof(txDataB[0])) != 0) ||
             (memcmp(txDataB, rxData2, sizeof(txDataB)/sizeof(txDataB[0])) != 0)) {
            console.writeln("Failed read-back");
//            __asm__("bkpt");
         }
         else {
            console.writeln("Read-back OK");
         }
      }
      wait(10_ms);
   }

}
