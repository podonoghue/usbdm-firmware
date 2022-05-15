/*
 ============================================================================
 * @file    i2c-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo of using I2C interface
 *
 *  Created on: 10/6/2017
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "i2c.h"

using namespace USBDM;

// Address (LSB = R/W bit)
static const unsigned I2C_ADDRESS = 0x1D<<1;
static const unsigned I2C_SPEED   = 400_kHz;

   // Declare I2C interface
   I2c0 i2c{I2C_SPEED, I2cMode_Polled};

int main() {
#define SELECT 2

   for(;;) {
#if SELECT == 0
      {
         /*
          * Transmits 4 bytes and receives 2 bytes using different buffers
          * This will use a repeated start at turnover
          */
         static const uint8_t txData[] = { 0xA1,0xB2,0xC3,0xD4,};
         uint8_t rxData[2] = {};

         i2c.startTransaction();
         i2c.txRx(I2C_ADDRESS, txData, rxData);
         i2c.endTransaction();
      }
#elif SELECT == 1
      {
         /*
          * Transmits 2 bytes and receives 4 bytes into same buffer
          * This will use a repeated start at turnover
          */
         uint8_t data[4] = { 0xA1,0xB2, };

         i2c.startTransaction();
         i2c.txRx(I2C_ADDRESS, 2, sizeof(data), data);
         i2c.endTransaction();
      }
#elif SELECT == 2
      {
         /*
          * Transmits 4 bytes
          */
         static const uint8_t data[] = { 0xA1,0xB2,0xC3,0xD4,};

         i2c.startTransaction();
         i2c.transmit(I2C_ADDRESS, data);
         i2c.endTransaction();
      }
#else
      {
         /*
          * Receive 4 bytes
          */
         uint8_t data[4] = {};

         i2c.startTransaction();
         i2c.receive(I2C_ADDRESS, data);
         i2c.endTransaction();
      }
#endif
      waitMS(100);
   }
}


