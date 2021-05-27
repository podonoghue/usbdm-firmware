/**
 ============================================================================
 * @file     mag3310-example.cpp
 * @brief    Demonstrates use of MAG3310 Magnetometer over I2C
 * @version  V4.11.1.80
 * @author   podonoghue
 * @note You may need to change the pin-mapping of the I2C interface
============================================================================
 */
#include <math.h>
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "i2c.h"
#include "mag3310.h"

// Allows access to USBDM library name-space
using namespace USBDM;

/*************************************************
 * Global objects representing hardware
 **************************************************/

// I2C interface
I2c0     i2c0;

// Magnetometer via I2C
MAG3310  magnetometer(i2c0);

/**************************************************/

/**
 * Report magnetometer values
 *
 * @param magnetometer Magnetometer to use
 */
void report(MAG3310 &magnetometer) {
      int magStatus;
      int16_t magX,magY,magZ;
      magnetometer.readMagnetometerXYZ(magStatus, magX, magY, magZ);
      printf("s=0x%02X, mX=%10d, mY=%10d, mZ=%10d, ", magStatus,   magX,   magY,   magZ);
      // Assumes the sensor is level
      printf("a=%d\n", (int)(180*atan2(magX, magY)/M_PI));
}

int main() {
   printf("Starting\n");

   uint8_t id = magnetometer.readID();
   printf("Device ID = 0x%02X (should be 0xC4)\n", id);

   printf("Calibrating magnetometer\n"
          "Please rotate the board in all dimensions until complete (~20 s)\n");
   magnetometer.calibrateMagnetometer();

   for(;;) {
      report(magnetometer);
      waitMS(120);
   }
}
