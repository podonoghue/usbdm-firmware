/**
 ============================================================================
 * @file     mma845x-example.cpp
 * @brief    Demonstrates use of MMA845x Accelerometer over I2C
 * @version  V4.11.1.80
 * @author   podonoghue
 *
 * You may need to change the pin-mapping of the I2C interface
============================================================================
 */
#include <math.h>
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "i2c.h"
#include "mma845x.h"
#include "delay.h"

// Allows access to USBDM library name-space
using namespace USBDM;

/*************************************************
 * Global objects representing hardware
 **************************************************/

// I2C interface
I2c0     i2c0;

// Accelerometer via I2C
MMA845x  accelerometer(i2c0, MMA845x::AccelerometerMode_2Gmode);

/**************************************************/

/**
 * Report accelerometer values
 *
 * @param[in] accelerometer Accelerometer to use
 */
void report(MMA845x &accelerometer) {
   int accelStatus;
   int16_t accelX,accelY,accelZ;

   accelerometer.readAccelerometerXYZ(accelStatus, accelX, accelY, accelZ);
   console.setPadding(Padding_LeadingZeroes).setWidth(2).
         write("s=0x").write(accelStatus,Radix_16).
         setPadding(Padding_LeadingSpaces).setWidth(10).
         write(", aX=").write(accelX).
         write(", aY=").write(accelY).
         write(", aZ=").writeln(accelZ);
}

int main() {
   console.writeln("Starting\n");

   console.write("Device ID = 0x").write(accelerometer.readID(), Radix_16).writeln("(should be 0x1A)");

   // Check if any USBDM error yet (constructors)
   checkError();

   report(accelerometer);

   console.write("Doing simple calibration\n"
         "Make sure the device is level!\n");
   waitMS(2000);

   if (accelerometer.calibrateAccelerometer() != E_NO_ERROR) {
      console.write("Calibration failed!\n");
      __asm__("bkpt");
   }

   // Make sure we have new values
   waitMS(100);

   console.write("After calibration\n");
   for(;;) {
      report(accelerometer);
      waitMS(400);
   }
}

