/**
 ================================================================================
 * @file   fxos8700cq-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief  Demonstrates use of FXOS8700CQ Accelerometer and Magnetometer over I2C
 * @version  V4.11.1.90
 * @author   podonoghue
 * @note You may need to change the pin-mapping of the I2C interface
=================================================================================
 */
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "i2c.h"
#include "fxos8700cq.h"
#include "delay.h"

// Allows access to USBDM library name-space
using namespace USBDM;

/*************************************************
 * Global objects representing hardware
 **************************************************/

// I2C interface
I2c0     i2c0;

// Magnetometer/Accelerometer via I2C
FXOS8700CQ  accelmag(i2c0, FXOS8700CQ::ACCEL_2Gmode);

/**************************************************/

/**
 * Report magnetometer & accelerometer values
 *
 * @param magnetometer Magnetometer/Accelerometer to use
 */
void report(FXOS8700CQ &accelerometer) {
   int accelStatus, magStatus;
   int16_t accelX,accelY,accelZ;
   int16_t magX,magY,magZ;

   accelerometer.readAccelerometerXYZ(&accelStatus, &accelX, &accelY, &accelZ);
   accelerometer.readMagnetometerXYZ(&magStatus, &magX, &magY, &magZ);
   console.write("s=0x", accelStatus, Radix_16, ", aX=", accelX, "aY=", accelY, ", aZ=", accelZ);
   console.write("s=0x", magStatus, Radix_16, ", aX=", magX, "aY=", magY, ", aZ=", magZ);
   console.writeln("a=", (int)(180*atan2(magX, magY)/3.14159));
}

int main() {
   console.writeln("Starting\n");

   console.writeln("Device ID = 0x", accelmag.readID(), Radix_16, "(should be 0xC7)");

   // Enable both Accelerometer and magnetometer
   accelmag.enable(FXOS8700CQ::ACCEL_MAG);

   console.writeln("Before simple calibration (make sure the device is level!)\n");
   report(accelmag);
   report(accelmag);
   report(accelmag);

   accelmag.calibrateAccelerometer();

   // Make sure we have new values
   waitMS(100);

   console.writeln("After calibration\n");
   report(accelmag);

   console.writeln("Calibrating magnetometer\n"
          "Please rotate the board in all dimensions until complete (~20 s)\n");

   for (int time=20; time>0; time--) {
      console.write("Calibrating for ").write(time).writeln(" seconds");
      accelmag.calibrateMagnetometer(1);
   }

   for(;;) {
      report(accelmag);
      waitMS(400);
   }
}
