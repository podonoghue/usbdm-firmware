/**
 ============================================================================
 * @file mma8491q-example.cpp
 * @brief Demonstrates use of MMA8491q Accelerometer over I2C
 * @version  V4.11.1.90
 * @author   podonoghue
 * @note You may need to change the pin-mapping of the I2C interface
============================================================================
 */
#include <math.h>
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "mma8491q.h"

// Allows access to USBDM library name-space
using namespace USBDM;

/*************************************************
 * Global objects representing hardware
 **************************************************/

// I2C interface
I2c0     i2c0;

// Accelerometer via I2C
// Enable pin will need adjustment e.g.
// D8 => USBDM::GpioA<13> on FRDM-KL25
// D8 => USBDM::GpioA<12> on FRDM-MK20D50
MMA8491Q_T<USBDM::GpioA<13>> accelerometer(i2c0);

/**************************************************/

/**
 * Report accelerometer values
 *
 * @param accelerometer Accelerometer to use
 */
void report(MMA8491Q &accelerometer) {
   int accelStatus;
   int16_t accelX,accelY,accelZ;

   accelerometer.active();
   waitMS(1000);
   accelerometer.readAccelerometerXYZ(accelStatus, accelX, accelY, accelZ);
   accelerometer.standby();
   console.
      write("s=").write(accelStatus, Radix_16).
      write(", aX=").write(accelX).
      write(", aY=").write(accelY).
      write(", aZ=").writeln(accelZ);
}

int main() {
   printf("Starting\n");

   report(accelerometer);
   printf("Doing simple calibration\n"
          "Make sure the device is level!\n");

   waitMS(4000);

   accelerometer.calibrateAccelerometer();

   // Make sure we have new values
   waitMS(100);

   printf("After calibration\n");
   for(;;) {
      report(accelerometer);
//      waitMS(400);
   }
}

