/**
 ============================================================================
 * @file     hmc5883l-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief    Demonstrates use of HMC5883L Magnetometer over I2C
 * @version  V4.11.1.90
 * @author   podonoghue
 * @note You may need to change the pin-mapping of the I2C interface
============================================================================
 */
#include <math.h>
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "i2c.h"
#include "hmc5883l.h"

// Allows access to USBDM library name-space
using namespace USBDM;

/*************************************************
 * Global objects representing hardware
 **************************************************/

// I2C interface
I2c0     i2c0;

// Magnetometer via I2C
HMC5883L magnetometer(i2c0);

/**************************************************/

/**
 * Report magnetometer values
 *
 * @param magnetometer Magnetometer to use
 */
void report(HMC5883L &magnetometer) {
      int16_t compassX,compassY,compassZ;
      magnetometer.doMeasurement(&compassX, &compassY, &compassZ);
      console.
         write("X=").write(compassX).
         write("Y=").write(compassY).
         write("Z=").writeln(compassZ);
}

int main() {
   console.writeln("Starting");

   uint32_t id = magnetometer.readID();
   console.write("Device ID = ").write(id).writeln(" (should be 0x483433)\n");

//   magnetometer.setGain(3);

   for(;;) {
      report(magnetometer);
      waitMS(120);
   }
}
