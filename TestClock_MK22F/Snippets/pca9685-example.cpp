/**
 ============================================================================
 * @file pca9685-example.cpp (180.ARM_Peripherals/Snippets/)
 * @brief Demonstrates use of PCA9685 over I2C
 * @version  V4.11.1.90
 * @author   podonoghue
 * @note You may need to change the pin-mapping of the I2C interface
============================================================================
 */
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "pca9685.h"

// Allows access to USBDM library name-space
using namespace USBDM;

/*************************************************
 * Global objects representing hardware
 **************************************************/

// I2C interface
I2c0     i2c0;

// PCA9685 via I2C
PCA9685 pca9685(i2c0);

/**************************************************/


// LED connections
#define RED_LED   USBDM::gpio_LED_RED
#define GREEN_LED USBDM::gpio_LED_GREEN

int main() {

   pca9685.set_pin_high(3);
   pca9685.set_pin_pwm(3, 50);

   bool odd=true;
   for (;;) {
      odd = !odd;
      for (int i=0; i<15; i++) {
         if (odd) {
            pca9685.set_pin_high(i);
         }
         else {
            pca9685.set_pin_low(i);
         }
      }
   }
//   RED_LED::setOutput();
//   GREEN_LED::setOutput();
//   RED_LED::set();
//   GREEN_LED::set();
//   for(;;) {
//      RED_LED::toggle();
//      delay();
//      RED_LED::toggle();
//      delay();
//      GREEN_LED::toggle();
//      delay();
//      GREEN_LED::toggle();
//      delay();
//   }
}
