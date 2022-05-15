/**
 ============================================================================
 * @file analogue-joystick-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief Example showing use of a use of 2 ADC channels with a 2-pot joystick
 *
 *  Created on: 10/6/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "adc.h"

using namespace USBDM;

/*
 * External Joy-stick
 * 2 x Analogue input
 * 1 x Digital input
 *
 */

// Connection mapping - change as required
using MyAdc = USBDM::Adc0;

using JOYSTICK_X = Adc0::Channel<8>;
using JOYSTICK_Y = Adc0::Channel<9>;
using JOYSTICK_K = GpioD<5,  ActiveLow>;

int main(void) {

   // Enable and configure ADC
   MyAdc::configure(AdcResolution_8bit_se);

   // Calibrate before use
   MyAdc::calibrate();

   // Connect ADC channels to pins
   JOYSTICK_X::setInput();
   JOYSTICK_Y::setInput();

   // Connect and configure digital input pin
   JOYSTICK_K::setInput(PinPull_Up);

   for(;;) {
      int  x      = JOYSTICK_X::readAnalogue();
      int  y      = JOYSTICK_Y::readAnalogue();
      bool button = JOYSTICK_K::isPressed();
      console.write("Joystick (X,Y,K) = ", x, ", ", y, ", ", button?"Pressed":"Released");
   }
}
