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

using namespace USBDM;

/*
 * External Joy-stick
 * 2 x Analogue input
 * 1 x Digital input
 *
 */

// Connection mapping - change as required
using Adc = USBDM::Adc0;

using JOYSTICK_X = Adc::adc_A2;
using JOYSTICK_Y = Adc::adc_A1;
using JOYSTICK_K = gpio_A0;

int main(void) {

   // Enable and configure ADC
   Adc::configure(AdcResolution_8bit_se);

   // Calibrate before use
   Adc::calibrate();

   // Connect ADC channels to pins
   JOYSTICK_X::setInput();
   JOYSTICK_Y::setInput();

   // Connect and configure digital input pin
   JOYSTICK_K::setInput(PinPull_Up);

   for(;;) {
      int  x      = JOYSTICK_X::readAnalogue();
      int  y      = JOYSTICK_Y::readAnalogue();
      bool button = JOYSTICK_K::isPressed();
      console.write("Joystick (X,Y,K) = ").write(x).write(", ").write(y).write(", ").writeln(button?"Pressed":"Released");
   }
}
