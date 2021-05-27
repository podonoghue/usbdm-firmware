/*
 ============================================================================
 * @file    main.cpp (180.ARM_Peripherals/Sources/main.cpp)
 * @brief   Basic C++ demo
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "flash.h"

using namespace USBDM;

/*
 * PTB8 (= A0 on FRDM board) is self erase pin
 * Tie low during reset to cause self erase.
 */
using TestPin = GpioB<8,ActiveLow>;

// LED connections
using BlueLed    = GpioB<10,ActiveLow>;
using GreenLed   = GpioB<9,ActiveLow>;

// SWD pins
using Swdclk = GpioA<0>;
using Reset  = GpioA<1>;
using Swdio  = GpioA<2>;

int main() {

   // Configure Pins
   TestPin::setInput(PinPull_Up, PinAction_None, PinFilter_Passive);
   GreenLed::setOutput(PinDriveStrength_High);
   BlueLed::setOutput(PinDriveStrength_High);

   // Wait a while before disabling SWD
   for (int delay=20; delay-->0;) {
      if (waitMS(100, TestPin::isActive)) {
         BlueLed::off();
         GreenLed::on();
         Flash::eraseAll();
      }
      BlueLed::toggle();
   }

   // Disable SWD interface
   // Note: reset is also disabled in FOPT
   Swdclk::Pcr::setPCR(PinMux_Analog);
   Reset::Pcr::setPCR(PinMux_Analog);
   Swdio::Pcr::setPCR(PinMux_Analog);

   // Green LED on to indicate complete
   GreenLed::on();

   for(;;) {
   }

   return 0;
}
