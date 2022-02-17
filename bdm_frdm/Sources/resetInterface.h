/*
 * ResetInterface.h
 *
 *  Created on: 22Dec.,2016
 *      Author: podonoghue
 */

#ifndef SOURCES_RESETINTERFACE_H_
#define SOURCES_RESETINTERFACE_H_

#include "hardware.h"

/**
 * Reset signal
 * Ouptut only
 * No reset event detection
 */
class ResetInterface {

private:
   // Output to RESET driver
   using Data      = USBDM::Reset_IO; // USBDM::GpioB<1>;

public:
   /**
    * Initialise Transceiver
    *
    * Initial state:
    *    Reset signal Input/HighZ
    */
   static void initialise() {

      // Enable pins
      Data::setInput();
   }

   /**
    * Drive signal low
    */
   static void low() {
      Data::low();
      Data::setOut();
   }
   /**
    * Disable Transceiver (high-impedance)\n
    * Actually sets to input
    */
   static void highZ() {
      Data::setIn();
   }

   /**
    * Read value from receiver
    *
    * @return value on pin or driven value if output
    *
    * @note Assumes already set as input or returns driven value
    */
   static bool read() {
      return Data::read();
   }

   /**
    * Check if receiver input is low
    *
    * @return true if input is low
    *
    * @note Assumes already set as input or returns driven value
    */
   static bool isLow() {
      return !Data::read();
   }

   /**
    * Check if receiver input is high
    *
    * @return true if input is high
    *
    * @note Assumes already set as input or returns driven value
    */
   static bool isHigh() {
      return Data::read();
   }
};

#endif /* SOURCES_RESETINTERFACE_H_ */
