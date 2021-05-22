/*
 * ResetInterface.h
 *
 *  Created on: 22Dec.,2016
 *      Author: podonoghue
 */

#ifndef SOURCES_RESETINTERFACE_H_
#define SOURCES_RESETINTERFACE_H_

//#include "USBDM_MK.h"
#include "hardware.h"

/**
 * 3-State I/O for reset signal
 */
class ResetInterface {

private:
   using Data      = USBDM::GpioB<1>;

   static bool  fResetActivity;

public:
   /**
    * Initialise Transceiver
    *
    * Initial state:
    *    Input/HighZ
    */
   static void initialise() {

      // Set pin as input
      Data::setInput();

      // IRQ on falling edge - reset detection

      fResetActivity = false;
   }
   /**
    * Drive signal low
    */
   static void low() {
      Data::low();
      Data::setOut();
   }
   /**
    * Drive signal low
    *
    * @note Assumes driver already enabled
    */
   static void _low() {
      Data::low();
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

   /**
    * Check and clear reset activity flag
    *
    * @return True  Reset has been active since last polled
    * @return False Reset has not been active since last polled
    */
   static bool resetActivity() {
      bool temp = fResetActivity;
      fResetActivity = false;
      return temp;
   }
};

#endif /* SOURCES_RESETINTERFACE_H_ */
