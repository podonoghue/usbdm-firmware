/*
 * interface.h
 *
 *  Created on: 21 May 2021
 *      Author: peter
 */

#ifndef SOURCES_INTERFACE_H_
#define SOURCES_INTERFACE_H_
#include "hardware.h"

/**
 * GPIO for Activity LED
 */
class UsbLed : public USBDM::GpioD<4, USBDM::ActiveLow> {
public:
   /** Initialise activity LED */
   static void initialise() {
      setOutput();
   }
};

/**
 * GPIO for Debug pin
 */
class Debug : public USBDM::GpioC<1, USBDM::ActiveHigh> {
public:
   /** Initialise debug pin */
   static void initialise() {
      setOutput();
   }
};

/**
 * GPIO controlling some interface signals (SWD, UART-TX)
 */
class InterfaceEnable : public USBDM::GpioA<4, USBDM::ActiveLow> {
public:
   /** Initialise activity LED */
   static void initialise() {
      setOutput();
   }
};

#endif /* SOURCES_INTERFACE_H_ */
