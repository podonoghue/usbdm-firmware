/*
 * interface.h
 *
 *  Created on: 21 May 2021
 *      Author: peter
 */

#ifndef SOURCES_INTERFACE_H_
#define SOURCES_INTERFACE_H_
#include "hardware.h"
#include "smc.h"

/**
 * GPIO for Activity LED
 */
class UsbLed : private USBDM::SDA_LED {

public:
   using USBDM::SDA_LED::toggle;
   using USBDM::SDA_LED::on;
   using USBDM::SDA_LED::off;
   using USBDM::SDA_LED::write;

   /** Initialise activity LED */
   inline static void initialise() {
     using namespace USBDM;

     setOutput(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Slow);
   }
};

/**
 * GPIO controlling some interface signals (SWD, UART-TX)
 */
class InterfaceEnable : public USBDM::GpioA<4, USBDM::ActiveLow> {
public:
   /** Initialise pin as output driving inactive level */
   static void initialise() {
     setOutput();
   }
};


#endif /* SOURCES_INTERFACE_H_ */
