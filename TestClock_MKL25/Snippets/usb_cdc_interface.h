/**
 * @file     usb_cdc_interface.h
 * @brief    USB-CDC interface
 *
 * This module handles the UART interface for USB CDC
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */

#ifndef SOURCES_USB_CDC_INTERFACE_H_
#define SOURCES_USB_CDC_INTERFACE_H_

#include <stdint.h>
#include <string.h>
#include "usb_defs.h"

namespace USBDM {

class CDC_Interface {

private:
   static constexpr uint8_t CDC_STATE_DCD_MASK        = 1<<0;
   static constexpr uint8_t CDC_STATE_DSR_MASK        = 1<<1;
   static constexpr uint8_t CDC_STATE_BREAK_IN_MASK   = 1<<2;
   static constexpr uint8_t CDC_STATE_RI_MASK         = 1<<3;
   static constexpr uint8_t CDC_STATE_FRAME_MASK      = 1<<4;
   static constexpr uint8_t CDC_STATE_PARITY_MASK     = 1<<5;
   static constexpr uint8_t CDC_STATE_OVERRUN_MASK    = 1<<6;
   static constexpr uint8_t CDC_STATE_CHANGE_MASK     = 1<<7;

   static constexpr uint8_t CDC_LINE_CONTROL_DTR_MASK = 1<<0;
   static constexpr uint8_t CDC_LINE_CONTROL_RTS_MASK = 1<<1;

protected:

   /** Type for call-back */
   using SimpleCallback = bool (*)();

protected:
   CDC_Interface() {}
   virtual ~CDC_Interface() {}

public:
   /**
    * Initialise class
    */
   static void initialise();

   /**
    * Set USB notify function
    *
    * @param cb The function to call to notify the USB In interface that new data is available
    */
   static void setUsbInNotifyCallback(SimpleCallback cb);

   /**
    * Get state of serial interface
    *
    * @return Bit mask value
    */
   static CdcLineState getSerialState();

   /**
    * Process data received from host
    *
    * @param size Amount of data
    * @param buff Buffer for data
    *
    * @note The data is volatile so should be processed or saved immediately.
    */
   static void putData(int size, const volatile uint8_t *buff);

   /**
    * Get data to transmit to host
    *
    * @param bufSize Size of buffer
    * @param buff    Buffer for data
    *
    * @return Amount of data placed in buffer
    */
   static int getData(int bufSize, uint8_t *buff);

public:
   /**
    * Set line coding
    *
    * @param lineCoding Line coding information
    */
   static void setLineCoding(volatile LineCodingStructure &lineCoding);

   /**
    *  Set CDC Line values
    *
    * @param value - Describes desired settings
    */
   static void setControlLineState(uint8_t value);

   /**
    *  Send CDC break\n
    *
    * @param length Length of break in milliseconds (see note)\n
    *  - 0x0000 => End BREAK
    *  - 0xFFFF => Start indefinite BREAK
    *  - else   => Send a break of 10 chars

    * @note - only partially implemented
    *       - breaks are sent after currently queued characters
    */
   static void sendBreak(uint16_t length);
}; // class CDC_interface

}; // end namespace USBDM

#endif /* SOURCES_USB_CDC_INTERFACE_H_ */
