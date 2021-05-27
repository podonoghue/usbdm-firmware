/**
 * @file     usb_cdc_interface.cpp
 * @brief    USB-UART interface
 *
 * This module handles the UART interface for USB CDC
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */
#include <string.h>

#include "hardware.h"
#include "uart.h"
#include "uart_queue.h"
#include "usb_cdc_interface.h"

namespace USBDM {

// Which UART to use
using UartInfo = Uart0Info;

static UartBuffered_basic_T<UartInfo, 20, 20> uart;

static volatile uint8_t  cdcStatus            = 0;
static volatile uint8_t  breakCount           = 0;
static bool              (*inCallback)()      = nullptr;

/**
 * Initialise class
 */
void CDC_Interface::initialise() {
}

/**
 * Set USB notify function
 *
 * @param cb The function to call to notify the USB In interface that new data is available
 */
void CDC_Interface::setUsbInNotifyCallback(SimpleCallback cb) {
   inCallback = cb;
}

/**
 * Process data received from host
 *
 * @param size Amount of data
 * @param buff Buffer containing data
 *
 * @note the Data is volatile so should be processed or saved immediately.
 */
void CDC_Interface::putData(int size, const volatile uint8_t *buff) {

   while (size-->0) {
      if (uart.txQueue.isFull()) {
         // Discard data and flag overrun
         cdcStatus |= UART_S1_OR_MASK;
         return;
      }
      uart.txQueue.enQueue(*buff++);
   }
}

/**
 * Get data to transmit to host
 *
 * @param bufSize Size of buffer
 * @param buff    Buffer for data
 *
 * @return Amount of data placed in buffer
 */
int CDC_Interface::getData(int bufSize, uint8_t *buff) {
   int size = 0;
   while (size<bufSize) {
      if (uart.rxQueue.isEmpty()) {
         break;
      }
      size++;
      *buff++ = uart.rxQueue.deQueue();
   }
   return size;
}

/**
 * Get state of serial interface
 *
 * @return Bit mask value
 */
CdcLineState CDC_Interface::getSerialState() {

   static uint8_t lastSciStatus = 0x00;

   // Assume DCD & DSR
   CdcLineState status{(uint8_t)(CDC_STATE_DCD_MASK|CDC_STATE_DSR_MASK)};

   if (cdcStatus&UART_S1_FE_MASK) {
      status.bits |= CDC_STATE_FRAME_MASK;
   }
   if (cdcStatus&UART_S1_OR_MASK) {
      status.bits |= CDC_STATE_OVERRUN_MASK;
   }
   if (cdcStatus&UART_S1_PF_MASK) {
      status.bits |= CDC_STATE_PARITY_MASK;
   }
   if (lastSciStatus != cdcStatus) {
      // Remember error status so we only report changes
      lastSciStatus  = cdcStatus&(UART_S1_FE_MASK|UART_S1_OR_MASK|UART_S1_PF_MASK);
      status.bits   |= CDC_STATE_CHANGE_MASK;
   }
   cdcStatus = 0;
   return status;
}

/**
 *  Set CDC communication characteristics
 *
 * @param lineCodingStructure - Structure describing desired settings
 *
 * The UART is quite limited when compared to the serial interface implied
 * by LineCodingStructure.
 * It does not support many of the combinations available.
 */
void CDC_Interface::setLineCoding(volatile LineCodingStructure &lineCoding) {
   uint8_t  UARTC1Value = 0x00;
   uint8_t  UARTC3Value = 0x00;

   // Ignore illegal values
   volatile unsigned newBaud = lineCoding.dwDTERate;
   if ((newBaud == 0) || (newBaud>115200)) {
      return;
   }

   // Initialise UART and set baud rate
   uart.setBaudRate(newBaud);

   cdcStatus  = CDC_STATE_CHANGE_MASK;
   breakCount = 0; // Clear any current BREAKs

   //! Note - for a 48MHz bus speed the useful baud range is ~300 to ~115200 for 0.5% error
   //  230400 & 460800 have a 8.5% error

   // Configure pins
   UartInfo::initPCRs();

   // Disable the transmitter and receiver while changing settings.
   UartInfo::uart().C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

   // Note: lineCoding.bCharFormat is ignored (always 1 stop bit)
   //   switch (lineCoding.bCharFormat) {
   //      case 0:  // 1 bits
   //      case 1:  // 1.5 bits
   //      case 2:  // 2 bits
   //   }

   // Available combinations
   //============================================
   // Data bits  Parity   Stop |  M   PE  PT  T8
   //--------------------------------------------
   //     7      Odd       1   |  0   1   1   X
   //     7      Even      1   |  0   1   0   X
   //     8      None      1   |  0   0   X   X
   //     8      Odd       1   |  1   1   1   X
   //     8      Even      1   |  1   1   0   X
   //     8      Mark      1   |  1   0   X   0
   //     8      Space     1   |  1   0   X   1
   //--------------------------------------------
   //   All other values default to 8-None-1

   switch (lineCoding.bDataBits) {
   // 5,6,7,8,16
   case 7 :
      switch (lineCoding.bParityType) {
      case 1:  UARTC1Value = UART_C1_PE_MASK|UART_C1_PT_MASK; break; // Odd
      case 2:  UARTC1Value = UART_C1_PE_MASK;                 break; // Even
      }
      break;
   case 8 :
      UARTC1Value = UART_C1_M_MASK; // 9-data or 8-data+parity
      switch (lineCoding.bParityType) {
      case 0:  UARTC1Value  = 0;                               break; // None
      case 1:  UARTC1Value |= UART_C1_PE_MASK|UART_C1_PT_MASK; break; // Odd
      case 2:  UARTC1Value |= UART_C1_PE_MASK;                 break; // Even
      case 3:  UARTC3Value  = UART_C3_T8_MASK;                 break; // Mark
      case 4:                                                  break; // Space
      }
      break;
   default :
      break;
   }
   UartInfo::uart().C1 = UARTC1Value;
   UartInfo::uart().C2 =
         UART_C2_RIE_MASK| // Receive interrupts (Transmit interrupts enabled when data written)
         UART_C2_RE_MASK|  // Receiver enable
         UART_C2_TE_MASK;  // Transmitter enable
   UartInfo::uart().C3 = UARTC3Value|
         UART_C3_FEIE_MASK| // Framing error
         UART_C3_NEIE_MASK| // Noise error
         UART_C3_ORIE_MASK| // Overrun error
         UART_C3_PEIE_MASK; // Parity error

   NVIC_EnableIRQ(UartInfo::irqNums[0]);
}

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
void CDC_Interface::sendBreak(uint16_t length) {
   if (length == 0xFFFF) {
      // Send indefinite BREAKs
      breakCount = 0xFF;
   }
   else if (length == 0x0) {
      // Stop sending BREAKs
      breakCount = 0x00;
   }
   else {
      // Queue a series of BREAKs
      breakCount = 10;
   }
}

/**
 *  Set CDC Line values
 *
 * @param value - Describes desired settings
 */
void CDC_Interface::setControlLineState(uint8_t value) {
   (void) value;
   // Not implemented as no control signals
}

}; // end namespace USBDM
