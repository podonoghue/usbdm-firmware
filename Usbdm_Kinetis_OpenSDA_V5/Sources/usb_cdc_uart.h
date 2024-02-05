/**
 * @file     usb_cdc_uart.h
 * @brief    USB-UART interface
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */

#ifndef PROJECT_HEADERS_CDC_UART_H_
#define PROJECT_HEADERS_CDC_UART_H_

#include "pin_mapping.h"
#include "uart.h"
#include "usb_defs.h"
#include "uart_queue.h"
#include "usb.h"

namespace USBDM {

/**
 * Buffered UART
 *
 * @tparam UartInfo     UART information
 * @tparam QUEUE_SIZE   Size of out queue (writes to UART)
 */
template<class UartInfo, int QUEUE_SIZE>
class CdcUart {

private:
   static uint8_t                cdcStatus;
   static uint8_t                breakCount;
   static LineCodingStructure    lineCoding;

   // Call-back to pass a character to the CDC IN Pipe
   static bool                   (*cdcInCallback)(uint8_t);

   /* Queue of outgoing characters (to UART) */
   static UartQueue<char, QUEUE_SIZE> outQueue;

public:
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

   /**
    * Set callback to handle received characters
    *
    * @param callback Call-back to execute when a serial character is received
    */
   static void setInCallback(bool (*callback)(uint8_t)) {
      cdcInCallback = callback;
   }

   /**
    * Write character to output queue (to UART)
    *
    * @param ch Character to write
    *
    * @return true => success, false => overrun or similar error
    *
    * @note The Overrun flag is set on write to full queue
    */
   static bool putChar(uint8_t ch) {
//      USBDM::console.WRITELN("Tx:", ch,Radix_16);

      if (!outQueue.enQueueDiscardOnFull(ch)) {
         cdcStatus = cdcStatus | UART_S1_OR_MASK;
         return false;
      }
      // Make sure Transmit IRQ enabled
      UartInfo::uart->C2 = UartInfo::uart->C2 | UART_C2_TIE_MASK;
      return true;
   }

   /**
    * Get space available in output queue
    */
   static unsigned getRemaingCapacity() {
      return outQueue.getRemainingCapacity();
   }

   /**
    * Get state of serial interface
    *
    * @return Bit mask value
    */
   static CdcLineState getSerialState() {

      static uint8_t lastSciStatus = 0x00;

      // Assume DCD & DSR
      CdcLineState status(CDC_STATE_DCD_MASK|CDC_STATE_DSR_MASK);

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
    * The CDC is quite limited when compared to the serial interface implied
    * by LineCodingStructure.
    * It does not support many of the combinations available.
    */
   static void setLineCoding(const LineCodingStructure *lineCodingStructure) {
      uint8_t  UARTC1Value = 0x00;
      uint8_t  UARTC3Value = 0x00;

      lineCoding = *lineCodingStructure;

      // Disable the transmitter and receiver while changing settings.
      UartInfo::uart->C2 = UartInfo::uart->C2 & ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
      UartInfo::setBaudRate(UartBaudRate(leToNative32(lineCoding.dwDTERate)));

      cdcStatus  = CDC_STATE_CHANGE_MASK;
      breakCount = 0; // Clear any current BREAKs

      //! Note - for a 48MHz bus speed the useful baud range is ~300 to ~115200 for 0.5% error
      //  230400 & 460800 have a 8.5% error

      // Configure pins
      UartInfo::initPCRs();

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
      UartInfo::uart->C1 = UARTC1Value;
      UartInfo::uart->C2 =
            UART_C2_RIE_MASK| // Receive interrupts (Transmit interrupts enabled when data written)
            UART_C2_RE_MASK|  // Receiver enable
            UART_C2_TE_MASK;  // Transmitter enable
      UartInfo::uart->C3 = UARTC3Value|
            UART_C3_FEIE_MASK| // Framing error
            UART_C3_NEIE_MASK| // Noise error
            UART_C3_ORIE_MASK| // Overrun error
            UART_C3_PEIE_MASK; // Parity error

      NVIC_EnableIRQ(UartInfo::irqNums[0]);
   }

   /**
    *  Get CDC communication characteristics\n
    *
    *  @return lineCodingStructure - Static structure describing current settings
    */
   static const LineCodingStructure *getLineCoding(void) {
      return &lineCoding;
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
   static void sendBreak(uint16_t length) {
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
   static void setControlLineState(uint8_t value) {
      (void) value;
      // Not implemented as no control signals
   }

   /**
    * Interrupt callback for UART
    */
   static void uartCallback() {

      uint8_t status = Uart1Info::uart->S1;

      if (status&UART_S1_RDRF_MASK) {
         // Transfers a char from the UART_D to CDC-IN queue
         char ch = UartInfo::uart->D;
//         USBDM::console.WRITE(ch);

         if (!cdcInCallback(ch)) {
            cdcStatus |= UART_S1_OR_MASK;
         }
      }
      if (status&UART_S1_TDRE_MASK) {
         //  Transfer a char from the CDC-OUT queue to UART_D
         if (!outQueue.isEmpty()) {
            UartInfo::uart->D = outQueue.deQueue(); // Send the char
         }
         else if (breakCount > 0) {
            // Send another BREAK 'char'
            UartInfo::uart->C2 = UartInfo::uart->C2 | UART_C2_SBK_MASK;
            UartInfo::uart->C2 = UartInfo::uart->C2  & ~UART_C2_SBK_MASK;
            if (breakCount != 0xFF) {
               breakCount--;
            }
         }
         else {
            // No characters available
            // Disable further UART transmit interrupts
            UartInfo::uart->C2 = UartInfo::uart->C2 & ~UART_C2_TIE_MASK;
         }
      }
      if (status&(UART_S1_FE_MASK|UART_S1_PF_MASK|UART_S1_FE_MASK|UART_S1_NF_MASK|UART_S1_OR_MASK)) {
         // Record and clear error status
         cdcStatus |= status;
         Uart0Info::clearError();
      }
   }

   static void initialise() {
      /**
       * Default initialisation value for Uart
       * This value is created from Configure.usbdmProject settings
       */
      static constexpr Uart1Info::Init uartInitValue = {
         UartBaudRate_115200 , // (uart_baud_rate) UART baud rate - 115200
         UartLinBreakAction_None , // (uart_bdh_lbkdie) LIN break detect action - None
         UartRxdActiveEdgeAction_None , // (uart_bdh_rxedgie) RxD input active edge action - None
         UartTxCompleteAction_None , // (uart_c2_tcie) Transmit complete action - None
         UartIdleLineDetectAction_None , // (uart_c2_ilie) Idle line detect action - None
         UartTxEmptyAction_Interrupt , // (uart_c5_tdmas) Transmit empty DMA/Interrupt action - Interrupt
         UartRxFullAction_Interrupt,  // (uart_c5_rdmas) Receive full DMA/interrupt action - Interrupt
      };

      Uart1Info::configure(uartInitValue);

      UartInfo::setCallback(Uart1IrqNum_Error, uartCallback);
      UartInfo::setCallback(Uart1IrqNum_RxTx,  uartCallback);
      Uart1Info::enableNvicInterrupts(Uart1IrqNum_RxTx);
      Uart1Info::enableNvicInterrupts(Uart1IrqNum_Error);
    }
};

template<class UartInfo, int QUEUE_SIZE>
uint8_t             CdcUart<UartInfo, QUEUE_SIZE>::breakCount = 0;
template<class UartInfo, int QUEUE_SIZE>
uint8_t             CdcUart<UartInfo, QUEUE_SIZE>::cdcStatus  = CdcUart<UartInfo, QUEUE_SIZE>::CDC_STATE_CHANGE_MASK;
template<class UartInfo, int QUEUE_SIZE>
LineCodingStructure CdcUart<UartInfo, QUEUE_SIZE>::lineCoding = {leToNative32(9600UL),0,1,8};

template<class UartInfo, int QUEUE_SIZE>
UartQueue<char, QUEUE_SIZE> CdcUart<UartInfo, QUEUE_SIZE>::outQueue;

template<class UartInfo, int QUEUE_SIZE>
bool (*CdcUart<UartInfo, QUEUE_SIZE>::cdcInCallback)(uint8_t ch);

}; // end namespace USBDM

#endif /* PROJECT_HEADERS_CDC_UART_H_ */
