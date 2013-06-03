/*
 * uart.c
 *
 *  Created on: Nov 6, 2012
 *      Author: podonoghue
 */
#include <string.h>
#include "derivative.h" /* include peripheral declarations */
#include "Common.h"
#include "clock.h"
#include "CDC.h"

#if (HW_CAPABILITY&CAP_CDC)

#define CONCAT3_(x,y,z) x ## y ## z

#ifndef UART_NUM
#define UART_NUM               0
#endif

#define UART_C1(x)             CONCAT3_(UART,x,_C1)
#define UART_C2(x)             CONCAT3_(UART,x,_C2)
#define UART_C3(x)             CONCAT3_(UART,x,_C3)
#define UART_C4(x)             CONCAT3_(UART,x,_C4)
#define UART_BDH(x)            CONCAT3_(UART,x,_BDH)
#define UART_BDL(x)            CONCAT3_(UART,x,_BDL)
#define UART_S1(x)             CONCAT3_(UART,x,_S1)
#define UART_D(x)              CONCAT3_(UART,x,_D)
#define INT_UART_RX_TX(x)      CONCAT3_(INT_UART,x,_RX_TX)
#define UART_IRQHandler(x)     CONCAT3_(UART,x,_IRQHandler)
#define SIM_SCGC4_UART_MASK(x) CONCAT3_(SIM_SCGC4_UART,x,_MASK)

#define UARTx_C1               UART_C1(UART_NUM)
#define UARTx_C2               UART_C2(UART_NUM)
#define UARTx_C3               UART_C3(UART_NUM)
#define UARTx_C4               UART_C4(UART_NUM)
#define UARTx_BDH              UART_BDH(UART_NUM)
#define UARTx_BDL              UART_BDL(UART_NUM)
#define UARTx_S1               UART_S1(UART_NUM)
#define UARTx_D                UART_D(UART_NUM)
#define INT_UARTx_RX_TX        INT_UART_RX_TX(UART_NUM)
#define UARTx_IRQHandler       UART_IRQHandler(UART_NUM)
#define SIM_SCGC4_UARTx_MASK   SIM_SCGC4_UART_MASK(UART_NUM)

#if 0
__inline
static uint32_t swap32(uint32_t data) {
   return ((data<<24)&0xFF000000)|((data<<8)&0x00FF0000)|
          ((data>>24)&0x000000FF)|((data>>8)&0x0000FF00);
}
__inline
static uint16_t swap16(uint16_t data) {
       return ((data<<16)&0xFF00)|((data>>8)&0xFF);
}
#define leToNative32(x) swap32(x)
#define leToNative16(x) swap16(x)
#define nativeToLe32(x) swap32(x)
#define nativeToLe16(x) swap16(x)
#else
__inline
static uint32_t noChange32(uint32_t data) {
   return data;
}
__inline
static uint16_t noChange16(uint16_t data) {
       return data;
}
#define leToNative32(x) noChange32(x)
#define leToNative16(x) noChange16(x)
#define nativeToLe32(x) noChange32(x)
#define nativeToLe16(x) noChange16(x)
#endif

#define CDC_TX_BUFFER_SIZE (16)  // Should equal end-point buffer size 
static char txBuffer[CDC_TX_BUFFER_SIZE];
static uint8_t txHead        = 0;
static uint8_t txBufferCount = 0;
static uint8_t breakCount    = 0;
#define CDC_RX_BUFFER_SIZE (16)  // Should less than or equal to end-point buffer size 
static char *rxBuffer;
static uint8_t rxBufferCount = 0;
static uint8_t cdcStatus = SERIAL_STATE_CHANGE;

#if CPU == MK20D5
   #define enableUartIrq()   NVIC_ISER((INT_UARTx_RX_TX-16)/32) = NVIC_ISER_SETENA(1<<((INT_UARTx_RX_TX-16)%32));
   #define disableUartIrq()  NVIC_ICER((INT_UARTx_RX_TX-16)/32) = NVIC_ICER_CLRENA(1<<((INT_UARTx_RX_TX-16)%32));
#else
   #error "CPU not set"
#endif


// The following routines are assumed to be called from interrupt code - Interrupts masked
//

//
// Simple double-buffering for Rx (in conjunction with USB buffer)
//

//! putRxBuffer() -  Add a char to the CDC-Rx buffer
//!
//! @param ch - char to add
//!
//! @note Overun flag is set on buffer full
//!
void cdc_putRxBuffer(char ch) {
   if (rxBufferCount >= CDC_RX_BUFFER_SIZE) {
      cdcStatus |= UART_S1_OR_MASK;
      return;
   }
   rxBuffer[rxBufferCount++] = ch;
//   if (rxBufferCount == CDC_RX_BUFFER_SIZE) {
//      checkUsbCdcRxData();
//   }
}

//! setRxBuffer() - Sets CDC-Rx buffer
//!
//! @param buffer - buffer to write future data to
//!
//! @return -  number of characters in existing buffer
//!
uint8_t cdc_setRxBuffer(char *buffer) {
   uint8_t temp;
#ifdef LOG   
   *buffer = 'X'; // Debug - This character should never appear!
#endif   
   rxBuffer = buffer;
   temp = rxBufferCount;
   rxBufferCount = 0;
   return temp;
}

//! RxBufferEmpty() - Check if Rx buffer is empty
//!
//! @return -  1 => buffer is empty
//!            0 => buffer is not empty
//!
uint8_t cdc_rxBufferItemCount(void) {
   return rxBufferCount;
}

//
// Simple double-buffering for CDC-Tx (in conjunction with USB end-point buffer)
//

//! putTxBuffer() -  Copy characters to the CDC-Tx buffer
//! UART interrupts are enabled.
//!
//! @param source - buffer of source chars
//! @param size   - number of source characters in buffer
//!
//! @return - 0 - OK
//!           1 - Buffer is busy (overrun)
//!
uint8_t cdc_putTxBuffer(char *source, uint8_t size) {
   if (txBufferCount > 0) {
      return 1; // Busy
   }
   (void)memcpy(txBuffer, source, size);
   txHead        = 0;
   txBufferCount = size;
   UARTx_C2 |= UART_C2_TIE_MASK; // Enable UART Tx interrupts
   return 0;
}

//! getTx() -  Gets a character from the CDC-Tx queue.
//!
//! @return 
//!  -  -ve => queue is empty \n
//!  -  +ve => char from queue
//!
static int cdc_getTxBuffer(void) {
   uint8_t ch;
   if (txBufferCount == 0) {
      // Check data in USB buffer & restart USB Out if needed
      checkUsbCdcTxData();
   }
   // Need to re-check as above may have copied data
   if (txBufferCount == 0) {
      return -1;
   }
   ch = txBuffer[txHead++];
   if (txHead >= txBufferCount)
      txBufferCount = 0;
   return ch;
}

//! cdcTxSpace - check if CDC-Tx buffer is free
//!
//! @return 0 => buffer is occupied
//!         1 => buffer is free
//!
uint8_t cdc_txBufferIsFree(void) {
   return (txBufferCount == 0);
}

uint8_t cdc_getSerialState(void) {
   uint8_t status = 0x3;
   static uint8_t lastSciStatus = 0x00;

   if (cdcStatus&UART_S1_FE_MASK) {
      status |= 1<<4; 
   }
   if (cdcStatus&UART_S1_OR_MASK) {
      status |= 1<<6; 
   }
   if (cdcStatus&UART_S1_PF_MASK) {
      status |= 1<<5;
   }
   if (lastSciStatus != cdcStatus) {
      lastSciStatus = cdcStatus;
      status |= SERIAL_STATE_CHANGE;
   }
   cdcStatus = 0;
   return status;
}

//! Interrupt handler for CDC-Tx \n
//! Transfers a char from the CDC-Tx queue to UART_D
//!
//! @note Interrupts are disabled on empty queue
//!
void cdc_txHandler(void) {
int ch;

   ch = cdc_getTxBuffer();
   if (ch >= 0) {
      UARTx_D = (uint8_t)ch; // Send the char
   }
   else if (breakCount > 0) {
      UARTx_C2 |=  UART_C2_SBK_MASK; // Send another BREAK 'char'
      UARTx_C2 &= ~UART_C2_SBK_MASK;
      if (breakCount != 0xFF) {
         breakCount--;
       }
   }
   else {
      UARTx_C2 &= ~UART_C2_TIE_MASK; // Disable further Tx interrupts
   }
}

//! Interrupt handler for CDC Rx \n
//! Transfers a char from the UART_D to USB IN queue
//! 
//! @note Overruns are ignored
//!
void cdc_rxHandler(void) {
   cdc_putRxBuffer(UARTx_D);
}

void UARTx_IRQHandler() {
   uint8_t status = UARTx_S1;
   if (status&UART_S1_RDRF_MASK) {
      cdc_rxHandler();      
   }
   else if (status&UART_S1_TDRE_MASK) {
      cdc_txHandler();
   }
   else {
      // Record and clear error status
      cdcStatus |= status;
      UARTx_S1 = status;
   }
}

#define BAUDDIVIDER(x)  (((BUS_FREQ/16))/(x))

static LineCodingStructure lineCoding = {CONST_NATIVE_TO_LE32(9600UL),0,1,8};

//! Set CDC Tx characteristics
//!
//! @param lineCodingStructure - Structure describing desired settings
//!
//! The CDC is quite limited when compared to the serial interface implied by
//! LineCodingStructure.
//! It does not support many of the combinations available.
//!
void cdc_setLineCoding(const LineCodingStructure *lineCodingStructure) {
   uint32_t baudrate;
   uint16_t ubd;
   uint8_t  UARTC1Value = 0x00;
   uint8_t  UARTC3Value = 0x00;

   cdcStatus  = SERIAL_STATE_CHANGE;
   breakCount = 0; // Clear any current BREAKs
   
   (void)memcpy(&lineCoding, lineCodingStructure, sizeof(LineCodingStructure));

   //! todo  Note - for a 48MHz bus speed the useful baud range is ~300 to ~115200 for 0.5% error
   //        230400 & 460800 have a 8.5% error

   // Enable clock to PTA (for UART0 pin muxing)
   SIM_SCGC5  |= SIM_SCGC5_PORTA_MASK;
   
   // Configure shared pins
//   PORTB_PCR16  = PORT_PCR_MUX(3); // Rx (PFE?)
//   PORTB_PCR17  = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK| PORT_PCR_SRE_MASK; // Tx

   RX_OUT_EN_PCR = PORT_PCR_MUX(RX_ALT_FN);
   TX_OUT_EN_PCR = PORT_PCR_MUX(TX_ALT_FN)|PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;

   // Enable clock to UART0
   SIM_SCGC4  |= SIM_SCGC4_UARTx_MASK;
   
   // Disable the transmitter and receiver while changing settings.
   UARTx_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

   // Determine baud rate divider
   baudrate = leToNative32(lineCoding.dwDTERate);

   // Calculate baud settings
   ubd = (uint16_t)(SystemCoreClock/(baudrate * 16));

   // Set Baud rate register
   UARTx_BDH = (UARTx_BDH&~UART_BDH_SBR_MASK) | UART_BDH_SBR((ubd>>8));
   UARTx_BDL = UART_BDL_SBR(ubd);

   // Determine fractional divider to get closer to the baud rate
//   brfa     = (uint8_t)(((sysclk*32000)/(baud * 16)) - (ubd * 32));
   UARTx_C4 = 0x00;  

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
      case 7  :
         switch (lineCoding.bParityType) {
            case 1:  UARTC1Value = UART_C1_PE_MASK|UART_C1_PT_MASK; break; // Odd
            case 2:  UARTC1Value = UART_C1_PE_MASK;                 break; // Even
         }
           break;
      case 8  :
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
   UARTx_C1 = UARTC1Value;
   UARTx_C3 = UARTC3Value;
   UARTx_C2 = UART_C2_RIE_MASK|UART_C2_RE_MASK|UART_C2_TE_MASK; // Enable Rx/Tx with interrupts
   UARTx_C3 = UART_C3_FEIE_MASK|UART_C3_NEIE_MASK|UART_C3_ORIE_MASK|UART_C3_PEIE_MASK;
   
   // Discard any data in buffers
   rxBufferCount = 0;
   txBufferCount = 0;
   
   enableUartIrq();
}

//! Get CDC Tx characteristics
//!
//! @param lineCodingStructure - Structure describing desired settings
//!
const LineCodingStructure *cdc_getLineCoding(void) {
   return &lineCoding;  
}

//! Set CDC Line values
//!
//! @param value - Describing desired settings
//!
void cdc_setControlLineState(uint8_t value) {
#define LINE_CONTROL_DTR (1<<0)
#define LINE_CONTROL_RTS (1<<1) // Ignored
   
   (void) value; // remove warning
   // Temp fix until I can determine why the value is incorrect
   DTR_ACTIVE();
// if (value & (LINE_CONTROL_DTR|LINE_CONTROL_RTS)) {
//    DTR_ACTIVE();
// }
// else {
//    DTR_INACTIVE();
// }
}

//! Send CDC break
//! 
//! @param length - length of break in milliseconds (see note)\n
//!  - 0x0000 => End BREAK
//!  - 0xFFFF => Start indefinite BREAK
//!  - else   => Send a 10 BREAK 'chars'
//!
//! @note - only partially implemented
//!       - breaks are sent after currently queued characters
//!
void cdc_sendBreak(U16 length) {
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

#endif // (HW_CAPABILITY&CAP_CDC)
