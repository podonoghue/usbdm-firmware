/*! \file
    \brief Simple SCI code JM60
    
   \verbatim
   JMxx SCI Code
    
   Copyright (C) 2010  Peter O'Donoghue

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   \endverbatim

   \verbatim
Change History
+============================================================================================
| 29 Sep 2010 | Created                                                           V4.2 - pgo 
+============================================================================================
   \endverbatim
*/

#include <termio.h>
#include <string.h>
#include "Common.h"
#include "Configure.h"
#include "SCI.h"
#include "USB.h"

#if (HW_CAPABILITY&CAP_CDC)

// This defaults to SCI on JS16 or SCI2 on JMxx
//
#ifndef SCID
#ifdef USE_SCI1
// SCI2
#define SCID              SCI1D

#define SCIS1             SCI1S1
#define SCIS1_RDRF        SCI1S1_RDRF
#define SCIS1_TDRE        SCI1S1_TDRE
#define SCIS1_OR_MASK     SCI1S1_OR_MASK
#define SCIS1_NF_MASK     SCI1S1_NF_MASK
#define SCIS1_FE_MASK     SCI1S1_FE_MASK
#define SCIS1_PF_MASK     SCI1S1_PF_MASK

#define SCIBD             SCI1BD

#define SCIC1             SCI1C1
#define SCIC1_LOOPS_MASK  SCI1C1_LOOPS_MASK
#define SCIC1_PE_MASK     SCI1C1_PE_MASK
#define SCIC1_PT_MASK     SCI1C1_PT_MASK
#define SCIC1_M_MASK      SCI1C1_M_MASK

#define SCIC2             SCI1C2
#define SCIC2_TE_MASK     SCI1C2_TE_MASK
#define SCIC2_RE_MASK     SCI1C2_RE_MASK
#define SCIC2_TIE         SCI1C2_TIE
#define SCIC2_TIE_MASK    SCI1C2_TIE_MASK
#define SCIC2_RIE_MASK    SCI1C2_RIE_MASK
#define SCIC2_SBK         SCI1C2_SBK

#define SCIC3             SCI1C3
#define SCIC3_T8_MASK     SCI1C3_T8_MASK
#define SCIC3_FEIE_MASK   SCI1C3_FEIE_MASK
#define SCIC3_NEIE_MASK   SCI1C3_NEIE_MASK
#define SCIC3_ORIE_MASK   SCI1C3_ORIE_MASK
#define SCIC3_PEIE_MASK   SCI1C3_PEIE_MASK
#else
// SCI2
#define SCID              SCI2D

#define SCIS1             SCI2S1
#define SCIS1_RDRF        SCI2S1_RDRF
#define SCIS1_TDRE        SCI2S1_TDRE
#define SCIS1_OR_MASK     SCI2S1_OR_MASK
#define SCIS1_NF_MASK     SCI2S1_NF_MASK
#define SCIS1_FE_MASK     SCI2S1_FE_MASK
#define SCIS1_PF_MASK     SCI2S1_PF_MASK

#define SCIBD             SCI2BD

#define SCIC1             SCI2C1
#define SCIC1_LOOPS_MASK  SCI2C1_LOOPS_MASK
#define SCIC1_PE_MASK     SCI2C1_PE_MASK
#define SCIC1_PT_MASK     SCI2C1_PT_MASK
#define SCIC1_M_MASK      SCI2C1_M_MASK

#define SCIC2             SCI2C2
#define SCIC2_TE_MASK     SCI2C2_TE_MASK
#define SCIC2_RE_MASK     SCI2C2_RE_MASK
#define SCIC2_TIE         SCI2C2_TIE
#define SCIC2_TIE_MASK    SCI2C2_TIE_MASK
#define SCIC2_RIE_MASK    SCI2C2_RIE_MASK
#define SCIC2_SBK         SCI2C2_SBK

#define SCIC3             SCI2C3
#define SCIC3_T8_MASK     SCI2C3_T8_MASK
#define SCIC3_FEIE_MASK   SCI2C3_FEIE_MASK
#define SCIC3_NEIE_MASK   SCI2C3_NEIE_MASK
#define SCIC3_ORIE_MASK   SCI2C3_ORIE_MASK
#define SCIC3_PEIE_MASK   SCI2C3_PEIE_MASK
#endif
#endif

#define SCI_TX_BUFFER_SIZE (16)  // Should equal end-point buffer size 
static char txBuffer[SCI_TX_BUFFER_SIZE];
static U8 txHead        = 0;
static U8 txBufferCount = 0;
static U8 breakCount    = 0;
#define SCI_RX_BUFFER_SIZE (16)  // Should less than or equal to end-point buffer size 
static char *rxBuffer;
static U8 rxBufferCount = 0;
static U8 sciStatus = SERIAL_STATE_CHANGE;

// The following routines are assumed to be called from interrupt code - I masked
//

//
// Simple double-buffering for Rx (in conjunction with USB buffer)
//

//! putRxBuffer() -  Add a char to the SCI-Rx buffer
//!
//! @param ch - char to add
//!
//! @note Overun flag is set on buffer full
//!
void putRxBuffer(char ch) {
   if (rxBufferCount >= SCI_RX_BUFFER_SIZE) {
      sciStatus |= SCIS1_OR_MASK;
      return;
   }
   rxBuffer[rxBufferCount++] = ch;
//   if (rxBufferCount == SCI_RX_BUFFER_SIZE) {
//      checkUsbCdcRxData();
//   }
}

//! getRxBuffer() - Sets SCI-Rx buffer
//!
//! @param buffer - buffer to write future data to
//!
//! @return -  number of characters in existing buffer
//!
uint8_t setRxBuffer(char *buffer) {
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
uint8_t rxBufferItems(void) {
	return rxBufferCount;
}

//
// Simple double-buffering for SCI-Tx (in conjunction with USB end-point buffer)
//

//! putTxBuffer() -  Copy characters to the SCI-Tx buffer
//! SCI interrupts are enabled.
//!
//! @param source - buffer of source chars
//! @param size   - number of source characters in buffer
//!
//! @return - 0 - OK
//!           1 - Buffer is busy (overrun)
//!
uint8_t putTxBuffer(char *source, uint8_t size) {
   if (txBufferCount > 0) {
	   return 1; // Busy
   }
   (void)memcpy(txBuffer, source, size);
   txHead        = 0;
   txBufferCount = size;
   SCIC2_TIE = 1;    // Enable SCI Tx interrupts
   return 0;
}

//! getTx() -  Gets a character from the SCI-Tx queue.
//!
//! @return 
//!  -  -ve => queue is empty \n
//!  -  +ve => char from queue
//!
static int getTxBuffer(void) {
   U8 ch;
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

//! sciTxSpace - check if SCI-Tx buffer is free
//!
//! @return 0 => buffer is occupied
//!         1 => buffer is free
//!
U8 sciTxBufferFree(void) {
   return (txBufferCount == 0);
}

uint8_t getSerialState(void) {
   uint8_t status = 0x3;
   static uint8_t lastSciStatus = 0x00;

   if (sciStatus&SCIS1_FE_MASK)
	   status |= 1<<4; 
   if (sciStatus&SCIS1_OR_MASK)
	   status |= 1<<6; 
   if (sciStatus&SCIS1_PF_MASK)
	   status |= 1<<5;
   if (lastSciStatus != sciStatus) {
      lastSciStatus = sciStatus;
      status |= SERIAL_STATE_CHANGE;
   }
   sciStatus = 0;
   return status;
}

#pragma TRAP_PROC
void sciErrorHandler(void) {
   // Record and clear error status
   sciStatus |= SCIS1;
   (void)SCID;
}

//! Interrupt handler for SCI-Tx \n
//! Transfers a char from the SCI-Tx queue to SCID
//!
//! @note Interrupts are disabled on empty queue
//!
#pragma TRAP_PROC
void sciTxHandler(void) {
int ch;

   (void)SCIS1; // Dummy read
   ch = getTxBuffer();
   if (ch >= 0) {
      SCID = (U8)ch; // Send the char
   }
   else if (breakCount > 0) {
	  SCIC2_SBK = 1; // Send another BREAK 'char'
	  SCIC2_SBK = 0;
	  if (breakCount != 0xFF) {
		  breakCount--;
      }
   }
   else {
      SCIC2_TIE = 0; // Disable future interrupts
   }
}

//! Interrupt handler for SCI Rx \n
//! Transfers a char from the SCI Rx to USB IN queue
//! 
//! @note Overruns are ignored
//!
#pragma TRAP_PROC
void sciRxHandler(void) {
   (void)SCIS1; // Dummy read
   putRxBuffer(SCID);
}

#define SWAP32(value)    \
   {                     \
   asm (ldx  value:0);   \
   asm (lda  value:3);   \
   asm (stx  value:3);   \
   asm (sta  value:0);   \
   asm (ldx  value:1);   \
   asm (lda  value:2);   \
   asm (stx  value:2);   \
   asm (sta  value:1);   \
   }
#define BAUDDIVIDER(x)  ((BUS_FREQ/16)/(x))

//static const LineCodingStructure defaultLineCoding = {CONST_NATIVE_TO_LE32(9600UL),0,1,8};
static LineCodingStructure lineCoding              = {CONST_NATIVE_TO_LE32(9600UL),0,1,8};

//! Set SCI Tx characteristics
//!
//! @param lineCodingStructure - Structure describing desired settings
//!
//! The SCI is quite limited when compared to the serial interface implied by
//! LineCodingStructure.
//! It does not support many of the combinations available.
//!
void sciSetLineCoding(const LineCodingStructure *lineCodingStructure) {
   U32 baudrate;
   U8 SCIC1Value = 0x00;
   U8 SCIC3Value = 0x00;

   sciStatus  = SERIAL_STATE_CHANGE;
   breakCount = 0; // Clear any current BREAKs
   
   (void)memcpy(&lineCoding, lineCodingStructure, sizeof(LineCodingStructure));

   // Determine baud rate divider
   baudrate = lineCoding.dwDTERate;
   SWAP32(baudrate); // Convert to BE

   // Note - for a 24MHz bus speed the useful baud range is ~300 to ~115200 for 0.5% error
   //        230400 & 460800 have a 8.5% error
   SCIBD = (U16)BAUDDIVIDER(baudrate)&0x3FFFU;

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
    	      case 1:  SCIC1Value = SCIC1_PE_MASK|SCIC1_PT_MASK; break; // Odd
    	      case 2:  SCIC1Value = SCIC1_PE_MASK;               break; // Even
    	   }
           break;
      case 8  :
    	  SCIC1Value = SCIC1_M_MASK; // 9-data or 8-data+parity
    	   switch (lineCoding.bParityType) {
    	      case 0:  SCIC1Value  = 0;                           break; // None
    	      case 1:  SCIC1Value |= SCIC1_PE_MASK|SCIC1_PT_MASK; break; // Odd
    	      case 2:  SCIC1Value |= SCIC1_PE_MASK;               break; // Even
    	      case 3:  SCIC3Value  = SCIC3_T8_MASK;               break; // Mark
    	      case 4:                                             break; // Space
    	   }
    	   break;
	  default :
		  break;
   }
   SCIC1 = SCIC1Value;
   SCIC3 = SCIC3Value;
   SCIC2 = SCIC2_RIE_MASK|SCIC2_RE_MASK|SCIC2_TE_MASK; // Enable Rx/Tx with interrupts
   SCIC3 = SCIC3_FEIE_MASK|SCIC3_NEIE_MASK|SCIC3_ORIE_MASK|SCIC3_PEIE_MASK;
   
   // Discard any data in buffers
   rxBufferCount = 0;
   txBufferCount = 0;
}

//! Get SCI Tx characteristics
//!
//! @param lineCodingStructure - Structure describing desired settings
//!
const LineCodingStructure *sciGetLineCoding(void) {
   return &lineCoding;	
}

//! Set SCI Line values
//!
//! @param value - Describing desired settings
//!
void sciSetControlLineState(U8 value) {
#define LINE_CONTROL_DTR (1<<0)
#define LINE_CONTROL_RTS (1<<1) // Ignored
	
	(void) value; // remove warning
	// Temp fix until I can determine why the value is incorrect
	DTR_ACTIVE();
//	if (value & (LINE_CONTROL_DTR|LINE_CONTROL_RTS)) {
//		DTR_ACTIVE();
//	}
//	else {
//		DTR_INACTIVE();
//	}
}

//! Send SCI break
//! 
//! @param length - length of break in milliseconds (see note)\n
//!  - 0x0000 => End BREAK
//!  - 0xFFFF => Start indefinite BREAK
//!  - else   => Send a 10 BREAK 'chars'
//!
//! @note - only partially implemented
//!       - breaks are sent after currently queued characters
//!
void sciSendBreak(U16 length) {
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


//struct {
//	U32 baudrate;
//	U16 baudDivider;
//} baudTable[] = {
//	{921600UL, BAUDDIVIDER(921600UL)},
//	{460800UL, BAUDDIVIDER(460800UL)},
//	{230400UL, BAUDDIVIDER(230400UL)},
//	{115200UL, BAUDDIVIDER(115200UL)},
//	{ 57600UL, BAUDDIVIDER( 57600UL)},
//	{ 38400UL, BAUDDIVIDER( 38400UL)},
//	{ 19200UL, BAUDDIVIDER( 19200UL)},
//	{  9600UL, BAUDDIVIDER(  9600UL)},
//	{  4800UL, BAUDDIVIDER(  4800UL)},
//	{  2400UL, BAUDDIVIDER(  2400UL)},
//	{  1200UL, BAUDDIVIDER(  1200UL)},
//	{   300UL, BAUDDIVIDER(   300UL)},
//	{     0,   BAUDDIVIDER(  9600UL)},
//};
//{
	//   SCIC1 = SCIC1_LOOPS_MASK; // Debug
	   
	//   while (baudrate<baudTable[sub++].baudrate) {
	//   }
	//   SCIBD = baudTable[sub].baudDivider;     // Set baud rate divider
	//   
	//   baudrate = baudTable[sub++].baudrate;   // Record actual rate used
	//   SWAP32(baudrate); // Convert to LE
	//   lineCoding.dwDTERate =  baudrate;
//}
#endif // (HW_CAPABILITY&CAP_CDC)
