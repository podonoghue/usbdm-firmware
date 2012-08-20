/*! \file
    \brief USBDM - JTAG communication.

   \verbatim
   This software was modified from TBLCF software
   This software was modified from TBDML software

   USBDM
   Copyright (C) 2007  Peter O'Donoghue

   Turbo BDM Light
   Copyright (C) 2005  Daniel Malik

   Turbo BDM Light ColdFire
   Copyright (C) 2005  Daniel Malik

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
   +=======================================================================================
   | 20 Aug 2011 | Changes so TCLK consistently idles low                      V3.7   - pgo
   |  4 Aug 2011 | Added JTAG_DRV control                                      V3.7   - pgo
   | 24 Mar 2011 | Added TDI idle value control                                V4.6   - pgo
   | 21 Jun 2010 | Moved to new module                                         V3.5   - pgo
   +=======================================================================================
   \endverbatim
 */

#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "BDM.h"
#include "BDM_CF.h"
#include "CmdProcessing.h"
#include "BDMCommon.h"
#include "SPI.h"
#include "JTAGSequence.h"

#pragma MESSAGE DISABLE C4301 // Disable warnings about inline expansion

//#pragma DATA_SEG __SHORT_SEG Z_PAGE
// MUST be placed into the direct segment (assumed in ASM code).
//extern volatile U8 bitCount;  //!< Used as a general purpose local variable
//#pragma DATA_SEG DEFAULT

#if !(HW_CAPABILITY&CAP_JTAG_HW)
void jtag_init(void) { }
#else

#define TDI_IDLE_HIGH 1
// Fill byte used for TDI when don't care
#ifdef TDI_IDLE_HIGH
static U8 jtagFillByte = 0xFF;
#else
static U8 jtagFillByte = 0x00;
#endif

//#define USE_SPI_FOR_JTAG // Define to use SPI for JTAG - broken on DSC?

#ifdef USE_SPI_FOR_JTAG
extern U8 SPIBaud;

//======================================================================================================
// JTAG support
//
//  The JTAG interface makes use of both SPI interfaces:
//    One SPI operates as a master and handles the TDI and TCLK outputs and TDO input.
//    The second SPI operates as a slave and handles the TMS output.
//  Since the SPI is byte orientated there are some inefficiencies where additional bits are transmitted.  
//  These will not affect operation for a correctly implemented TAP controller!
//
#define SPIxC1_OFF        (                                 SPI1C1_MSTR_MASK) //!< Mask to disable SPI
#define SPIxC1_JTAG_M_ON  (SPI1C1_SPE_MASK|SPI1C1_CPOL_MASK|SPI1C1_CPHA_MASK|SPI1C1_LSBFE_MASK|SPI1C1_MSTR_MASK) //!< Mask to enable SPI as master
#define SPIxC1_JTAG_S_ON  (SPI1C1_SPE_MASK|SPI1C1_CPOL_MASK|SPI1C1_CPHA_MASK|SPI1C1_LSBFE_MASK                 ) //!< Mask to enable SPI as slave
#define SPIxC2_JTAG_M     (0                                   ) //!< 8-bit mode
#define SPIxC2_JTAG_S     (SPI1C2_BIDIROE_MASK|SPI1C2_SPC0_MASK) //!< 8-bit mode + single pin I/O=O

//!  Sets the JTAG hardware interface to an idle condition
//!
//! \verbatim
//!  Port configuration
//!  Name         uC pin function           BDM cable
//! ===================================================
//!  TDO/DSO      input & externally driven input
//!  --------------------------------------------------
//!  TDI/DSI      output       0            0 
//!  TDI/DSI_DDR  output       0            -
//!  TRST*/DSCLK  3-state      -            - 
//!  TMS/BKPT*    output       1            1 
//!  TCLK         output       1            1 
//!  --------------------------------------------------
//!  RSTO         input & externally driven input
//!  RSTI         input & pulled high       3-state
//!  TA           input & pulled high       3-state
//!  --------------------------------------------------
//! \endverbatim
//! 
static void jtag_interfaceIdle(void) {

   DATA_PORT     = JTAG_IDLE;
   DATA_PORT_DDR = JTAG_IDLE_DDR; 
   RESET_3STATE();  
   TA_3STATE();  
   TRST_3STATE();
   TCLK_HIGH();   // TCLK idles high
}

//! Initialises the TAP controller
//!
//!  The TAP controller is left in TEST-LOGIC-RESET state
//!
void jtag_init(void) {
   //U32 idcode;

   (void)initJTAGSequence();   

   //   TMS_LOW();       // assert TMS
   //   RESET_LOW();     // assert RESET 
   //   WAIT_MS(50);     // 50ms 
   //   RESET_3STATE();  // unassert RESET 
   //   WAIT_MS(10);     // 10ms 
   //   TMS_HIGH();      // unassert TMS

   jtagFillByte   = 0xFF;

   TDO_IN_DDR     = 0;
   RESET_IN_DDR   = 0;
   DSCLK_DRV_DISABLE();   // DSCLK signal become TRST*

   jtag_interfaceIdle();  // JTAG mode
   // Now the JTAG pins are in their default states

   // Enable SPI1 as master (TDO,TDI,TCLK)
   SPI1C1 = SPIxC1_OFF;
   SPI1C2 = SPIxC2_JTAG_M;
   SPI1BR = SPIBaud;
   SPI1C1 = SPIxC1_JTAG_M_ON; 

   // Enable SPI2 as slave to SPI1 (TMS)
   SPI2C1 = SPIxC1_OFF;
   SPI2C2 = SPIxC2_JTAG_S;
   SPI2BR = SPIBaud;
   SPI2C1 = SPIxC1_JTAG_S_ON; 

   // Force async reset of TAP to TEST-LOGIC-RESET
   TRST_LOW();    // assert TRST 
   WAIT_MS(50);   // 50ms 
   TRST_3STATE(); // release TRST 
   WAIT_MS(10);   // 10ms 

   // Dummy reads of SPI status
   (void)SPI1S;
   (void)SPI2S;

   // Dummy reads of SPI data
   (void)SPI1D;
   (void)SPI2D;

#if 0   
   // Clocked transition of TAP to TEST-LOGIC-RESET
   jtag_transition_reset();

   jtag_transition_shift(1);   // SHIFT-IR
   jtag_write(1, 4, "\x02");   // Instruction = IDCODE (0010)
   jtag_transition_shift(0);   // SHIFT-DR
   idcode = 0x00000000;
   jtag_read(1, 32, (U8 *)&idcode); // Read IDCODE
#endif   
}

//! Transitions the TAP controller to TEST-LOGIC-RESET state
//! Makes no assumptions about initial TAP state
//!
void jtag_transition_reset(void) {

   while ((SPI1S_SPTEF == 0)||(SPI2S_SPTEF == 0)){  // Wait for Tx buffer empty
   }
   SPI2D = 0xFF;              // TMS = 1111_1XXX = TEST-LOGIC-RESET
   asm(nop); asm(nop); asm(nop); asm(nop);
   SPI1D = jtagFillByte;      // TDI = don't care
   while ((SPI1S_SPRF == 0)||(SPI2S_SPRF == 0)){  // Wait for Rx buffer full
   }
   // Discard read data
   (void)SPI1D;
   (void)SPI2D;
}

//! Transitions the TAP controller to SHIFT-DR or SHIFT-IR state
//! Assumes TAP is in TEST-LOGIC-RESET or RUN-TEST/IDLE
//!
//! @param mode  \ref SHIFT_DR => TAP controller is left in SHIFT-DR \n
//!              \ref SHIFT_IR => TAP controller is left in SHIFT-IR
//!
void jtag_transition_shift(U8 mode) {

   while ((SPI1S_SPTEF == 0)||(SPI2S_SPTEF == 0)){  // Wait for Tx buffer empty
   }
   if (mode != 0)
      SPI2D = 0x30;           // TMS = 0011_0000 = SHIFT-IR
   else                       //         or
      SPI2D = 0x20;           //       0010_0000 = SHIFT-DR
   asm(nop); asm(nop); asm(nop); asm(nop);
   SPI1D = jtagFillByte;      // TDI = don't care
   while ((SPI1S_SPRF == 0)||(SPI2S_SPRF == 0)){  // Wait for Rx buffer full
   }
   // Discard read data
   (void)SPI1D;
   (void)SPI2D;
}

//! Writes given bit stream into data/instruction path of the JTAG
//!
//! @param options \ref JTAG_STAY_SHIFT    => leave the TAP state unchanged [SHIFT-DR or SHIFT-IR]  \n
//!                \ref JTAG_EXIT_SHIFT_DR => leave the TAP in SHIFT-DR                             \n
//!                \ref JTAG_EXIT_SHIFT_IR => leave the TAP in SHIFT-IR                             \n
//!                \ref JTAG_EXIT_IDLE     => leave the TAP in RUN-TEST/IDLE
//! @param bitCount   => specifies the number of bits to write [>0] [it is not possible to do 1 bit]
//! @param writePtr   => pointer to block of data
//! @note  Data are transmitted starting with LSB of the LAST byte in the supplied buffer (for
//!        easier readability of code which uses JTAG)
//! @note  On entry, expects to find the TAP controller in SHIFT-DR or SHIFT-IR state
//!
void jtag_write(U8 options, U8 bitCount, const U8 *writePtr) {
   const static U8 FinishTMS[] = {0,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

   writePtr += (bitCount-1)>>3;  // Each byte has 8 bits, point to the last byte in the buffer 

   while ((SPI1S_SPTEF == 0)||(SPI2S_SPTEF == 0)){  // Wait for Tx buffer empty
   }
   // Transmit data bytes
   while (bitCount > 0) {
      if (bitCount > 8) {     // Full byte
         SPI2D     = 0x00;    // TMS = 0 - remain in SHIFT-DR/IR
         bitCount -= 8;
      }
      else {
         // Transmit last/partial byte - makes use of EXIT1-DR/IR & PAUSE-DR/IR to skip unused bits
         // TAP is left in EXIT1-DR/IR or PAUSE-DR/IR
         SPI2D    = FinishTMS[bitCount];  // TMS = ....1.. 
         bitCount = 0;
      }
      asm(nop); asm(nop); asm(nop); asm(nop);
      SPI1D = *writePtr--;             // TDI = date byte
      while ((SPI1S_SPRF == 0)||(SPI2S_SPRF == 0)){  // Wait for Rx buffer full
      }
      // Discard read data
      (void)SPI1D;
      (void)SPI2D;
   }

   while ((SPI1S_SPTEF == 0)||(SPI2S_SPTEF == 0)){  // Wait for Tx buffer empty
   }
   // Return TAP to SHIFT-DR/IR or move to RUN-TEST/IDLE
   switch (options&0x0F) {
   case JTAG_STAY_SHIFT :
      SPI2D = 0x40;           // TMS = 0100_0000, return to SHIFT-DR/IR
      break;
   case JTAG_EXIT_SHIFT_DR :
      SPI2D = 0x38;           // TMS = 0011_1000, move to SHIFT-DR
      break;
   case JTAG_EXIT_SHIFT_IR :
      SPI2D = 0x3C;           // TMS = 0011_1100, move to SHIFT-IR
      break;
   default:
   case JTAG_EXIT_IDLE :
      SPI2D = 0x60;           // TMS = 0110_0000, move to RUN-TEST/IDLE
      break;             
   }
   asm(nop); asm(nop); asm(nop); asm(nop);
   SPI1D = jtagFillByte;      // TDI = don't care
   while ((SPI1S_SPRF == 0)||(SPI2S_SPRF == 0)){  // Wait for Rx buffer full
   }
   // Discard read data
   (void)SPI1D;
   (void)SPI2D;
}

//! Reads bitstream out of JTAG
//!
//! @param options \ref JTAG_STAY_SHIFT    => leave the TAP state unchanged [SHIFT-DR or SHIFT-IR]  \n
//!                \ref JTAG_EXIT_SHIFT_DR => leave the TAP in SHIFT-DR                             \n
//!                \ref JTAG_EXIT_SHIFT_IR => leave the TAP in SHIFT-IR                             \n
//!                \ref JTAG_EXIT_IDLE     => leave the TAP in RUN-TEST/IDLE                        \n
//!                + \ref JTAG_WRITE_1     => write 1's while reading
//!                + \ref JTAG_WRITE_0     => write 0's while reading
//! @param bitCount   => specifies the number of bits to read [>0] [it is not possible to do 1 bit]
//! @param readPtr    => pointer to buffer for data
//! @note  Data are stored starting with LSB of the LAST byte in the supplied buffer (for
//!        easier readability of code which uses JTAG)
//! @note  On entry, expects to find the TAP in SHIFT-DR or SHIFT-IR state
//!
void jtag_read(U8 options, U8 bitCount, U8 *readPtr) {
   const static U8 finishTMS[] = {0,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
   const static U8 dataMasks[] = {0,0x01,0x03,0x07,0x0F,0x1F,0x3F,0x7F,0xFF};
   U8 dataMask    = 0xFF;

   jtagFillByte = (options&JTAG_WRITE_1)?0xFF:0x00;
   readPtr += (bitCount-1)>>3;  // Each byte has 8 bits, point to the last byte in the buffer 

   while ((SPI1S_SPTEF == 0)||(SPI2S_SPTEF == 0)){  // Wait for Tx buffer empty
   }
   // Receive data bytes
   while (bitCount > 0) {
      if (bitCount > 8) { // Full byte
         SPI2D     = 0x00;         // TMS = 0 - remain in SHIFT-DR/IR
         bitCount -= 8;
      }
      else {
         // Transmit last/partial byte - makes use of 
         //    EXIT1-DR/IR & PAUSE-DR/IR to skip unused bits
         // TAP is left in  EXIT1-DR/IR or PAUSE-DR/IR
         SPI2D    = finishTMS[bitCount];  // TMS = ...010... 
         dataMask = dataMasks[bitCount];  // Mask for last byte
         bitCount = 0;
      }
      asm(nop); asm(nop); asm(nop); asm(nop);
      SPI1D = jtagFillByte;              // TDI = dummy byte
      while ((SPI1S_SPRF == 0)||(SPI2S_SPRF == 0)){  // Wait for Rx buffer full
      }
      *readPtr-- = SPI1D & dataMask; // data byte = TDO
      // Discard read data
      (void)SPI2D;
   }

   while ((SPI1S_SPTEF == 0)||(SPI2S_SPTEF == 0)){  // Wait for Tx buffer empty
   }
   // Return TAP to SHIFT-DR/IR or move to RUN-TEST/IDLE
   switch (options&0x0F) {
   case JTAG_STAY_SHIFT :
      SPI2D = 0x40;           // TMS = 0100_0000, return to SHIFT-DR/IR
      break;
   case JTAG_EXIT_SHIFT_DR :
      SPI2D = 0x38;           // TMS = 0011_1000, move to SHIFT-DR
      break;
   case JTAG_EXIT_SHIFT_IR :
      SPI2D = 0x3C;           // TMS = 0011_1100, move to SHIFT-IR
      break;
   default:
   case JTAG_EXIT_IDLE :
      SPI2D = 0x60;           // TMS = 0110_0000, move to RUN-TEST/IDLE
      break;             
   }
   asm(nop); asm(nop); asm(nop); asm(nop);
   SPI1D = jtagFillByte;      // TDI = don't care
   while ((SPI1S_SPRF == 0)||(SPI2S_SPRF == 0)){  // Wait for Rx buffer full
   }
   // Discard read data
   (void)SPI1D;
   (void)SPI2D;
}

//! Reads bitstream out of JTAG
//!
//! @param options   \ref JTAG_STAY_SHIFT    => leave the TAP state unchanged [SHIFT-DR or SHIFT-IR] \n
//!                  \ref JTAG_EXIT_SHIFT_DR => leave the TAP in SHIFT-DR                            \n
//!                  \ref JTAG_EXIT_SHIFT_IR => leave the TAP in SHIFT-IR                            \n
//!                  \ref JTAG_EXIT_IDLE     => leave the TAP in RUN-TEST/IDLE
//! @param bitCount   => specifies the number of bits to read/write [>0] [it is not possible to do 1 bit]
//! @param readWritePtr => pointer to buffer for data
//! @note  Data are stored starting with LSB of the LAST byte in the supplied buffer (for
//!        easier readability of code which uses JTAG)
//! @note  On entry, expects to find the TAP in SHIFT-DR or SHIFT-IR state
//!
void jtag_read_write(U8 options, U8 bitCount, const U8 *writePtr, U8 *readPtr) {
   const static U8 finishTMS[] = {0,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
   const static U8 dataMasks[] = {0,0x01,0x03,0x07,0x0F,0x1F,0x3F,0x7F,0xFF};
   U8 dataMask    = 0xFF;

   readPtr   += (bitCount-1)>>3;  // Each byte has 8 bits, point to the last byte in the buffer 
   writePtr  += (bitCount-1)>>3;  // Each byte has 8 bits, point to the last byte in the buffer 

   while ((SPI1S_SPTEF == 0)||(SPI2S_SPTEF == 0)){  // Wait for Tx buffer empty
   }
   // Receive data bytes
   while (bitCount > 0) {
      if (bitCount > 8) {   // Full byte
         SPI2D     = 0x00;  // TMS = 0 - remain in SHIFT-DR/IR
         bitCount -= 8;
      }
      else {
         // Transmit last/partial byte - makes use of 
         //    EXIT1-DR/IR & PAUSE-DR/IR to skip unused bits
         // TAP is left in  EXIT1-DR/IR or PAUSE-DR/IR
         SPI2D    = finishTMS[bitCount];  // TMS = ...010... 
         dataMask = dataMasks[bitCount];  // Mask for last byte
         bitCount = 0;
      }
      asm(nop); asm(nop); asm(nop); asm(nop);
      SPI1D = *writePtr--;           // TDI = write data byte
      while ((SPI1S_SPRF == 0)||(SPI2S_SPRF == 0)){  // Wait for Rx buffer full
      }
      *readPtr-- = SPI1D & dataMask; // date byte = TDO
      // Discard read data
      (void)SPI2D;
   }

   while ((SPI1S_SPTEF == 0)||(SPI2S_SPTEF == 0)){  // Wait for Tx buffer empty
   }
   // Return TAP to SHIFT-DR/IR or move to RUN-TEST/IDLE
   switch (options&0x0F) {
   case JTAG_STAY_SHIFT :
      SPI2D = 0x40;           // TMS = 0100_0000, return to SHIFT-DR/IR
      break;
   case JTAG_EXIT_SHIFT_DR :
      SPI2D = 0x38;           // TMS = 0011_1000, move to SHIFT-DR
      break;
   case JTAG_EXIT_SHIFT_IR :
      SPI2D = 0x3C;           // TMS = 0011_1100, move to SHIFT-IR
      break;
   default:
   case JTAG_EXIT_IDLE :
      SPI2D = 0x60;           // TMS = 0110_0000, move to RUN-TEST/IDLE
      break;             
   }
   asm(nop); asm(nop); asm(nop); asm(nop);
   SPI1D = jtagFillByte;      // TDI = don't care
   while ((SPI1S_SPRF == 0)||(SPI2S_SPRF == 0)){  // Wait for Rx buffer full
   }
   // Discard read data
   (void)SPI1D;
   (void)SPI2D;
}

#else

extern U8 SPIBaud;

//======================================================================================================
// JTAG support
//
// Software bit-banging method
//
static U16 dataRegisterHeader         = 0; 
static U16 dataRegisterTrailer        = 0; 
static U16 instructionRegisterHeader  = 0; 
static U16 instructionRegisterTrailer = 0; 

typedef enum {test_logic_reset, run_test_idle, 
   select_dr_scan, capture_dr, shift_dr, exit1_dr, pause_dr, exit2_dr, update_dr, 
   select_ir_scan, capture_ir, shift_ir, exit1_ir, pause_ir, exit2_ir, update_ir, 
} TapStates_t;

TapStates_t tapState = test_logic_reset;

void jtag_set_hdr(U16 value) {
   dataRegisterHeader = value; 
}

void jtag_set_hir(U16 value) {
   instructionRegisterHeader = value; 
}

void jtag_set_tdr(U16 value) {
   dataRegisterTrailer = value; 
}

void jtag_set_tir(U16 value) {
   instructionRegisterTrailer = value; 
}

//!  Sets the JTAG hardware interface to an idle condition
//!
//! \verbatim
//!  Port configuration
//!  Name           uC pin function            BDM cable
//! =====================================================
//!  TDO/DSO        input & externally driven  input
//!  ----------------------------------------------------
//!  TDI/DSI_O      output       0             0 
//!  TDI/DSI_DDR    output       0             -
//!  TRST*/DSCLK_O  3-state      -             - 
//!  TMS/BKPT_O*    output       0             0 
//!  TCLK_O         output       0             1 
//!  JTAG_DRV       output       0             -
//!  ----------------------------------------------------
//!  RST_I          input & externally driven  input
//!  RST_O          input & pulled high        3-state
//!  TA_O           input & pulled high        3-state
//!  ----------------------------------------------------
//! \endverbatim
//! 
void jtag_interfaceIdle(void) {

#ifdef DATA_PORT  
   DATA_PORT     = JTAG_IDLE;
   DATA_PORT_DDR = JTAG_IDLE_DDR;
#endif
#if (HW_CAPABILITY & CAP_CFVx_HW)
   TA_3STATE();  
#endif   
   RESET_3STATE();  
   TRST_3STATE();
   TDO_IN_DDR  = 0;  // TDO_IN input
   TDI_OUT_DDR = 1;  // TDI_OUT low
   if (jtagFillByte)
      TDI_HIGH();
   else
      TDI_LOW();
   TMS_OUT_DDR = 1;  // TMS low
   TMS_LOW();
   TCLK_OUT_DDR = 1; // TCLK idles low
   TCLK_LOW();      
   JTAG_DRV_ENABLE();
}

//! Initialises the JTAG Interface
//!
//! @note - This is probably called with the target power off
//!
void jtag_init(void) {
   //U32 idcode;

   (void)initJTAGSequence();   

   jtagFillByte   = 0xFF;

   TDO_IN_DDR     = 0;
   RESET_IN_DDR   = 0;
#ifdef DSCLK_DRV_DISABLE  
   //USBDM HW
   DSCLK_DRV_DISABLE();   // DSCLK signal become TRST*
#endif
#ifdef TCLK_CTL_ENABLE
   TCLK_CTL_ENABLE();  // OSBDM HW
#endif
#ifdef TCLK_ENABLE
   TCLK_ENABLE();   // OSBDM HW
#endif

   jtag_interfaceIdle();  // JTAG mode
   // Now the JTAG pins are in their default states

   dataRegisterHeader  = 0; 
   dataRegisterTrailer = 0; 
   instructionRegisterHeader  = 0; 
   instructionRegisterTrailer = 0; 
}

//! Initialises the JTAG Interface
//!
//! @note - This is probably called with the target power off
//!
void jtag_off(void) {
   RESET_3STATE();  
#ifdef TA_3STATE
   TA_3STATE(); 
#endif
#ifdef TRST_3STATE
   TRST_3STATE();
#endif
#ifdef BKPT_HIGH
   BKPT_HIGH();
#endif
#ifdef JTAG_DRV_DISABLE
   JTAG_DRV_DISABLE();
#endif
#ifdef TCLK_CTL_DISABLE
   TCLK_CTL_DISABLE();
#endif
}

#pragma INLINE
void halfBitDelay(void) {
	asm {					
		  ldx   bitDelay	
		  dbnzx *-0		
	}
}

//! Transitions the TAP controller to TEST-LOGIC-RESET state
//! Makes no assumptions about initial TAP state
//!
void jtag_transition_reset(void) {
   U8 i;
   // TMS = 1,1,1,1,1,1
   if (jtagFillByte)
      TDI_HIGH();
   else
      TDI_LOW();
   TCLK_LOW();
   TMS_HIGH();
   for (i=0; i<=6; i++) {
      TCLK_HIGH();
      halfBitDelay();
      TCLK_LOW();
      halfBitDelay();
   }
   tapState = test_logic_reset;
}

void jtag_header_shift(void) {
   U16 numBits = 0;
   switch (tapState) {
   case shift_dr:      numBits = dataRegisterHeader; break;
   case shift_ir:      numBits = instructionRegisterHeader; break;
   }
   // IR always shifts '1' to keep other devices in bypass
   if ((jtagFillByte) || (tapState == shift_ir))
      TDI_HIGH();
   else
      TDI_LOW();
   while (numBits-->0) {
      TCLK_HIGH();
      halfBitDelay();
      TCLK_LOW();
      halfBitDelay();
   }
}

//! Transitions the TAP controller to SHIFT-DR or SHIFT-IR state
//! Assumes TAP is in TEST-LOGIC-RESET or RUN-TEST/IDLE
//!
//! @param mode  \ref JTAG_SHIFT_DR => TAP controller is moved to SHIFT-DR \n
//!              \ref JTAG_SHIFT_IR => TAP controller is moved to SHIFT-IR
//!
void jtag_transition_shift(U8 mode) {

   // SHIFT-IR TMS = 0,1,1,0,0
   // SHIFT-DR TMS = 0,1,0,0
   if (jtagFillByte)
      TDI_HIGH();
   else
      TDI_LOW();
   TMS_LOW();
   TCLK_HIGH();      // 0
   halfBitDelay();
   TCLK_LOW();
   TMS_HIGH();
   halfBitDelay();
   if (mode == JTAG_SHIFT_IR) {
      TCLK_HIGH();   // 1
      halfBitDelay();
      TCLK_LOW();
      halfBitDelay();
      tapState = shift_ir;
   }
   else {
      tapState = shift_dr;
   }
   TCLK_HIGH();      // 1
   halfBitDelay();
   TCLK_LOW();
   TMS_LOW();
   halfBitDelay();
   TCLK_HIGH();      // 0
   halfBitDelay();
   TCLK_LOW();
   halfBitDelay();
   TCLK_HIGH();      // 0
   halfBitDelay();
   TCLK_LOW();
   halfBitDelay();
   jtag_header_shift();
}

//! Shifts out any TID/TDR bits
//! Assumes the last data bit has been placed on TDI pin but not shifted (TCK is low)
//! Leaves last data/fill bit on TDI pin but not shifted (TCK low)
//
void jtag_trailer_shift(void) {
   U16 numBits = 0;
   switch (tapState) {
   case shift_dr:      numBits = dataRegisterTrailer;        break;
   case shift_ir:      numBits = instructionRegisterTrailer; break;
   }
   while (numBits-->0) {
	  TCLK_HIGH(); // Shift out bit
      halfBitDelay();
      TCLK_LOW();
      halfBitDelay();
      // IR always shifts '1' to keep other devices in bypass
      if ((jtagFillByte) || (tapState == shift_ir))
         TDI_HIGH();
      else
         TDI_LOW();
   }
}

//! Transitions the TAP controller to RUN-TEST/IDLE or SHIFT-DR or SHIFT-IR state
//! Assumes TAP is in SHIFT-IR or SHIFT-DR 
//! Assumes the last actual data bit has been placed on TDI pin but not shifted (TCK is low)
//! Leaves TCK low on exit
//!
//! @param options \ref JTAG_STAY_SHIFT    => leave the TAP state unchanged [SHIFT-DR or SHIFT-IR]  \n
//!                \ref JTAG_EXIT_SHIFT_DR => exit SHIFT-IR/DR & enter SHIFT-DR                     \n
//!                \ref JTAG_EXIT_SHIFT_IR => exit SHIFT-IR/DR & enter SHIFT-IR                     \n
//!                \ref JTAG_EXIT_IDLE     => exit SHIFT-IR/DR & enter RUN-TEST/IDLE
//
void jtag_exit_shift(U8 options) {
   U8 bitCount;
   U8 writeByte;

   if (options != JTAG_STAY_SHIFT) {
      // Shift out any TIR/TDR bits
      jtag_trailer_shift();
      // Do exit state change
	  switch (options) {
	  case JTAG_EXIT_IDLE:      bitCount = 3; writeByte = 0x03; tapState = run_test_idle; break;
	  case JTAG_EXIT_SHIFT_DR:  bitCount = 5; writeByte = 0x07; tapState = shift_dr;      break;
	  case JTAG_EXIT_SHIFT_IR:  bitCount = 6; writeByte = 0x0F; tapState = shift_ir;      break;
	  }
	  while (bitCount-- > 0) {
		 if (writeByte&0x01)
			TMS_HIGH();
		 else
			TMS_LOW();
		 TCLK_HIGH();
		 writeByte >>= 1;
         halfBitDelay();
		 TCLK_LOW();
         halfBitDelay();
		 if (jtagFillByte)
			TDI_HIGH();
		 else
			TDI_LOW();
	  }
   }
   else {
	  // Shift out last data/fill bit & remain in shift
      halfBitDelay();
      TCLK_HIGH();
      halfBitDelay();
      TCLK_LOW();
   }
}

//! Writes given bit stream into data/instruction path of the JTAG
//!
//! @param options \ref JTAG_STAY_SHIFT    => leave the TAP state unchanged [SHIFT-DR or SHIFT-IR]  \n
//!                \ref JTAG_EXIT_SHIFT_DR => leave the TAP in SHIFT-DR                             \n
//!                \ref JTAG_EXIT_SHIFT_IR => leave the TAP in SHIFT-IR                             \n
//!                \ref JTAG_EXIT_IDLE     => leave the TAP in RUN-TEST/IDLE
//!                + \ref JTAG_WRITE_1     => write 1's when TDI not in use
//!                + \ref JTAG_WRITE_0     => write 0's when TDI not in use
//! @param bitCount   => specifies the number of bits to write [>0] [it is not possible to do 1 bit]
//! @param writePtr   => pointer to block of data
//! @note  Data are transmitted starting with LSB of the LAST byte in the supplied buffer (for
//!        easier readability of code which uses JTAG)
//! @note  On entry, expects to find the TAP controller in SHIFT-DR or SHIFT-IR state
//!
void jtag_write(U8 options, U8 bitCount, const U8 *writePtr) {
   U8 writeByte     = 0x00;
   U8 currentBitNum = 0;

   writePtr += (bitCount-1)>>3;  // Each byte has 8 bits, point to the last byte in the buffer 
   jtagFillByte = (options&JTAG_WRITE_1);
   options &= JTAG_EXIT_ACTION_MASK;

   // Transmit data bytes
   while (bitCount-- > 0) {
      if (currentBitNum-- == 0) {
         writeByte   = *writePtr--;
         currentBitNum = 7;
      }
      else {
         writeByte >>= 1;
      }
      if (writeByte&0x01) {
         TDI_HIGH();
      }
      else {
         TDI_LOW();
      }
      if (bitCount != 0) {
    	 // Shift bit
         halfBitDelay();
         TCLK_HIGH();
         halfBitDelay();
         TCLK_LOW();
      }
   }
   // Do exit action
   jtag_exit_shift(options);
   enableInterrupts();
}

//! Reads bitstream out of JTAG
//!
//! @param options \ref JTAG_STAY_SHIFT    => leave the TAP state unchanged [SHIFT-DR or SHIFT-IR]  \n
//!                \ref JTAG_EXIT_SHIFT_DR => leave the TAP in SHIFT-DR                             \n
//!                \ref JTAG_EXIT_SHIFT_IR => leave the TAP in SHIFT-IR                             \n
//!                \ref JTAG_EXIT_IDLE     => leave the TAP in RUN-TEST/IDLE                        \n
//!                + \ref JTAG_WRITE_1     => write 1's while reading
//!                + \ref JTAG_WRITE_0     => write 0's while reading
//! @param bitCount   => specifies the number of bits to read [>0] [it is not possible to do 1 bit]
//! @param readPtr    => pointer to buffer for data
//! @note  Data are stored starting with LSB of the LAST byte in the supplied buffer (for
//!        easier readability of code which uses JTAG)
//! @note  On entry, expects to find the TAP in SHIFT-DR or SHIFT-IR state
//!
void jtag_read(U8 options, U8 bitCount, U8 *readPtr) {
   U8 readByte      = 0x00;
   U8 currentBitNum = 0;

   readPtr += (bitCount-1)>>3;  // Each byte has 8 bits, point to the last byte in the buffer 
   jtagFillByte = (options&JTAG_WRITE_1);
   options     &= JTAG_EXIT_ACTION_MASK;

   // Park TDI
   if (jtagFillByte)
      TDI_HIGH();
   else
      TDI_LOW();

   // Receive data bytes
   while (bitCount-- > 0) {
      if (currentBitNum-- == 0) {
         currentBitNum = 7;
      }
      // Capture TDO
      readByte >>= 1;
      if (TDO_IN)
         readByte |= 0x80;
      if (currentBitNum == 0) {
         *readPtr-- = readByte;
         readByte = 0x00;
      }
      if (bitCount != 0) {
    	  // Shift bit
		  TCLK_HIGH();
	      halfBitDelay();
	      TCLK_LOW();
          halfBitDelay();
      }
   }
   if (currentBitNum>0) {
      // Adjust/save last byte
      while (currentBitNum-->0) {
         readByte >>= 1;
      }
      *readPtr-- = readByte;
   }
   jtag_exit_shift(options);
   enableInterrupts();
}

//! Reads bitstream out of JTAG
//!
//! @param options   \ref JTAG_STAY_SHIFT    => leave the TAP state unchanged [SHIFT-DR or SHIFT-IR] \n
//!                  \ref JTAG_EXIT_SHIFT_DR => leave the TAP in SHIFT-DR                            \n
//!                  \ref JTAG_EXIT_SHIFT_IR => leave the TAP in SHIFT-IR                            \n
//!                  \ref JTAG_EXIT_IDLE     => leave the TAP in RUN-TEST/IDLE
//! @param bitCount   => specifies the number of bits to read/write [>0] [it is not possible to do 1 bit]
//! @param writePtr   => pointer to block of data
//! @param readPtr    => pointer to buffer for data
//! @note  Data are stored starting with LSB of the LAST byte in the supplied buffer (for
//!        easier readability of code which uses JTAG)
//! @note  On entry, expects to find the TAP in SHIFT-DR or SHIFT-IR state
//!
void jtag_read_write(U8 options, U8 bitCount, const U8 *writePtr, U8 *readPtr) {
   U8 writeByte   = 0x00;
   U8 readByte    = 0x00;
   U8 currentBitNum = 0;

   writePtr += (bitCount-1)>>3;  // Each byte has 8 bits, point to the last byte in the buffer 
   readPtr  += (bitCount-1)>>3;   
   jtagFillByte = (options&JTAG_WRITE_1);
   options &= JTAG_EXIT_ACTION_MASK;

   // Transmit data bytes
   while (bitCount-- > 0) {
      if (currentBitNum-- == 0) {
         writeByte   = *writePtr--;
         currentBitNum = 7;
      }
      else
         writeByte >>= 1;
      if (writeByte&0x01)
         TDI_HIGH();
      else
         TDI_LOW();
      // Capture TDO
      readByte >>= 1;
      if (TDO_IN)
         readByte |= 0x80;
      if (currentBitNum == 0) {
         *readPtr-- = readByte;
         readByte = 0x00;
      }
      if (bitCount != 0) {
    	  // Shift bit
         halfBitDelay();
         TCLK_HIGH();
         halfBitDelay();
         TCLK_LOW();
      }
   }
   if (currentBitNum != 0) {
      // Adjust/save last byte
      while (currentBitNum-- != 0) {
         readByte >>= 1;
      }
      *readPtr-- = readByte;
   }
   jtag_exit_shift(options);
   enableInterrupts();
}

#endif

#endif // (HW_CAPABILITY&CAP_CFVx)
