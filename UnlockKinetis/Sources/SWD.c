/*! \file
    \brief ARM-SWD routines

   \verbatim

   USBDM

   Copyright (C) 2007  Peter O'Donoghue

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

   Change History
   +===============================================================================================
   | 30 Aug 2012 | ARM-JTAG & ARM-SWD Changes                                               V4.9.5
   +===============================================================================================
   \endverbatim
*/

#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "SWD.h"
#include "Commands.h"
#include "BDM.h"
//#include "CmdProcessing.h"
#include "BDMCommon.h"
#include "SPI.h"
#include "TargetDefines.h"
#include "SWD.h"

//#define SLOWER_METHOD

#if TARGET_CAPABILITY & CAP_ARM_SWD

//! SPI CTAR value
//#define SPI_CTAR_MASK  (SPI_CTAR_CPOL_MASK|SPI_CTAR_CPHA_MASK|SPI_CTAR_LSBFE_MASK|SPI_CTAR_PASC(1)|SPI_CTAR_ASC(0))
#define SPI_CTAR_MASK  (SPI_CTAR_LSBFE_MASK|SPI_CTAR_PASC(1)|SPI_CTAR_ASC(0))

// SPI_PUSHR_PCS for Tx operations 
#define SWD_PUSHR_TX  SPI_PUSHR_PCS((1<<0)|(1<<1)) // PCS0=SWDIO_O_En, PCS1=SWDCLK_En 

// SPI_PUSHR_PCS for Rx operations 
#define SWD_PUSHR_RX  SPI_PUSHR_PCS((1<<1))        // PCS1=SWDCLK_En

#define SWD_READ_IDCODE 0xA5 // (Park,Stop,Parity,A[32],R/W,AP/DP,Start) = 10100101

// Masks for SWD_WR_DP_ABORT clear sticky
#define SWD_DP_ABORT_CLEAR_STICKY_ERRORS_B3 0x1E

// Masks for SWD_WR_DP_ABORT abort AP
#define SWD_DP_ABORT_ABORT_AP_B3 0x01

// Masks for SWD_RD_DP_STATUS
#define SWD_RD_DP_STATUS_ANYERROR_B3 0xB2

//! Calculate parity
//!
//! @param data data to calculate parity for
//!
//! @return 0/1 indicating even/odd parity
//!
__forceinline
static inline uint8_t calcParity(const uint8_t dataptr[]) {
   uint32_t data = dataptr[0]^dataptr[1]^dataptr[2]^dataptr[3];
   data = (data>>4)^data;
   data = (data>>2)^data;
   data = (data>>1)^data;
   return data&1;
}

//! Sets the SWD interface to an idle state
//! RESET=3-state, SWCLK=High, SWDIO=3-state
//!
void swd_interfaceIdle(void) {
   RESET_3STATE();
}

//! Initialise the SWD interface and sets it to an idle state
//! RESET=3-state, SWCLK=High, SWDIO=3-state, SPI initialised
//!
//! @note This includes once-off initialisation such as PUPs etc 
//!
void swd_init(void) {
   swd_interfaceIdle();
   // 4 pins SWD_OUT, SWD_OUT_EN, SWCLK_OUT, SWCLK_OUT_EN
   SWCLK_OUT_INIT();
   SWD_OUT_INIT();
   SWD_IN_INIT();
   RESET_OUT_INIT();
#ifdef RESET_IN_INIT
   RESET_IN_INIT();
#endif
#ifdef SWCLK_ENABLE
   SWCLK_ENABLE();
#endif
#ifdef SWD_3STATE
   SWD_3STATE();
#endif
   spi_init(SPI_CTAR_MASK|SPI_CTAR_FMSZ(8-1),      // 8-bit transfer
            SPI_CTAR_MASK|SPI_CTAR_FMSZ(4-1));     // 4-bit transfer 
}

//!  Turns off the SWD interface
//!
//!  Depending upon settings, may leave target power on.
//!
void swd_off( void ) {
#if ((HW_CAPABILITY & CAP_FLASH) != 0)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif
   if (!bdm_option.leaveTargetPowered) {
      VDD_OFF();
   }
   swd_interfaceIdle();
#ifdef SWCLK_3STATE
   SWCLK_3STATE();
#endif
#ifdef SWD_3STATE
   SWD_3STATE();
#endif
   SWCLK_OUT_FINI();
   SWD_OUT_FINI();
   SWD_IN_FINI();
   RESET_OUT_FINI();
#ifdef RESET_IN_FINI
   RESET_IN_FINI();
#endif
}

static const int ctas_8bit  = 0;
static const int ctas_Xbit  = 1;

//! Transmit a 8-bit word to the target 
//!
//! @param send    - data to send
//!
//! @return BDM_RC_OK => success
//!
__forceinline
static inline void spi_tx8(uint8_t data) {
   SWD_ENABLE();
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SPI_PUSHR_EOQ_MASK|SWD_PUSHR_TX|SPI_PUSHR_TXDATA(data); 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   (void)SPI0_POPR;              // Discard read data
   SPI0_SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
   SWD_3STATE();
}

//! Transmit a [mark, 8-bit word] to the target 
//!
//! @param send    - data to send
//!
//! @return BDM_RC_OK => success
//!
__forceinline
static inline void spi_mark_tx8(uint8_t data) {
   SWD_ENABLE();
   spi_setCTAR1(SPI_CTAR_MASK|SPI_CTAR_FMSZ(9-1));
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_Xbit)|SPI_PUSHR_EOQ_MASK|SWD_PUSHR_TX|SPI_PUSHR_TXDATA((data<<1)|1); 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   (void)SPI0_POPR;              // Discard read data
   SPI0_SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
   SWD_3STATE();
}

//! Transmit a [mark, 32-bit word, parity] to the target 
//!
//! @param send    - data to send
//!
//! @return BDM_RC_OK => success
//!
__forceinline
static inline void spi_mark_tx32_parity(const uint8_t *data) {
   SWD_ENABLE();
   uint8_t parity = calcParity(data);
   spi_setCTAR1(SPI_CTAR_MASK|SPI_CTAR_FMSZ(9-1));

   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_Xbit)|SWD_PUSHR_TX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA((data[3]<<1)|1);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_TX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data[2]);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_TX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data[1]);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_Xbit)|SWD_PUSHR_TX|                    SPI_PUSHR_TXDATA(data[0]|(parity<<8))|SPI_PUSHR_EOQ_MASK; 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   (void)SPI0_POPR;              // Discard read data
   (void)SPI0_POPR;
   (void)SPI0_POPR;
   (void)SPI0_POPR;
   SPI0_SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
   SWD_3STATE();
}

//! Transmit a [32-bit word] to the target 
//!
//! @param send    - data to send
//!
//! @return BDM_RC_OK => success
//!
__forceinline
static inline void spi_tx32(const uint8_t *data) {
   SWD_ENABLE();
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_TX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data[3]);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_TX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data[2]);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_TX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data[1]);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_TX|                    SPI_PUSHR_TXDATA(data[0])|SPI_PUSHR_EOQ_MASK; 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   SWD_3STATE();
   (void)SPI0_POPR;              // Discard read data
   (void)SPI0_POPR;
   (void)SPI0_POPR;
   (void)SPI0_POPR;
   SPI0_SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
}

//! Receive a 32-bit word + parity from the target 
//!
//! @param receive - data received
//!
//! @return BDM_RC_OK => success
//!
__forceinline
static inline uint8_t spi_rx32_parity(uint8_t *receive) {
   uint16_t dummy;
   spi_setCTAR1(SPI_CTAR_MASK|SPI_CTAR_FMSZ(9-1));
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_RX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_RX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_RX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_Xbit)|SWD_PUSHR_RX|                    SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK; 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   SPI0_SR = SPI_SR_EOQF_MASK;
   receive[3]  = SPI0_POPR;
   receive[2]  = SPI0_POPR;
   receive[1]  = SPI0_POPR;
   dummy       = SPI0_POPR;
   receive[0]  = dummy;
   return ((dummy>>8)!=calcParity(receive))?BDM_RC_ARM_PARITY_ERROR:BDM_RC_OK;
}

#if 0
//! Receive a 32-bit word from the target 
//!
//! @param receive - data received
//!
//! @return BDM_RC_OK => success
//!
__forceinline
static inline void spi_rx32(uint8_t *receive) {
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_RX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_RX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_RX|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_RX|                    SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK; 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   SPI0_SR = SPI_SR_EOQF_MASK;
   receive[3]  = SPI0_POPR;
   receive[2]  = SPI0_POPR;
   receive[1]  = SPI0_POPR;
   receive[0]  = SPI0_POPR;
}
#endif

//! Receive a 4-bit word from the target 
//!
//! @return data received
//!
__forceinline
static inline uint8_t spi_rx4(void) {
   spi_setCTAR1(SPI_CTAR_MASK|SPI_CTAR_FMSZ(4-1));
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_Xbit)|SWD_PUSHR_RX|SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK; 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   SPI0_SR = SPI_SR_EOQF_MASK;
   return SPI0_POPR;
}

#if 1
//! Transmit 8-bit command and receive a 4-bit word from the target 
//!
//! @return data received
//!
__forceinline
static inline uint8_t spi_tx8_rx4(uint8_t command) {
   spi_tx8(command);
   return spi_rx4();
}
#else
//! Transmit 8-bit command and receive a 4-bit word from the target 
//!
//! @return data received
//!
__forceinline
static inline uint8_t spi_tx8_rx4(uint8_t command) {
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_8bit)|SWD_PUSHR_TX|SPI_PUSHR_TXDATA(command); 
   spi_setCTAR1(SPI_CTAR_MASK|SPI_CTAR_FMSZ(4-1));
   SPI0_SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
   SPI0_PUSHR = SPI_PUSHR_CTAS(ctas_Xbit)|SWD_PUSHR_RX|SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK; 
   while ((SPI0_SR & SPI_SR_EOQF_MASK) == 0) {
   }
   (void)SPI0_POPR;              // Discard byte read data
   SPI0_SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
   return SPI0_POPR;             // Save 4-bit data
}
#endif

//! Transmits 8-bits of idle (SWDIO=0)
//!
//! @note 
//!    - ENTRY SWCLK=high \n
//!    - EXIT  SWCLK=unchanged(high), SWDIO=3-state
//!
__forceinline
inline void swd_txIdle8(void) {
   spi_tx8(0);
}

#ifdef SLOWER_METHOD
//! SWD command phase
//!
//! Writes 8-bit command and receives 3-bit response
//! It will retry on WAIT response a limited number of times
//!
//! @param command - 8-bit command to write to SWD (including parity!)
//!
//! @note 
//!    - ENTRY SWCLK=high, SWD_IN=pin \n
//!    - EXIT  SWCLK=high, SWD_IN=pin & SWDIO=3state
//!
//! @note Sequence:\n
//!   8-bit command
//!   1-bit turn-around
//!   3-bit acknowledge
//!   
//! @note A 1-bit turn-around will be appended on error responses
//!
//! @return \n
//!    == \ref BDM_RC_OK              => Success        \n
//!    == \ref BDM_RC_ARM_FAULT_ERROR => FAULT response from target \n
//!    == \ref BDM_RC_ACK_TIMEOUT     => Excessive number of WAIT responses from target \n
//!    == \ref BDM_RC_NO_CONNECTION   => Unexpected/no response from target
//!
uint8_t swd_sendCommandWithWait(uint8_t command) {
   int retry  = 20; // Set up retry count
   uint8_t rxData;
   uint8_t rc = BDM_RC_OK;
   spi_tx8(command);           // Tx command
   do {
      rxData = spi_rx4()>>1;
      if (rxData == SWD_ACK_OK) {
         return BDM_RC_OK; // Success
      }
      if (rxData == SWD_ACK_WAIT) {
         rc = BDM_RC_ACK_TIMEOUT;
         // 1 clock turn-around on WAIT + retry
         spi_mark_tx8(command);      // Turn-around + Tx command
         continue;
      }
      if (rxData == SWD_ACK_FAULT) {
         rc = BDM_RC_ARM_FAULT_ERROR;
      }
      else {
         rc = BDM_RC_NO_CONNECTION;
      }
      // No retry
      break;
   } while(retry-->0);
   return rc;
}
#endif

//! Transmits 32-bit value
//!
//! Sequence as follows:
//!   - 1-clock turn-around 
//!   - 32-bit data value
//!   - 1-bit parity
//!   - 8-bit idle
//!
//! @param data - ptr to 32-bit data to Tx
//!
//! @note 
//!    - ENTRY SWCLK=high \n
//!    - EXIT  SWCLK=high, SWDIO=3state
//! 
static void swd_tx32(const uint8_t *data) {
   spi_mark_tx32_parity(data);
   swd_txIdle8();
}

//! Switches interface to SWD
//!
//! Reference ARM® Debug Interface v5 Architecture Specification
//!           ADIv5.1 Supplement - 6.2.1 JTAG to Serial Wire switching
//!
//! Sequence as follows:
//!  - >=50-bit sequence of 1's
//!  - 16-bit magic number 0xE79E
//!  - >=50-bit sequence of 1's
//!  - 8-bit idle
//!
//! @note 
//!    - ENTRY SWCLK=high                       \n
//!    - EXIT  SWCLK=unchanged, SWDIO=enabled
//! 
//! @note Interface is reset even if already in SWD mode so IDCODE must be read
//!       to enable interface
//!
__forceinline
static inline void swd_JTAGtoSWD(void) {
   uint8_t allOnes[]  = {0xFF,0xFF,0xFF,0xFF,};
   uint8_t magic1[]   = {0x79,0xEF,0xFF,0xFF};
   uint8_t magic2[]   = {0xFF,0xFF,0xFF,0xFE};
   uint8_t trailing[] = {0x00,0xFF,0xFF,0xFF,};
      
   spi_tx32(allOnes);  // 32 1's
   spi_tx32(magic1);   // 20 1's + 0x79E 
   spi_tx32(magic2);   // 0xE + 28 1's 
   spi_tx32(trailing); // 24 1's + 8 0's
}

//! SWD - Try to connect to the target
//!
//! This will do the following:
//! - Switch the interface to SWD mode
//! - Read IDCODE
//!
//! @return \n
//!    == \ref BDM_RC_OK              => Success        \n
//!    == \ref BDM_RC_NO_CONNECTION   => Unexpected/no response from target
//!
uint8_t swd_connect(void) {
   uint8_t buff[4];	
   
   swd_JTAGtoSWD();

   // Target must respond to read IDCODE immediately
   return swd_readReg(SWD_READ_IDCODE, buff);
}

#ifndef SLOWER_METHOD
//! Read ARM-SWD DP & AP register
//!
//! @param command - SWD command byte to select register etc.
//! @param data    - buffer for 32-bit value read
//!
//! @return \n
//!    == \ref BDM_RC_OK               => Success        \n
//!    == \ref BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
//!    == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
//!    == \ref BDM_RC_NO_CONNECTION    => Unexpected/no response from target \n
//!    == \ref BDM_RC_ARM_PARITY_ERROR => Parity error on data read
//!
//! @note Action and Data returned depends on register (some responses are pipelined)\n
//!   SWD_RD_DP_IDCODE - Value from IDCODE reg \n
//!   SWD_RD_DP_STATUS - Value from STATUS reg \n
//!   SWD_RD_DP_RESEND - LAST value read (AP read or DP-RDBUFF), FAULT on sticky error    \n
//!   SWD_RD_DP_RDBUFF - Value from last AP read and clear READOK flag in STRL/STAT, FAULT on sticky error \n
//!   SWD_RD_AP_REGx   - Value from last AP read, clear READOK flag in STRL/STAT and INITIATE next AP read, FAULT on sticky error 
//!
uint8_t swd_readReg(uint8_t command, uint8_t *data) {
   int retry  = 20;            // Set up retry count
   uint8_t ack;
   uint8_t rc;
   ack = spi_tx8_rx4(command)>>1; // Tx command & get ACK (1st attempt)
   while (ack != SWD_ACK_OK) {
      if (ack == SWD_ACK_WAIT) {
         if (retry-- == 0) {
            return BDM_RC_ACK_TIMEOUT;
         }
         // 1 clock turn-around on WAIT + retry
         spi_mark_tx8(command);      // Turn-around + Tx command (retry)
         ack = spi_rx4()>>1;
         continue;
      }
      if (ack == SWD_ACK_FAULT) {
         return BDM_RC_ARM_FAULT_ERROR;
      }
      return BDM_RC_NO_CONNECTION;
   }
   rc = spi_rx32_parity(data);
   swd_txIdle8();
   return rc;
}
#else
//! Read ARM-SWD DP & AP register
//!
//! @param command - SWD command byte to select register etc.
//! @param data    - buffer for 32-bit value read
//!
//! @return \n
//!    == \ref BDM_RC_OK               => Success        \n
//!    == \ref BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
//!    == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
//!    == \ref BDM_RC_NO_CONNECTION    => Unexpected/no response from target \n
//!    == \ref BDM_RC_ARM_PARITY_ERROR => Parity error on data read
//!
//! @note Action and Data returned depends on register (some responses are pipelined)\n
//!   SWD_RD_DP_IDCODE - Value from IDCODE reg \n
//!   SWD_RD_DP_STATUS - Value from STATUS reg \n
//!   SWD_RD_DP_RESEND - LAST value read (AP read or DP-RDBUFF), FAULT on sticky error    \n
//!   SWD_RD_DP_RDBUFF - Value from last AP read and clear READOK flag in STRL/STAT, FAULT on sticky error \n
//!   SWD_RD_AP_REGx   - Value from last AP read, clear READOK flag in STRL/STAT and INITIATE next AP read, FAULT on sticky error 
//!
uint8_t swd_readReg(uint8_t command, uint8_t *data) {
   uint8_t rc = swd_sendCommandWithWait(command);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   rc = spi_rx32_parity(data);
   swd_txIdle8();
   
   return rc;   
}
#endif

#ifndef SLOWER_METHOD
//! Write ARM-SWD DP & AP register
//!
//! @param command - SWD command byte to select register etc.
//! @param data    - buffer containing 32-bit value to write
//!
//! @return \n
//!    == \ref BDM_RC_OK               => Success        \n
//!    == \ref BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
//!    == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
//!    == \ref BDM_RC_NO_CONNECTION    => Unexpected/no response from target
//!
//! @note Action depends on register (some responses are pipelined)\n
//!   SWD_WR_DP_ABORT   - Write value to ABORT register (accepted) \n
//!   SWD_WR_DP_CONTROL - Write value to CONTROL register (may be pending), FAULT on sticky error. \n
//!   SWD_WR_DP_SELECT  - Write value to SELECT register (may be pending), FAULT on sticky error. \n
//!   SWD_WR_AP_REGx    - Write to AP register.  May initiate action e.g. memory access.  Result is pending, FAULT on sticky error.
//!
uint8_t swd_writeReg(uint8_t command, const uint8_t *data) {
   int retry = 20;            // Set up retry count
   uint8_t ack;
   ack = spi_tx8_rx4(command)>>1; // Tx command & get ACK (1st attempt)
   while (ack != SWD_ACK_OK) {
      if (ack == SWD_ACK_WAIT) {
         if (retry-- == 0) {
            return BDM_RC_ACK_TIMEOUT;
         }
         // 1 clock turn-around on WAIT + retry
         spi_mark_tx8(command);      // Turn-around + Tx command (retry)
         ack = spi_rx4()>>1;
         continue;
      }
      if (ack == SWD_ACK_FAULT) {
         return BDM_RC_ARM_FAULT_ERROR;
      }
      return BDM_RC_NO_CONNECTION;
   }
   swd_tx32(data);
   return BDM_RC_OK;
}
#else
//! Write ARM-SWD DP & AP register
//!
//! @param command - SWD command byte to select register etc.
//! @param data    - buffer containing 32-bit value to write
//!
//! @return \n
//!    == \ref BDM_RC_OK               => Success        \n
//!    == \ref BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
//!    == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
//!    == \ref BDM_RC_NO_CONNECTION    => Unexpected/no response from target
//!
//! @note Action depends on register (some responses are pipelined)\n
//!   SWD_WR_DP_ABORT   - Write value to ABORT register (accepted) \n
//!   SWD_WR_DP_CONTROL - Write value to CONTROL register (may be pending), FAULT on sticky error. \n
//!   SWD_WR_DP_SELECT  - Write value to SELECT register (may be pending), FAULT on sticky error. \n
//!   SWD_WR_AP_REGx    - Write to AP register.  May initiate action e.g. memory access.  Result is pending, FAULT on sticky error.
//!
uint8_t swd_writeReg(uint8_t command, const uint8_t *data) {
   uint8_t rc = swd_sendCommandWithWait(command);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   swd_tx32(data);
   return rc;
}
#endif
//! Write register of Access Port
//!
//! @param 16-bit address \n
//!    A[15:8]  => DP-AP-SELECT[31:24] (AP # Select) \n
//!    A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
//!    A[3:2]   => APACC[3:2]          (Register select within bank)
//! @param buff \n
//!   - [1..4]  =>  32-bit register value
//!
//! @return
//!  == \ref BDM_RC_OK => success
//!
//! @note - Access is completed before return
//!
uint8_t swd_writeAPReg(const uint8_t *address, const uint8_t *buff) {
   static const uint8_t writeAP[] = {SWD_WR_AP_REG0,   SWD_WR_AP_REG1,    SWD_WR_AP_REG2,   SWD_WR_AP_REG3};
   uint8_t rc;
   uint8_t regNo = writeAP[(address[1]&0xC)>>2];
   uint8_t selectData[4];
   selectData[0] = address[0];
   selectData[1] = 0;
   selectData[2] = 0;
   selectData[3] = address[1]&0xF0;
   
   // Set up SELECT register for AP access
   rc = swd_writeReg(SWD_WR_DP_SELECT, selectData);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Initiate write to AP register
   rc = swd_writeReg(regNo, buff);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read from READBUFF register to allow stall/status response
   return swd_readReg(SWD_RD_DP_RDBUFF, selectData);
}

//! Read register of Access Port
//!
//! @param 16-bit address \n
//!    A[15:8]  => DP-AP-SELECT[31:24] (AP # Select) \n
//!    A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
//!    A[3:2]   => APACC[3:2]          (Register select within bank)
//! @param buff \n
//!   - [1..4]  =>  32-bit register value
//!
//! @return
//!  == \ref BDM_RC_OK => success
//!
//! @note - Access is completed before return
//!
uint8_t swd_readAPReg(const uint8_t *address, uint8_t *buff) {
   static const uint8_t readAP[]  = {SWD_RD_AP_REG0,   SWD_RD_AP_REG1,    SWD_RD_AP_REG2,   SWD_RD_AP_REG3};
   uint8_t rc;
   uint8_t regNo = readAP[(address[1]&0xC)>>2];
   uint8_t selectData[4];
   selectData[0] = address[0];
   selectData[1] = 0;
   selectData[2] = 0;
   selectData[3] = address[1]&0xF0;

   // Set up SELECT register for AP access
   rc = swd_writeReg(SWD_WR_DP_SELECT, selectData);
   if (rc != BDM_RC_OK) {
     return rc;
   }
   // Initiate read from AP register (dummy data)
   rc = swd_readReg(regNo, buff);
   if (rc != BDM_RC_OK) {
     return rc;	   
   }
   // Read from READBUFF register
   return swd_readReg(SWD_RD_DP_RDBUFF, buff);
}

//! ARM-SWD - clear sticky bits
//!
//! @return error code
//!
uint8_t swd_clearStickyError(void) {
   static const uint8_t swdClearErrors[4] = {0,0,0,SWD_DP_ABORT_CLEAR_STICKY_ERRORS_B3};
   return swd_writeReg(SWD_WR_DP_ABORT, swdClearErrors);
}

//! ARM-SWD - clear sticky bits and abort AP transactions
//!
//! @return error code
//!
uint8_t swd_abortAP(void) {
   static const uint8_t swdClearErrors[4] = 
      {0,0,0,SWD_DP_ABORT_CLEAR_STICKY_ERRORS_B3|SWD_DP_ABORT_ABORT_AP_B3};
   return swd_writeReg(SWD_WR_DP_ABORT, swdClearErrors);
}

#define MDM_AP_CONTROL 0x01000004
#define MDM_AP_STATUS  0x01000000
#define MDM_AP_IDR     0x0100003F

#define MDM_AP_CONTROL_MASS_ERASE_REQUEST (1<<0)
#define MDM_AP_CONTROL_DEBUG_REQUEST      (1<<2)
#define MDM_AP_CONTROL_RESET_REQUEST      (1<<3)
#define MDM_AP_CONTROL_VLLDBGREQ          (1<<5)
#define MDM_AP_CONTROL_VLLDBGACK          (1<<6)
#define MDM_AP_CONTROL_LLS_VLLSx_ACK      (1<<7)

#define MDM_AP_STATUS_FLASH_READY         (1<<1)
#define MDM_AP_STATUS_MASS_ERASE_ENABLE   (1<<5)

#define MDM_AP_CONTROL_HALT_VALUE      (MDM_AP_CONTROL_RESET_REQUEST)
#define MDM_AP_CONTROL_ERASE_VALUE     (MDM_AP_CONTROL_HALT_VALUE|MDM_AP_CONTROL_MASS_ERASE_REQUEST)

#define DP_CONTROL_CSYSPWRUPREG  (1<<30)
#define DP_CONTROL_CDBGPWRUPREG  (1<<28)

#define DP_CONTROL_VALUE (DP_CONTROL_CSYSPWRUPREG|DP_CONTROL_CDBGPWRUPREG)

#define SETTLE_COUNT (1000)  // How long to wait for Power-on bounces

uint8_t massErase(void) {
   // Compressed address for MDM-AP.Status register
   static const uint8_t statusRegAddress[2]   = { (MDM_AP_STATUS>>24)&0xFF, MDM_AP_STATUS&0xFF };
   // Compressed address for MDM-AP.Control register
   static const uint8_t controlRegAddress[2]   = { (MDM_AP_CONTROL>>24)&0xFF, MDM_AP_CONTROL&0xFF };
   static const uint8_t controlValueWrite[4]   = { (MDM_AP_CONTROL_ERASE_VALUE>>24)&0xFF, (MDM_AP_CONTROL_ERASE_VALUE>>16)&0xFF, 
                                                   (MDM_AP_CONTROL_ERASE_VALUE>>8)&0xFF,  (MDM_AP_CONTROL_ERASE_VALUE>>0)&0xFF };
   uint8_t valueRead[4];
   int eraseWait;
   uint8_t rc;

   // Wait for flash ready
   for (eraseWait=0; eraseWait<10000; eraseWait++) {
      rc = swd_readAPReg(statusRegAddress, valueRead);      
      if (rc != BDM_RC_OK) {
         continue;
      }  
      if ((valueRead[3]&MDM_AP_STATUS_FLASH_READY) != 0) {
         break;
      }
   }
   // Device Secured - Flash angrily
   while ((valueRead[3]&MDM_AP_STATUS_MASS_ERASE_ENABLE) == 0) {
      greenLedToggle();
      WAIT_MS(100);
   }

   // Write erase command
   rc = swd_writeAPReg(controlRegAddress, controlValueWrite);      
   if (rc != BDM_RC_OK) {
      return rc;
   }

   WAIT_MS(100);
   // Wait until complete
   for (eraseWait=0; eraseWait<10000; eraseWait++) {
      greenLedOn();
      rc = swd_readAPReg(controlRegAddress, valueRead);      
      if (rc != BDM_RC_OK) {
         continue;
      }
      rc = (((valueRead[3]&MDM_AP_CONTROL_ERASE_VALUE)&0xFF) == (MDM_AP_CONTROL_HALT_VALUE&0xFF))?BDM_RC_OK:BDM_RC_FAIL;
      if (rc == BDM_RC_OK) {
         break;
      }
   }
   return rc;
}

void swd_reset_capture_mass_erase(void) {
   
   static const uint8_t dpControlValueWrite[4] = { (DP_CONTROL_VALUE>>24)&0xFF, (DP_CONTROL_VALUE>>16)&0xFF, 
                                                   (DP_CONTROL_VALUE>>8)&0xFF,  (DP_CONTROL_VALUE>>0)&0xFF };
   // Compressed address for MDM-AP.Control register
   static const uint8_t controlRegAddress[2]   = { (MDM_AP_CONTROL>>24)&0xFF, MDM_AP_CONTROL&0xFF };
   static const uint8_t controlValueWrite[4]   = { (MDM_AP_CONTROL_HALT_VALUE>>24)&0xFF, (MDM_AP_CONTROL_HALT_VALUE>>16)&0xFF, 
                                                   (MDM_AP_CONTROL_HALT_VALUE>>8)&0xFF,  (MDM_AP_CONTROL_HALT_VALUE>>0)&0xFF };
   uint8_t buff[10];
   
   unsigned successCount = 0;
   uint8_t rc;
   unsigned attemptCount = 0;
   
   for (;;) {
      attemptCount++;
      if ((attemptCount&0xFFF)==0) {
         greenLedToggle();
      }
      // Do connect sequence
      rc = swd_connect();
      if (rc != BDM_RC_OK) {
         successCount = 0;
         continue;
      }
      // Power up Debug interface
      rc = swd_writeReg(SWD_WR_DP_CONTROL, dpControlValueWrite);
      if (rc != BDM_RC_OK) {
         successCount = 0;
         continue;
      }
      // Hold processor in reset
      rc = swd_writeAPReg(controlRegAddress, controlValueWrite);      
      if (rc != BDM_RC_OK) {
         successCount = 0;
         continue;
      }
      // Check processor status
      rc = swd_readAPReg(controlRegAddress, buff);      
      if (rc != BDM_RC_OK) {
         successCount = 0;
         continue;
      }
      rc = ((buff[3]&controlValueWrite[3]) == controlValueWrite[3])?BDM_RC_OK:BDM_RC_FAIL;
      if (rc != BDM_RC_OK) {
         successCount = 0;
         continue;
      }
      successCount++;
      // Only consider successful if done SETTLE_COUNT times in a row
      // This prevents mass-erase attempts during power-on bounces etc.
      rc = (successCount>SETTLE_COUNT)?BDM_RC_OK:BDM_RC_FAIL;
      if (rc == BDM_RC_OK) {
         successCount = 0;
         rc = massErase();
      }
      if (rc == BDM_RC_OK) {
         break;
      }
   }
   
   if (rc == BDM_RC_OK) {
      for(;;) {
         greenLedOn();
         WAIT_MS(900);
         greenLedOff();
         WAIT_MS(100);
      }
   }
   else {
      for(;;) {
         greenLedOn();
         WAIT_MS(100);
         greenLedOff();
         WAIT_MS(900);
      }      
   }
}
#endif // HW_CAPABILITY && CAP_SWD_HW

