/*! \file
    \brief USBDM - Coldfire V2, V3 & V4 low level BDM communication.
    
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
   |  1 Aug 2010 | Split JTAG code to new module                               V3.5   - pgo
   | 21 Jun 2010 | Changed JTAG SPI wait method                                       - pgo
   |  2 Jun 2010 | Added delay between polling & reading SPI in jtag_...()            - pgo
   |  2 Jun 2010 | Removed JTAG shifting to TEST-LOGIC_REST in jtag_init()     V3.4   - pgo
   -=======================================================================================
   |    Sep 2009 | Major changes for V2                                               - pgo
   | 18 Mar 2010 | Extended clock high time in bdmcf_txrx_start                       - pgo
   | 28 Jul 2009 | Added additional exit modes for JTAG read/writes                   - pgo
   | 17 Jun 2009 | Changed JTAG ending sequences slightly                             - pgo
   |  7 Jun 2009 | Fixed garbage in last data byte jtag_read when odd size            - pgo
   | 21 May 2009 | Added Speed options for CFVx & JTAG                                - pgo
   | 14 Jan 2009 | Moved timer code & macros to BDMCommon.c/BDMCommon.h               - pgo
   |  1 Jan 2009 | Ported from JB16 to USBDM/JMxx                                     - pgo
   +=======================================================================================
   \endverbatim
*/

#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "BDM.h"
#include "BDM_CF.h"
#include "bdmcfMacros.h"
#include "CmdProcessing.h"
#include "BDMCommon.h"
#if (HW_CAPABILITY&CAP_JTAG_HW)
#include "JTAGSequence.h"
#endif

#pragma DATA_SEG __SHORT_SEG Z_PAGE
// MUST be placed into the direct segment (assumed in ASM code).
U8 bitCount;  //!< Used as a general purpose variable in the bdm_Tx{} & bdm_Rx{}etc.
#pragma DATA_SEG DEFAULT

#if (HW_CAPABILITY&CAP_CFVx_HW) || (HW_CAPABILITY&CAP_JTAG_HW)
//! Structure to relate BDM Communication speed to SPI configuration value
typedef struct {
   U16 freq;         //!< Freq in kHz
   U8  spiValue;     //!< SPI baud value 
   U8  delayCount;   //!< count for start bit time
} SPISpeed;

#ifndef SPI1C1
#define SPI1BR_SPPR_BITNUM SPIBR_SPPR_BITNUM
#define SPI1BR_SPR_BITNUM SPIBR_SPR_BITNUM
#define SPI1BR SPIBR
#define SPI1C1 SPIC1
#define SPI1C1_MSTR_MASK SPIC1_MSTR_MASK
#define SPI1C2 SPIC2
#define SPI1C2_SPIMODE_MASK SPIC2_SPIMODE_MASK
#define SPI1C1_SPE_MASK SPIC1_SPE_MASK
#define SPI1D16 SPID16
#define SPI1S SPIS
#endif

//!  Table relating BDM Communication speed to SPI configuration value
//!  @note Assumes SPI Clock input is 24 MHz
static const SPISpeed SPISpeedValues[ ] = {
   // Freq in kHz           SPI Value                            // Divide by (SPPR+1) * 2**(SPR+1)
    { 12000, ((0<<SPI1BR_SPPR_BITNUM)|(0<<SPI1BR_SPR_BITNUM)),  1}, // = (0+1) * (2**(0+1)) =  /2 => 12   MHz
    {  6000, ((0<<SPI1BR_SPPR_BITNUM)|(1<<SPI1BR_SPR_BITNUM)),  1}, // = (0+1) * (2**(1+1)) =  /4 =>  6   MHz
    {  4000, ((2<<SPI1BR_SPPR_BITNUM)|(0<<SPI1BR_SPR_BITNUM)),  1}, // = (2+1) * (2**(0+1)) =  /6 =>  4   MHz
    {  3000, ((0<<SPI1BR_SPPR_BITNUM)|(2<<SPI1BR_SPR_BITNUM)),  1}, // = (0+1) * (2**(2+1)) =  /8 =>  3   MHz
    {  2000, ((2<<SPI1BR_SPPR_BITNUM)|(1<<SPI1BR_SPR_BITNUM)),  1}, // = (2+1) * (2**(1+1)) = /12 =>  2   MHz
    {  1500, ((0<<SPI1BR_SPPR_BITNUM)|(3<<SPI1BR_SPR_BITNUM)),  2}, // = (0+1) * (2**(3+1)) = /16 =>  1.5 MHz
    {  1000, ((2<<SPI1BR_SPPR_BITNUM)|(2<<SPI1BR_SPR_BITNUM)),  3}, // = (2+1) * (2**(2+1)) = /24 =>  1   MHz
    {   750, ((0<<SPI1BR_SPPR_BITNUM)|(4<<SPI1BR_SPR_BITNUM)),  4}, // = (0+1) * (2**(4+1)) = /32 =>  750 kHz
    {   500, ((2<<SPI1BR_SPPR_BITNUM)|(3<<SPI1BR_SPR_BITNUM)),  6}, // = (2+1) * (2**(3+1)) = /48 =>  500 kHz
    {   250, ((2<<SPI1BR_SPPR_BITNUM)|(4<<SPI1BR_SPR_BITNUM)), 11}, // = (2+1) * (2**(4+1)) = /96 =>  250 KHz
};

#define SPIxC1_OFF   (                SPI1C1_MSTR_MASK)     //!< SPI Masks - Mask to disable SPI
#define SPIxC1_M_ON  (SPI1C1_SPE_MASK|SPI1C1_MSTR_MASK)     //!< SPI Masks - Mask to enable SPI as master
#define SPIxC2_M     (SPI1C2_SPIMODE_MASK)                  //!< SPI Masks - 16-bit mode

U8 SPIBaud = ((2<<SPI1BR_SPPR_BITNUM)|(2<<SPI1BR_SPR_BITNUM)); //!< SPI config value

//! Sets Communication speed for CF V2, 3 & 4 targets
//!
//! @param freq => Frequency on kHz
//!
//! @return  \ref BDM_RC_OK              => Success                 \n
//!          \ref BDM_RC_ILLEGAL_PARAMS  => Speed is not supported
//!
U8 spi_setSpeed(U16 freq) {
int sub;

   bitCount = 40;
   cable_status.sync_length = 1;
   for (sub = 0; sub<sizeof(SPISpeedValues)/sizeof(SPISpeedValues[0]); sub++) {
      if (SPISpeedValues[sub].freq <= freq) {
         SPIBaud = SPISpeedValues[sub].spiValue;
         cable_status.sync_length = freq;
         SPI1BR = SPIBaud;
         bitCount = SPISpeedValues[sub].delayCount;
         return BDM_RC_OK;
      }
   }
   return BDM_RC_ILLEGAL_PARAMS;
}
#endif

#if (HW_CAPABILITY&CAP_CFVx_HW)
U16 bdmcf_txRx16(U16 data);
void bdmcf_tx16(U16 data);

//! Transmits a series of 17 bit messages
//!
//! @param count => number of messages to send
//! @param data  => pointer to message data buffer
//!
//! @note The first byte in the buffer is the MSB of the first message
//!
void bdmcf_tx(U8 count, U8 *data) {
   while(count--) {
      (void)bdmcf_txrx_start();
      bdmcf_tx16(*(U16*)data);
      data+=2;
   }
}

//! Waits for command complete indication & send the next command.
//!
//! @param  next_cmd => next command to send
//!
//! @return  \ref BDM_RC_OK                    => Success                                \n
//!          \ref BDM_RC_CF_BUS_ERROR          => Target returned Bus Error              \n
//!          \ref BDM_RC_CF_ILLEGAL_COMMAND    => Target returned Illegal Command error  \n
//!          \ref BDM_RC_NO_CONNECTION         => No connection / unexpected response
//!
//! @note The next command is sent, the return value is for the \b PREVIOUS command. \n
//!       On error the next command, although sent, would be ignored by the target
//!
U8 bdmcf_complete_chk(U16 next_cmd) {
U8 retryCount = BDMCF_RETRY;
U8 status;
U16 returnData;

   do {
      status     = bdmcf_txrx_start();
      returnData = bdmcf_txRx16(next_cmd);
      if (status == BDMCF_STATUS_OK) 
         return(BDM_RC_OK);
      if (returnData != BDMCF_RES_NOT_READY) 
         break;
   } while ((retryCount--)>0);
   
   switch (returnData) {
      case BDMCF_RES_BUS_ERROR : return BDM_RC_CF_BUS_ERROR;
      case BDMCF_RES_ILLEGAL   : return BDM_RC_CF_ILLEGAL_COMMAND;
      default                  : return BDM_RC_NO_CONNECTION;
   }
}

//! Waits for command completion [Send NOPs while waiting]
//!
//! @return  \ref BDM_RC_OK                    => Success                                \n
//!          \ref BDM_RC_CF_BUS_ERROR          => Target returned Bus Error              \n
//!          \ref BDM_RC_CF_ILLEGAL_COMMAND    => Target returned Illegal Command error  \n
//!          \ref BDM_RC_NO_CONNECTION         => No connection / unexpected response
//!
//! @note Checks bus error as well as not-ready.
//!
U8 bdmcf_complete_chk_rx(void) {
   return bdmcf_complete_chk(BDMCF_CMD_NOP);
}

//! Receives a series of 17 bit messages & Transmits next command
//!
//! @param count       => number of messages to receive
//! @param dataPtr     => pointer to message data buffer
//! @param nextCommand => next command to send
//!
//! @return  \ref BDM_RC_OK                    => Success                                \n
//!          \ref BDM_RC_CF_BUS_ERROR          => Target returned Bus Error              \n
//!          \ref BDM_RC_CF_ILLEGAL_COMMAND    => Target returned Illegal Command error  \n
//!          \ref BDM_RC_NO_CONNECTION         => No connection / unexpected response
//!
//! @note The first byte stored in the buffer is the MSB of the first message
//! @note Transmits the next command while receiving the last message
//!
U8 bdmcf_rxtx(U8 count, U8 *dataPtr, U16 nextCommand) {
U8   retryCount,status;
U16  data;
U16  dataOut = BDMCF_CMD_NOP; // Dummy command to send 

   while(count>0) {
      retryCount = BDMCF_RETRY;
      
      count--;
      if (count == 0)            // Last word to Tx?
         dataOut = nextCommand;  // Yes - send next command
      
      do {
         status = bdmcf_txrx_start();
         data   = bdmcf_txRx16( dataOut );
         if (status == BDMCF_STATUS_OK) 
            break;
         if (data != BDMCF_RES_NOT_READY)
            break;
      } while ((retryCount--)>0);
      
      if (status != BDMCF_STATUS_OK)
         switch (data) {
            case BDMCF_RES_BUS_ERROR : return BDM_RC_CF_BUS_ERROR;
            case BDMCF_RES_ILLEGAL   : return BDM_RC_CF_ILLEGAL_COMMAND;
            default                  : return BDM_RC_NO_CONNECTION;
         }
      *(U16*)dataPtr  = data;
      dataPtr        += 2;
   }
   return(BDM_RC_OK);
}

//! Receives series of 17 bit messages [TxData = BDMCF_CMD_NOP]
//!
//! @param count    => number of messages to receive
//! @param dataPtr  => pointer to message data buffer
//! @note The first byte stored in the buffer is the MSB of the first message
//!
//! @return  \ref BDM_RC_OK                    => Success                                \n
//!          \ref BDM_RC_CF_BUS_ERROR          => Target returned Bus Error              \n
//!          \ref BDM_RC_CF_ILLEGAL_COMMAND    => Target returned Illegal Command error  \n
//!          \ref BDM_RC_NO_CONNECTION         => No connection / unexpected response
//!
U8 bdmcf_rx(U8 count, U8 *dataPtr) {
   return bdmcf_rxtx(count, dataPtr, BDMCF_CMD_NOP);
}

//! Transmits a 17 bit message
//!
//! @param data  => data to Tx
//!
//! @return status bit
//!
U8 bdmcf_tx_msg(U16 data) {
U8 status;

   status = bdmcf_txrx_start();
   bdmcf_tx16(data);
   return(status);
}

//! Transmits a 17 bit message
//!
//! @param dataOut  => data to Tx
//!
//! @return  \ref BDM_RC_OK                    => Success                                \n
//!          \ref BDM_RC_CF_BUS_ERROR          => Target returned Bus Error              \n
//!          \ref BDM_RC_CF_ILLEGAL_COMMAND    => Target returned Illegal Command error  \n
//!          \ref BDM_RC_NO_CONNECTION         => No connection / unexpected response
//!
//! @note To be used for transmitting the second message in a multi-message command which can fail \n 
//!      (e.g. because target is not halted). The correct target response in these cases is Not Ready
//!
U8 bdmcf_tx_msg_half_rx(U16 dataOut) {
U16 dataIn;

   (void)bdmcf_txrx_start();
   dataIn = bdmcf_txRx16(dataOut);
   switch (dataIn) {
      case BDMCF_RES_BUS_ERROR : return BDM_RC_CF_BUS_ERROR;
      case BDMCF_RES_ILLEGAL   : return BDM_RC_CF_ILLEGAL_COMMAND;
      case BDMCF_RES_NOT_READY : return BDM_RC_OK;
      default                  : return BDM_RC_NO_CONNECTION;
   }
}

//! Receives a 17 bit message while sending a NOP
//!
//! @param data  => pointer to where to store the Rx data
//!
//! @return status bit
//!
U8 bdmcf_rx_msg(U16 *data) {
U8 status;
   status  = bdmcf_txrx_start();
   *data   = bdmcf_txRx16(BDMCF_CMD_NOP);
   return(status);
}

//! Resynchronizes communication with the target in case of noise on the CLK line, etc.
//!
//! @return  \ref BDM_RC_OK                    => success                     \n
//!          \ref BDM_RC_NO_CONNECTION         => no connection with target   
//!
U8 bdmcf_resync(void) {
U8  bitCount;
U16 data;

   (void)bdmcf_tx_msg(BDMCF_CMD_NOP);     // Send in 3 NOPs to clear any error
   (void)bdmcf_tx_msg(BDMCF_CMD_NOP);
   (void)bdmcf_rx_msg(&data);
   if ((data&3)==0) 
      return(BDM_RC_NO_CONNECTION);             // The last NOP did not return the expected value (at least one of the two bits should be 1)
   for (bitCount=20; bitCount>0; bitCount--) {  // Now start sending in another nop and watch the result
      if (bdmcf_txrx_start()==0)
         break;   // The first 0 is the status
   }
   if (bitCount==0) // No status bit found in 20 bits
      return(BDM_RC_NO_CONNECTION);
      
   // Transmitted & received the status, finish the nop
	bdmcf_tx16(BDMCF_CMD_NOP);
   
   return(BDM_RC_OK);
}

//!  Sets the CF BDM interface hardware to an idle condition
//!
//! \verbatim
//!  Port configuration
//!  Name             uC pin                   BDM cable
//! ====================================================
//!  DSO/TDO          input & driven           input
//!  ---------------------------------------------------
//!  DSI/TDI          output       0           0
//!  DSCLK            output       0           0
//!  DSCLK_DRV        output       0           -
//!  BKPT*/TMS        input & pulled high      1
//!  ---------------------------------------------------
//!  RSTO             input & driven           input
//!  RSTI             input & pulled high      3-state
//!  TA               input & pulled high      3-state
//!  ---------------------------------------------------
//!  TCLK             input & pulled high      1
//!  TRST             input & pulled high      [see DSCLK]
//!  ---------------------------------------------------
//! \endverbatim
//!
void bdmcf_interfaceIdle(void) {

   SPI1C1        = SPIxC1_OFF; // Disable SPI1
   DATA_PORT     = BDMCF_IDLE;
   DATA_PORT_DDR = BDMCF_IDLE_DDR; 
   RESET_3STATE();  
   TA_3STATE();  
   BKPT_HIGH();
}

//! Initialises the CF BDM interface
//!
void bdmcf_init(void) {

   DSI_OUT_PER     = 1;
   DSCLK_OUT_PER   = 1;
#ifdef DSCLK_DRV_PER
   DSCLK_DRV_PER   = 1;     // Holds DSCLK driver disabled when unused
#endif   
   BKPT_OUT_PER    = 1;     // Holds BKPT_OUT inactive when unused
#ifdef TA_OUT_PER
   TA_OUT_PER      = 1;     // Holds TA_OUT inactive when unused
#endif
   RESET_IN_PER    = 1;     // Needed for input level translation to 5V
   RESET_OUT_PER   = 1;     // Holds RESET_OUT inactive when unused
#ifdef ALLPST_IN_PER
   ALLPST_IN_PER   = 1;     // Needed for input level translation to 5V
#endif
   
   // Set up inputs
   DSO_IN_DDR      = 0;
   RESET_IN_DDR    = 0;
#ifdef ALLPST_IN_DDR
   ALLPST_IN_DDR   = 0;
#endif
   
   DSCLK_DRV_ENABLE();      // DSCLK is driven
#ifdef SPI2C1
   SPI2C1 = SPIxC1_OFF;     // SPI2 is unused (Port pin is used for BKPT*)
#endif
   SPI1C2 = SPIxC2_M;       // Init SPI1 but leave disabled
   SPI1BR = SPIBaud;
   bitCount = 20;
   bdmcf_interfaceIdle();
}

#pragma MESSAGE DISABLE C1404 // Disable warnings about missing return value
#pragma MESSAGE DISABLE C5703 // Disable warnings about unused parameter

//! Transmits & receives 16 bits
//! Assumes start bit has been sent
//!
//! @param data => data to Tx
//!
//! @return 16-bit received
//! 
//! @note Assumes DSCLK is low on entry. \n Leaves DSCLK low on exit
//!
U16 bdmcf_txRx16(U16 data) {
#define SPIS_SPTEF_BIT (5)
#define SPIS_SPRF_BIT  (7)

   asm {               
      //  Entry: U16 parameter in H:X for HCS08
      
      mov    #SPIxC1_M_ON,SPI1C1            // Enable SPI
  L1: brclr  SPIS_SPTEF_BIT,SPI1S,L1        // Wait for Tx buffer free
      sthx   SPI1D16                        // Send the word
  L2: brclr  SPIS_SPRF_BIT,SPI1S,L2         // Wait until Tx/Rx complete

   // The Coldfire BDM interface is not SPI compatible.
   // SPI input sample time is too early (captures previous bit).
   // The following code adjusts for last bit.
      brclr  DSO_IN_BITNUM,DSO_IN_PORT,next  // Capture last bit in Cy
   next:    
      lda    SPI1D16:0                       // Get Rx data (w/o last bit!)
      ldx    SPI1D16:1
      rolx                                   // Rotate last bit into data
      rola
      
      psha                                   
      pulh
      // Exit: U16 return value in H:X for HCS08
   }
}
#pragma MESSAGE DEFAULT C1404 // Restore warnings about missing return value
#pragma MESSAGE DISABLE C5703 // Restore warnings about unused parameter

//! Transmits 16 bits, discards Rx data
//!
//! @param data => data to Tx
//!
void bdmcf_tx16(U16 data) {
	(void)bdmcf_txRx16(data);
}

#pragma MESSAGE DISABLE C1404 // Disable warnings about return expected

//! Transmits 1 bit of logic low value and receives 1 bit
//!
//! @return received data
//!
//! @note Assumes DSCLK is currently low. \n Returns DSCLK low on exit
//!
//! Note - timing changed against calculations shown!!!
//!
U8 bdmcf_txrx_start(void) {
   asm {
      mov   #SPIxC1_OFF,SPI1C1                  // Disable SPI
      mov   #BDMCF_IDLE,DATA_PORT               // [4  pwpp]
      ASM_DSCLK_HIGH                            // [5 rfwpp]  Create rising edge on DSCLK (BSET)
                                                // ---------- 10 + call overhead 
      lda   bitCount                            // [3   rpp]
      dbnza *-0                                 // [4n fppp]
      ASM_DSCLK_LOW                             // [5 rfwpp]  Create falling edge on DSCLK (BCLR)
                                                // ----------  
      lda   bitCount                            // [3   rpp]
      dbnza *-0                                 // [4n fppp]
      clra                                      // [1     p]
      brclr  DSO_IN_BITNUM,DSO_IN_PORT,next     // [5 rpppp]  Capture input data (in Cy)
  next:    
                                                //            Sampled @12+15=27 clks from DSCLK rise
      rola                                      // [1     p]  Save captured data in A
   }
}
#pragma MESSAGE DEFAULT C1404 // Restore warnings about return expected

//!  Target power has been externally cycled. Holds BKPT/BKGD low while Vdd rises
//!
//! @return
//!    \ref BDM_RC_OK                 => Success           \n
//!    \ref BDM_RC_VDD_NOT_PRESENT    => various errors
//      
U8 bdmCF_powerOnReset(void) {
U8 rc = 0;

#if (HW_CAPABILITY&CAP_VDDSENSE)
   bdmcf_interfaceIdle();  // Make sure BDM interface is idle
   BKPT_LOW();

   // Wait for Vdd to rise within 50% of 3V and RESET to return high
   // RESET rise may be delayed by target POR
   WAIT_WITH_TIMEOUT_MS( 250 /* ms */, (bdm_targetVddMeasure()>75)&&
                                     (!bdm_option.useResetSignal)||RESET_IS_HIGH);

   // Let signals settle & CPU to finish reset (with BKGD held low)
   WAIT_US(BKGD_WAIT);

   if (bdm_targetVddMeasure()<=70) // Vpp didn't turn on!
      rc = BDM_RC_VDD_NOT_PRESENT;

   if (bdm_option.useResetSignal && (!RESET_IS_HIGH)) // RESET didn't rise
      rc = BDM_RC_RESET_TIMEOUT_RISE;

   bdmcf_interfaceIdle();  // Make sure BDM interface is idle (BKGD now high)

   // Let signals settle
   WAIT_US(RESET_SETTLE);

   cable_status.reset = RESET_DETECTED;    // Record the fact that reset was asserted

#endif // (HW_CAPABILITY&CAP_VDDSENSE)
   return(rc);
}

//!  Sets the BDM interface to a suspended state
//!
//!  - All signals idle \n
//!  - All voltages off.
//!
void bdmCF_suspend(void) {
   VDD_OFF();
   bdmcf_interfaceIdle();
}

//!  Turns off the BDM interface
//!
//!  Depending upon settings, may leave target power on.
//!
void bdmCF_off( void ) {
   bdmcf_interfaceIdle();
   TRST_3STATE();
}

#endif // (HW_CAPABILITY&CAP_CFVx)
