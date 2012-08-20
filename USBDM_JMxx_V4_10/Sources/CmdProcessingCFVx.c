/*! \file
    \brief USBDM - Coldfire V2, V3 & V4 BDM commands.

    This file processes the commands received over the USB link from the host

   \verbatim
   This software was modified from \e TBLCF software
   This software was modified from \e TBDML software

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
   |    Sep 2009 | Major changes for V2                                               - pgo
   -=======================================================================================
   | 20 Jan 2011 | Removed setBDMBusy() from f_CMD_JTAG_EXECUTE_SEQUENCE{}            - pgo
   |  9 Jun 2010 | Added f_CMD_JTAG_RESET{}                                           - pgo
   |  9 Jun 2010 | Added f_CMD_JTAG_EXECUTE_SEQUENCE{}                                - pgo
   | 25 Sep 2009 | Added f_CMD_CFVX_READ_STATUS_REG{}                                 - pgo
   | 23 Jan 2009 | Many size optimizations made                                       - pgo
   | 13 Jan 2009 | Re-organization for merge with USBDM                               - pgo
   | 13 Jan 2009 | Made command completion check compulsory                           - pgo
   | 12 Jan 2009 | Error handling extended                                            - pgo
   +=======================================================================================
   \endverbatim
*/
#include <string.h>
#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "TargetDefines.h"
#include "BDM.h"
#include "BDM_CF.h"
#include "bdmcfMacros.h"
#include "CmdProcessing.h"
#include "CmdProcessingCFVx.h"
#include "BDMCommon.h"
#include "JTAGSequence.h"

#if (HW_CAPABILITY&(CAP_CFVx_HW|CAP_JTAG_HW|CAP_SWD_HW))
//! Set comm speed to user supplied value
//!
//! @note
//!  commandBuffer\n
//!   - [2..3]  =>  speed in 'ticks'?
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_SPI_SET_SPEED(void) {

U16 freq = *(U16*)(commandBuffer+2); // Get the new speed

   return spi_setSpeed(freq);
}

U8 f_CMD_SPI_GET_SPEED(void) {
   *(U16*)(commandBuffer+1) = cable_status.sync_length;
   returnSize = 3;
   return BDM_RC_OK;
}
#endif

#if (HW_CAPABILITY&(CAP_CFVx_HW|CAP_JTAG_HW))
////! Assert the TA signal
////!
////! @note
////!  commandBuffer                                                         \n
////!    - [2]    => 8-bit time interval in 10us ticks                       \n
////!    - [3..4] => interface level [ignored] see \ref InterfaceLevelMasks_t
////!
////! @return
////!  == \ref BDM_RC_OK => success         \n
////!                                       \n
////!  commandBuffer                        \n
////!    - [1..2] => 0
////!
//U8 f_CMD_CFVx_CONTROL_INTERFACE(void) {
//U8  level  = commandBuffer[4];
//
//   // This may take a while
//   setBDMBusy();
//
//#if (HW_CAPABILITY&CAP_CFVx_HW)
//   TA_LOW();                       // Assert TA
//   WAIT_US(10*commandBuffer[2]);   // Wait the specified time
//   TA_3STATE();                    // Release TA
//#endif
//   
//#if (HW_CAPABILITY & CAP_RST_IO)
//   switch (level&SI_RESET) {
//      case SI_RESET_LOW : // RESET pin=L
//         RESET_LOW();
//         break;
//      default :
//         RESET_3STATE();
//         break;
//   }
//#endif
//   *(U16*)(commandBuffer+1) = 0;   // Dummy return value
//   returnSize = 3;
//   return BDM_RC_OK;
//}
#endif

#if (HW_CAPABILITY&CAP_CFVx_HW)
//! Resets the target processor
//!
//! @note
//!  commandBuffer\n
//!  - [2] = BDM mode, see \ref TargetMode_t\n
//!
//! @note Length of the reset pulse is 50ms,\n
//!       For special mode BKPT is active for 50ms after reset is released
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!

U8 f_CMD_CFVx_RESET(void) {
register U8 mode = commandBuffer[2]&RESET_MODE_MASK;

   // This may take a while
   setBDMBusy();
   switch (commandBuffer[2] & RESET_TYPE_MASK) {
      case RESET_SOFTWARE :
         return BDM_RC_ILLEGAL_PARAMS;
      case RESET_POWER :
         return bdm_cycleTargetVdd(mode);
      case RESET_HARDWARE :
      case RESET_ALL :
      default:
         RESET_LOW();                              // Assert RESET
         if (mode == RESET_SPECIAL)                // Assert BKPT in special mode
            BKPT_LOW();
         WAIT_MS(50 /* ms */);                     // Wait a while
         RESET_3STATE();                           // Release RESET
         WAIT_MS(50 /* ms */);                     // Give the reset pin enough time to rise even with slow RC
         BKPT_HIGH();                              // Release BKPT (if asserted)

         cable_status.reset = NO_RESET_ACTIVITY;   // Clear the reset flag
#if (HW_CAPABILITY&CAP_CFVx_HW)         
         (void)bdmcf_complete_chk_rx();            // Added in revision 0.3
#endif         
         return BDM_RC_OK;
   }
}

//! Resync communication with the target MCU
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_CFVx_RESYNC(void) {

   return bdmcf_resync();   // try to resynchronize
}

//! Start code execution from current PC address
//!
//! @param mode \n
//!    == 0 => run, \n
//!    != 0 => single step
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!
static U8 cfvx_target_go(U8 mode) {
U8 rc;
U8 buff[6];

   (void)bdmcf_tx_msg(BDMCF_CMD_RDMREG);  // Read CSR from target
   rc = bdmcf_rx(2,buff+2);
   if (rc != BDM_RC_OK)
      return rc;

   // Current CSR is in buff[2..5]
   if (mode)
      buff[5] |=  CFVx_CSR_SSM;           // Set the SSM bit (single step)
   else
      buff[5] &= ~CFVx_CSR_SSM;           // Clear the SSM bit (go)

   *((U16 *)buff) = BDMCF_CMD_WDMREG;     // Write the CSR back
   bdmcf_tx(3,buff);

   rc = bdmcf_complete_chk(BDMCF_CMD_GO); // GO & check rc from CSR write!
   if (rc != BDM_RC_OK)
      return rc;

   return bdmcf_complete_chk_rx();
}

//! Step over a single instruction at current PC
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_CFVx_STEP(void) {
   return cfvx_target_go(1);
}

//! Start code execution from current PC address
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_CFVx_GO(void) {
   return cfvx_target_go(0);
}

//! Stop execution of user code by asserting the BKPT line
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!
U8 f_CMD_CFVx_HALT(void) {

   BKPT_LOW();         // Assert BKPT for 700 us
   WAIT_US( 700 );     // This should be enough even for a target running at 2kHz clock
   BKPT_HIGH();

   (void)bdmcf_complete_chk_rx(); // Added in revision 0.3
   // Above is a workaround for a strange problem: CF CPU V2 seems to ignore the first transfer after a halt
   // I do not admit I know why it happens, but the extra NOP command fixes the problem...
   // The problem has nothing to do with the delay: adding up to 400ms of delay between the halt and the read did not fix it
   return BDM_RC_OK;
}

//======================================================================
//======================================================================
//======================================================================

//! Writes CFVx memory
//!
//! @note
//!  commandBuffer\n
//!    - [2]    = element size
//!    - [3]    = # of bytes
//!    - [4..7] = address [MSB ignored]
//!    - [8..N] = data to write
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_CFVx_WRITE_MEM(void) {
U8 rc;
U8 elementSize = commandBuffer[2];
U8 count       = commandBuffer[3];  // # of bytes
U8 *ptr        = commandBuffer+8;   // Start of data

   if (count>0) {
      switch (elementSize) {
         case 1 :
            *(U16*)(commandBuffer+2) = BDMCF_CMD_WRITE8;    // Set up command
            bdmcf_tx(3,commandBuffer+2);                    // Tx command & address
            (void)bdmcf_tx_msg(*ptr);                       // Tx 1st data byte
            ptr++;                                          // Start of remaining data in buffer
            count--;                                        // Count 1st byte
            while(count>0) {
               rc = bdmcf_complete_chk(BDMCF_CMD_FILL8);    // Tx write byte command
               if (rc != BDM_RC_OK)
                  return rc;
               (void)bdmcf_tx_msg(*ptr);                    // Tx the next byte of the data
               ptr++;
               count--;
            }
            break;
         case 2 :
            *(U16*)(commandBuffer+2) = BDMCF_CMD_WRITE16;   // Set up command
            bdmcf_tx(4,commandBuffer+2);                    // Tx command, address & 1st word
            ptr   += 2;                                     // Start of remaining data in buffer
            count >>= 1;                                    // Change to count of remaining words
            count--;
            while(count>0) {
               rc = bdmcf_complete_chk(BDMCF_CMD_FILL16);   // Tx write word command
               if (rc != BDM_RC_OK)
                  return rc;
               (void)bdmcf_tx_msg(*(U16 *)ptr);             // Tx next word of data
               ptr += 2;
               count--;
            }
            break;
         case 4 :
            *(U16*)(commandBuffer+2) = BDMCF_CMD_WRITE32;   // Set up command
            bdmcf_tx(5,commandBuffer+2);                    // Tx command, address & 1st long word
            ptr   += 4;                                     // Start of remaining data in buffer
            count >>= 2;                                    // Change to count of remaining longwords
            count--;
            while(count>0) {
               rc = bdmcf_complete_chk(BDMCF_CMD_FILL32);   // Tx send write dword command
               if (rc != BDM_RC_OK)
                  return rc;
               bdmcf_tx(2,ptr);                             // Tx next long word of data
               ptr += 4;
               count--;
            }
            break;
         default:
            return BDM_RC_ILLEGAL_PARAMS;
      }
   }
   return bdmcf_complete_chk_rx();
}

//! Read CFVx Memory
//!
//! @note
//!  commandBuffer\n
//!    - [2]    = element size
//!    - [3]    = # of bytes
//!    - [4..7] = address [MSB ignored]
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors  \n
//!                                       \n
//!  commandBuffer                        \n
//!    - [1..N] = data read
//!
U8 f_CMD_CFVx_READ_MEM(void) {
U8 rc;
U8 elementSize = commandBuffer[2];
U8 count       = commandBuffer[3];
U8 *ptr        = commandBuffer+1;     // Where first result should go
U8 buff[2];

   if (count>MAX_COMMAND_SIZE-1)
      return BDM_RC_ILLEGAL_PARAMS;  // requested block+status is too long to fit into the buffer

   returnSize  = count+1;

   if (count>0) {
      switch (elementSize) {
         case 1 :
            *(U16*)(commandBuffer+2) = BDMCF_CMD_READ8;     // Set up command
            bdmcf_tx(3,commandBuffer+2);                    // Tx command & address
            do {
               count--;                                     // decrement the number of bytes to read
               if (count>0)
                  rc = bdmcf_rxtx(1,buff,BDMCF_CMD_DUMP8);  // get the result & send in new DUMP command
               else
                  rc = bdmcf_rx(1,buff);                    // read the result (and send NOP)
               if (rc != BDM_RC_OK)
                  return rc;
               *ptr++ = buff[1];                            // the byte is LSB of the received word
            } while (count>0);
            break;
         case 2 :
            *(U16*)(commandBuffer+2) = BDMCF_CMD_READ16;    // Set up command
            bdmcf_tx(3,commandBuffer+2);                    // Tx command & address
            count>>=1;
            do {
               count--;                                     // decrement the number of bytes to read
               if (count>0)
                  rc = bdmcf_rxtx(1,ptr,BDMCF_CMD_DUMP16);  // get the result & send in new DUMP command
               else
                  rc = bdmcf_rx(1,ptr);                     // read the result (and send NOP)
               if (rc != BDM_RC_OK)
                  return rc;
               ptr += 2;
            } while(count>0);
            break;
         case 4 :
            *(U16*)(commandBuffer+2) = BDMCF_CMD_READ32;    // Set up command
            bdmcf_tx(3,commandBuffer+2);                    // Tx command & address
            count>>=2;
            do {
               count--;                                     // decrement the number of bytes to read
               if (count>0)
                  rc = bdmcf_rxtx(2,ptr,BDMCF_CMD_DUMP32);  // get the result & send in new DUMP command
               else
                  rc = bdmcf_rx(2,ptr);                     // read the result (and send NOP)
               if (rc != BDM_RC_OK)
                  return rc;
               ptr += 4;
            } while(count>0);
            break;
         default:
            return BDM_RC_ILLEGAL_PARAMS;
         }
   }
   return BDM_RC_OK;
}

//======================================================================
//======================================================================
//======================================================================

//! Write CFVx address/data register
//!
//! @note
//!  commandBuffer\n
//!    - [2..3] => 4-bit register number [MSB ignored]
//!    - [4..7] => 32-bit register value
//!
//! @return
//!    == \ref BDM_RC_OK => success  \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_CFVx_WRITE_REG(void) {
U8 rc;

   (void)bdmcf_tx_msg(BDMCF_CMD_WAREG+(commandBuffer[3]&0x0F));   // Send the command word

   rc = bdmcf_tx_msg_half_rx(*((U16 *)(commandBuffer+4))); // Send 1st word of register value
   if (rc != BDM_RC_OK)
      return rc;

   (void)bdmcf_tx_msg(*((U16 *)(commandBuffer+6)));

   return bdmcf_complete_chk_rx();
}

//! Read CFVx address/data register
//!
//! @note
//!  commandBuffer\n
//!    - [2..3] => 4-bit register number [MSB ignored]
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors  \n
//!                                       \n
//!  commandBuffer                        \n
//!  - [1..4] => 32-bit register value
//!
U8 f_CMD_CFVx_READ_REG(void) {

   returnSize  = 5;
   (void)bdmcf_tx_msg(BDMCF_CMD_RAREG+(commandBuffer[3]&0x0F));    /* send the command */
   return bdmcf_rx(2,commandBuffer+1);
}

//! Write CFVx debug register;
//!
//! @note
//!  commandBuffer                                          \n
//!    - [2..3] => 5-bit register number [MSB ignored]     \n
//!    - [4..7] => 32-bit register value
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_CFVx_WRITE_DREG(void) {
U8 rc;

   (void)bdmcf_tx_msg(BDMCF_CMD_WDMREG+(commandBuffer[3]&0x1F));  // Send the command word

   rc = bdmcf_tx_msg_half_rx(*((U16 *)(commandBuffer+4))); // Send 1st word of register value
   if (rc != BDM_RC_OK)
      return rc;

   (void)bdmcf_tx_msg(*((U16 *)(commandBuffer+6)));

   return bdmcf_complete_chk_rx();
}

//! Read CFVx debug register;
//!
//! @note
//!  commandBuffer\n
//!    - [2..3] => 5-bit register number [MSB ignored]
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors  \n
//!                                       \n
//!  commandBuffer                        \n
//!   - [1..4] => 32-bit register value
//!
U8 f_CMD_CFVx_READ_DREG(void) {

   returnSize  = 5;
   (void)bdmcf_tx_msg(BDMCF_CMD_RDMREG+(commandBuffer[3]&0x1F));   // send the command
   return bdmcf_rx(2,commandBuffer+1);
}

//! Read CFVx status register;
//! This is an alias for f_CMD_CFVx_READ_DREG(reg=CSR)
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors  \n
//!                                       \n
//!  commandBuffer                        \n
//!   - [1..4] => 32-bit register value
//!
U8 f_CMD_CFVx_READ_STATUS_REG(void) {

   returnSize  = 5;
   (void)bdmcf_tx_msg(BDMCF_CMD_RDMREG+0);   // send the command
   return bdmcf_rx(2,commandBuffer+1);
}


//!  Write CFVx control register;
//!
//! @note
//!  commandBuffer\n
//!    - [2..3] => 16-bit register number
//!    - [4..7] => 32-bit register value
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_CFVx_WRITE_CREG(void) {

   (void)bdmcf_tx_msg(BDMCF_CMD_WCREG);   // send the command

   *(U16*)(commandBuffer) = 0;      // Extend address to 32-bits
   bdmcf_tx(4,commandBuffer);       // Tx address & register value

   return bdmcf_complete_chk_rx();
}

//! Read CFVx control register;
//!
//! @note
//!  commandBuffer\n
//!    - [2..3] => 16-bit register number [MSB ignored]
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors  \n
//!                                       \n
//!  commandBuffer                        \n
//!   - [1..4] => 32-bit register value
//!
U8 f_CMD_CFVx_READ_CREG(void) {

   returnSize  = 5;

   (void)bdmcf_tx_msg(BDMCF_CMD_RCREG);      // send the command
   (void)bdmcf_tx_msg(0);                    // and the register address (padded)
   (void)bdmcf_tx_msg(*((U16 *)(commandBuffer+2)));

   // the 4 bytes of the register contents are received into commandBuffer[1..4]
   return bdmcf_rx(2,commandBuffer+1);
}
#endif

#if (HW_CAPABILITY&CAP_JTAG_HW)
//==========================================================================
// JTAG commands
//==========================================================================

//! Resets the target processor
//!
//! @note
//!  commandBuffer\n
//!  - [2] = BDM mode, see \ref TargetMode_t\n
//!
//! @note Length of the reset pulse is 50ms,\n
//!       For special mode BKPT is active for 50ms after reset is released
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!

U8 f_CMD_JTAG_RESET(void) {
   setBDMBusy();   // May take too long
   RESET_LOW();                              // Assert RESET
   WAIT_MS(50 /* ms */);                     // Wait a while
   RESET_3STATE();                           // Release RESET
   WAIT_MS(50 /* ms */);                     // Give the reset pin enough time to rise even with slow RC

   cable_status.reset = NO_RESET_ACTIVITY;   // Clear the reset flag
   return BDM_RC_OK;
}


//! Transitions the TAP controller to TEST-LOGIC-RESET state
//! Makes no assumptions about initial TAP state
//!
//! @return
//!    == \ref BDM_RC_OK => success \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_JTAG_GOTORESET(void) {

   jtag_transition_reset();
   return BDM_RC_OK;
}

//! Transitions the TAP controller to SHIFT-DR or SHIFT-IR state
//! Assumes TAP is in TEST-LOGIC-RESET or RUN-TEST/IDLE
//!
//! @return
//!  == \ref BDM_RC_OK => success                     \n
//!  != \ref BDM_RC_OK => various errors              \n
//!                                                   \n
//!  commandBuffer                                    \n
//!  - [2] == 0 => TAP controller is left in SHIFT-DR \n
//!        != 0 => TAP controller is left in SHIFT-IR
//!
U8 f_CMD_JTAG_GOTOSHIFT(void) {

   jtag_transition_shift(commandBuffer[2]);
   return BDM_RC_OK;
}

//! Writes given bit stream into data/instruction path of the JTAG
//!
//! @note
//!  commandBuffer\n
//!  - [2]    => TAP controller exit state, see \ref JTAG_ExitActions_t
//!  - [3]    => specifies the number of bits to write [>0]
//!  - [4..N] => block of data to write
//!
//! @note  Data are transmitted starting with LSB of the LAST byte in the supplied buffer [for
//!        easier readability of code which uses JTAG]
//! @note  On entry, expects to find the TAP controller in SHIFT-DR or SHIFT-IR state
//!
//! @return
//!    == \ref BDM_RC_OK => success  \n
//!    != \ref BDM_RC_OK => error
//!
U8 f_CMD_JTAG_WRITE(void) {

   if (commandBuffer[3] == 0)
      return BDM_RC_ILLEGAL_PARAMS;

   jtag_write(commandBuffer[2], commandBuffer[3], commandBuffer+4);
   return BDM_RC_OK;
}

//! Reads bitstream out of JTAG
//!
//! @note
//!  commandBuffer\n
//!  - [2]    => TAP controller exit state, see \ref JTAG_ExitActions_t
//!  - [3]    => specifies the number of bits to read [>0]
//!
//! @note  Data are stored starting with LSB of the LAST byte in the supplied buffer (for
//!        easier readability of code which uses JTAG)
//! @note  On entry, expects to find the TAP in SHIFT-DR or SHIFT-IR state
//!
//! @return
//!    == \ref BDM_RC_OK => success       \n
//!    != \ref BDM_RC_OK => error         \n
//!                                       \n
//!  commandBuffer                        \n
//!  - [1..N] => block of data read
//!
U8 f_CMD_JTAG_READ(void) {

U8 numBytes;

   // Calculate the number of bytes to return
   numBytes = (commandBuffer[3]>>3)+((commandBuffer[3]&0x07)!= 0);

   if ((numBytes == 0) || (numBytes>MAX_COMMAND_SIZE-1))
      return BDM_RC_ILLEGAL_PARAMS;

   jtag_read(commandBuffer[2], commandBuffer[3], commandBuffer+1);
   
   returnSize = numBytes+1;
   return BDM_RC_OK;
}

//! Reads/Writes given bit stream into data/instruction path of the JTAG
//!
//! @note
//!  commandBuffer\n
//!  - [2]    => TAP controller exit state, see \ref JTAG_ExitActions_t
//!  - [3]    => specifies the number of bits to write [>0]
//!  - [4..N] => block of data to write
//!
//! @note  Data are transmitted starting with LSB of the LAST byte in the supplied buffer (for
//!        easier readability of code which uses JTAG)
//! @note  On entry, expects to find the TAP controller in SHIFT-DR or SHIFT-IR state
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors  \n
//!                                       \n
//!  commandBuffer                        \n
//!  - [1..N] => block of data read
//!
U8 f_CMD_JTAG_READ_WRITE(void) {
U8 numBytes;

   // Calculate the number of bytes to return
   numBytes = (commandBuffer[3]>>3)+((commandBuffer[3]&0x07)!= 0);

   if ((numBytes == 0) || (numBytes>MAX_COMMAND_SIZE-4))
      return BDM_RC_ILLEGAL_PARAMS;

   jtag_read_write(commandBuffer[2], commandBuffer[3], commandBuffer+4, commandBuffer+4);
   (void)memcpy(commandBuffer+1, commandBuffer+4, numBytes); // relocate data to correct location

   returnSize = numBytes+1;
   return BDM_RC_OK;
}

#if 0
//! Reads/Writes given bit stream into data/instruction path of the JTAG
//!
//! @note
//!  commandBuffer\n
//!  - [2]    => JTAG Sequence length
//!  - [3..N] => JTAG Sequence to execute
//!
//! @note  On entry, expects to find the TAP controller in RUN-TEST/IDLE state
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors  \n
//!                                       \n
//!  commandBuffer                        \n
//!  - [1..M] => block of data read
//!
U8 f_CMD_JTAG_EXECUTE_SEQUENCE(void) {
U8 rc;
U8 sequenceLength = commandBuffer[2];
   
   rc = processJTAGSequence((const U8*) (commandBuffer+3), (commandBuffer+3+sequenceLength));
   returnSize = commandBuffer[sequenceLength+3];
   (void)memcpy(commandBuffer+1, (commandBuffer+sequenceLength+4), returnSize); // relocate data to correct location

   return rc;
}
#else
//! Reads/Writes given bit stream into data/instruction path of the JTAG
//!
//! @note
//!  commandBuffer\n
//!  - [2]      => JTAG input sequence length (M)
//!  - [3]      => JTAG command sequence length (N)
//!  - [4..N+3] => JTAG Sequence to execute
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors  \n
//!                                       \n
//!  commandBuffer                        \n
//!  - [1..M] => block of data read
//!

U8 f_CMD_JTAG_EXECUTE_SEQUENCE(void) {
U8 rc;
U8 sequenceLength = commandBuffer[3];

    returnSize = commandBuffer[2];
    
    // Unfortunately the sequence & the return buffer share the space available in buffer[]
   if ((sequenceLength+returnSize)>MAX_COMMAND_SIZE-5) {
	   return BDM_RC_ILLEGAL_PARAMS;
   }
   rc = processJTAGSequence((const U8*) (commandBuffer+4), (commandBuffer+4+sequenceLength));
   returnSize = commandBuffer[sequenceLength+4];
   (void)memcpy(commandBuffer+1, (commandBuffer+sequenceLength+5), returnSize); // relocate data to correct location

   return rc;
}
#endif
#endif
