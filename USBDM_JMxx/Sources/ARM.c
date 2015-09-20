/*! \file
    \brief ARM-JTAG routines

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
#include "BDM_CF.h"
#include "CmdProcessing.h"
#include "BDMCommon.h"
#include "SPI.h"
#include "TargetDefines.h"
#include "JTAGSequence.h"
#include "ARM.h"
  
#if TARGET_CAPABILITY & CAP_ARM_JTAG

// ARM JTAG Commands
#define ARM_JTAG_MASTER_IR_LENGTH   (4)     //!< IR length for commands below

// JTAG registers selectors
#define JTAG_ABORT_SEL_COMMAND      (0x08)  //!< JTAG IDCODE Register
#define JTAG_DP_DPACC_SEL_COMMAND   (0x0A)  //!< JTAG-DP DP Access Register (DPACC)
#define JTAG_DP_APACC_SEL_COMMAND   (0x0B)  //!< JTAG-DP AP Access Register (APACC)
#define JTAG_IDCODE_SEL_COMMAND     (0x0E)  //!< JTAG IDCODE Register

#define DP_AP_WRITE                 (0x0)
#define DP_AP_READ                  (0x1)
                                    
#define DP_CTRL_STAT_REG            (0x2) //!< R/W access DP STATUS/CONTROL registers
#define DP_SELECT_REG               (0x4) //!< R/W access AP SELECT register
#define DP_RDBUFF_REG               (0x6) //!< RAX/WI access to RDBUFF register

// Responses from DP/AP access
#define ACK_OK_FAULT                (0x02) //!< Access completed (either OK or FAULT)
#define ACK_WAIT                    (0x01) //!< Access incomplete - try again
                                   
//! How many times to retry a MEM-AP access
#define MAX_ARM_RETRY               (500)

#define ARM_WR_ABORT_B3     (0x01)

#define STAT_WDATAERR_B3    (1<<7)
#define STAT_READOK_B3      (1<<6)
#define STAT_STICKYERR_B3   (1<<5)
#define STAT_STICKYCMP_B3   (1<<4)
#define STAT_STICKYORUN_B3  (1<<1)
#define STAT_ANYERROR_B3    (STAT_STICKYERR_B3|STAT_STICKYCMP_B3|STAT_STICKYORUN_B3)

uint8_t lastJtagIR_Value = -1;

//! Sets up JTAG IR value & transitions to shift-DR
//!
//! @param regNo - register number (Not the JTAG register no!)
//!
void arm_writeIR(uint8_t regNo) {
   // JTAG IR value to use to access the register
   static const uint8_t jtagIR_Values[] = {
	   JTAG_ABORT_SEL_COMMAND,    JTAG_DP_DPACC_SEL_COMMAND, JTAG_DP_DPACC_SEL_COMMAND, JTAG_DP_DPACC_SEL_COMMAND,
	   JTAG_DP_APACC_SEL_COMMAND, JTAG_DP_APACC_SEL_COMMAND, JTAG_DP_APACC_SEL_COMMAND, JTAG_DP_APACC_SEL_COMMAND};
   regNo = jtagIR_Values[regNo];
   if (regNo != lastJtagIR_Value) {
      // Change IR
      lastJtagIR_Value = regNo;
      // Write ABORT/DPACC_SEL/APACC_SEL command to IR, move to JTAG_SHIFT_DR
      jtag_transition_shift(JTAG_SHIFT_IR);
      jtag_write(JTAG_EXIT_SHIFT_DR, ARM_JTAG_MASTER_IR_LENGTH, &lastJtagIR_Value);
   }
   else {
      // Assume IR is already set correctly
      jtag_transition_shift(JTAG_SHIFT_DR);
   }
}

//! Read ARM-JTAG register
//!
//! @param regNo - register number
//! @param data  - buffer for 32-bit value read
//!
//! @return \n
//!    == \ref BDM_RC_OK               => Success        \n
//!    == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
//!
//! @note Assumes TAP in JTAG_IDLE
//! @note Leaves TAP in JTAG_IDLE
//! @note - The read has not completed on return.  It is necessary to do another operation \n
//!         to obtain the read value.
//!
uint8_t arm_readReg(uint8_t regNo, uint8_t *data) {
   // 3-bit code to access register for read
   static const uint8_t apDpRegNo[] = {0, DP_AP_READ|DP_CTRL_STAT_REG, DP_AP_READ|DP_SELECT_REG, DP_AP_READ|DP_RDBUFF_REG,
                                  DP_AP_READ|0, DP_AP_READ|2, DP_AP_READ|4, DP_AP_READ|6};
   uint8_t ack;
   uint16_t retry = MAX_ARM_RETRY;  
   if ((regNo == 0) || (regNo > 7)) {
      return BDM_RC_ILLEGAL_PARAMS;
   }
   // Write DPACC_SEL/APACC_SEL command to IR, move to JTAG_SHIFT_DR
   arm_writeIR(regNo);
   for(;;) {
      // Write read-operation/read status, stay in SHIFT-DR
      jtag_read_write(JTAG_STAY_SHIFT, 3, &apDpRegNo[regNo], &ack);
      if (ack == ACK_OK_FAULT) {
         // Read data, exit to JTAG_IDLE afterwards
         jtag_read(JTAG_EXIT_IDLE, 32, data);
         return BDM_RC_OK;
      }
      if (retry-- == 0) {
         // Read data, exit to JTAG_IDLE afterwards
         jtag_read(JTAG_EXIT_IDLE, 32, data);
         return BDM_RC_ACK_TIMEOUT;
      }
      // Read dummy data, exit & re-enter SHIFT-DR afterwards
      jtag_read(JTAG_EXIT_SHIFT_DR, 32, data);
   }
}

//! Initiates a write to an ARM-JTAG register
//!
//! @param regNo - register number
//! @param data  - buffer containing 32-bit value to write
//!
//! @return \n
//!    == \ref BDM_RC_OK               => Success        \n
//!    == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
//!
//! @note Assumes TAP in JTAG_IDLE
//! @note Leaves TAP in JTAG_IDLE
//! @note - The write has not completed on return.  It is necessary to do another operation 
//!         to confirm the write is complete.
//!
uint8_t arm_writeReg(uint8_t regNo, const uint8_t *data) {
   // 3-bit code to access register for write
   static const uint8_t apDpRegNo[] = {DP_AP_WRITE|0, DP_AP_WRITE|DP_CTRL_STAT_REG, DP_AP_WRITE|DP_SELECT_REG, DP_AP_WRITE|DP_RDBUFF_REG,
		                          DP_AP_WRITE|0, DP_AP_WRITE|2, DP_AP_WRITE|4, DP_AP_WRITE|6};
   uint8_t ack;
   uint16_t retry = MAX_ARM_RETRY;  
   if (regNo > 7) {
      return BDM_RC_ILLEGAL_PARAMS;
   }
   // Write ABORT/DPACC_SEL/APACC_SEL command to IR, move to JTAG_SHIFT_DR
   arm_writeIR(regNo);
   for(;;) {
      // Write write-operation/read status, stay in SHIFT-DR
      jtag_read_write(JTAG_STAY_SHIFT, 3, &apDpRegNo[regNo], &ack);
      // Write data, exit & enter SHIFT-IR afterwards
      // Don't expect ACK for regNo=ABORT
      if ((ack == ACK_OK_FAULT) || (regNo == 0)) {
         // Write data, exit to JTAG_IDLE afterwards
         jtag_write(JTAG_EXIT_IDLE, 32, data);
         return BDM_RC_OK;
      }  
      if (retry-- == 0) {
         // Write data, exit to JTAG_IDLE afterwards
    	 jtag_write(JTAG_EXIT_IDLE, 32, data);
         return BDM_RC_ACK_TIMEOUT;
      }
      // Write data, exit & re-enter SHIFT-DR afterwards
      jtag_write(JTAG_EXIT_SHIFT_DR, 32, data);
   }
}

static const uint8_t dummyData[4] = {0,0,0,0};

//! ARM-JTAG - abort AP transactions
//!
//! @return error code
//!
uint8_t arm_abortAP(void) {
   static const uint8_t abortValue[] = {0x00, 0x00, 0x00, ARM_WR_ABORT_B3};
   return arm_writeReg(ARM_WR_ABORT, abortValue);
}

//! Check & clear sticky bits on STATUS register
//!
//! @return
//!  == \ref BDM_RC_OK => No sticky bits set
//!
//! @note - It is assumed that a read status register command has been executed but 
//!         the results are not yet read.
//!
uint8_t arm_CheckStickyPipelined(void) {
   uint8_t status[4];
   uint8_t rc;
   
   // Read status & check
   rc = arm_readReg(ARM_RD_DP_RDBUFF, status);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   if ((status[3] & STAT_ANYERROR_B3) != 0) {
      // Clear stick errors (don't check status)
      (void)arm_writeReg(ARM_WR_DP_CONTROL, status);
      return BDM_RC_ARM_FAULT_ERROR;
   }
   return BDM_RC_OK;
}

//! Check & clear sticky bits on STATUS register
//!
//! @return
//!  == \ref BDM_RC_OK => No sticky bits set
//!
uint8_t arm_CheckStickyUnpipelined(void) {
   uint8_t rc;
   static const uint8_t dummyData[4] = {0,0,0,0};

   // Initiate read status (discard data)
   rc = arm_readReg(ARM_RD_DP_STATUS, (uint8_t *)dummyData);
   if (rc != BDM_RC_OK) {
	  return rc;
   }
   return arm_CheckStickyPipelined();
}

#if 0
//! Initiates a write to an ARM_WR_AHB_DRW register assuming IR already set
//!
//! @param regNo - register number
//! @param data  - buffer containing 32-bit value to write
//!
//! @return \n
//!    == \ref BDM_RC_OK               => Success        \n
//!    == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
//!
//! @note Assumes TAP in JTAG_IDLE
//! @note Leaves TAP in JTAG_IDLE
//! @note - The write has not completed on return.  It is necessary to do another operation 
//!         to confirm the write is complete.
//!
uint8_t arm_repeatWriteReg(const uint8_t *data) {
   // 3-bit code to access register for write
   static const uint8_t apDpRegNo = DP_AP_WRITE|6;
   uint8_t ack;
   uint16_t retry = MAX_ARM_RETRY;  
   jtag_transition_shift(JTAG_SHIFT_DR);
   for(;;) {
      // Write write-operation/read status, stay in SHIFT-DR
      jtag_read_write(JTAG_STAY_SHIFT, 3, &apDpRegNo, &ack);
      // Write data, exit & enter SHIFT-IR afterwards
      if (ack == ACK_OK_FAULT) {
         // Write data, exit to JTAG_IDLE afterwards
         jtag_write(JTAG_EXIT_IDLE, 32, data);
         return BDM_RC_OK;
      }  
      if (retry-- == 0) {
         // Write data, exit to JTAG_IDLE afterwards
    	 jtag_write(JTAG_EXIT_IDLE, 32, data);
         return BDM_RC_ACK_TIMEOUT;
      }
      // Write data, exit & re-enter SHIFT-DR afterwards
      jtag_write(JTAG_EXIT_SHIFT_DR, 32, data);
   }
}
#endif

//! Initiate read from AP register (sets DP_SELECT & APACC)
//!
//! @param address  - 16-bit abbreviated address in AP space \n
//!    A[15:8]  => DP-AP-SELECT[31:24] (AP Select) \n
//!    A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
//!    A[3:2]   => APACC[3:2]          (Register select within bank)
//!
//! @param data - ptr to input buffer to return values
//!
//! @return
//!  == \ref BDM_RC_OK => success
//!
//! @note Assumes TAP in JTAG_IDLE
//! @note Leaves TAP in JTAG_IDLE
//! @note - The read has not completed on return.  It is necessary to do another operation \n
//!         to obtain the read value.
//!
uint8_t arm_readAPReg(const uint8_t *address, uint8_t *data) {
   uint8_t rc;
   uint8_t selectData[4];
   selectData[0] = address[0];
   selectData[1] = 0;
   selectData[2] = 0;
   selectData[3] = address[1]&0xF0;
   
   // Write AP# & Register bank to SELECT register
   rc = arm_writeReg(ARM_WR_DP_SELECT, selectData);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Initiate read operation (discard data)
   return arm_readReg(ARM_RD_AP_REG0+((address[1]>>2)&0x03), data);
}

//! Write AP register
//!
//! @param 16-bit abbreviated address \n
//!    A[15:8]  => DP-AP-SELECT[31:24] (AP # Select) \n
//!    A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
//!    A[3:2]   => APACC[3:2]          (Register select within bank)
//! @param buff \n
//!   - [1..4]  =>  32-bit register value
//!
//! @return
//!  == \ref BDM_RC_OK => success
//!
//! @note Assumes TAP in JTAG_IDLE
//! @note Leaves TAP in JTAG_IDLE
//! @note - The read has not completed on return.  It is necessary to do another operation 
//!         to obtain the read value.
//!
uint8_t arm_writeAPReg(const uint8_t *address, const uint8_t *data) {   
   uint8_t rc;
   uint8_t selectData[4];
   selectData[0] = address[0];
   selectData[1] = 0;
   selectData[2] = 0;
   selectData[3] = address[1]&0xF0;
   
   // Write AP# & Register bank to SELECT register
   rc = arm_writeReg(ARM_WR_DP_SELECT, selectData);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Do write operation
   return arm_writeReg(ARM_WR_AP_REG0+((address[1]>>2)&0x03), data);
}

#if 0
//! Reads multiple from AP register (sets AP_SELECT & APACC)
//!
//! @param numWords - 8-bit number of words to read from address
//! @param address  - 16-bit abbreviated address in AP space \n
//!    A[15:8]  => DP-AP-SELECT[31:24] (AP Select) \n
//!    A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
//!    A[3:2]   => APACC[3:2]          (Register select within bank)
//!
//! @param data - ptr to input buffer to return values
//!    4*numWords => data-32 read \n
//!    4          => Control/Status register value
//!
//! @return error code
//!
uint8_t arm_readAP(uint8_t numWords, const uint8_t *address, uint8_t *data) {
   uint8_t ack;
   uint8_t increment;
   uint16_t retry = MAX_ARM_RETRY;
   uint8_t reg32RnW = DP_AP_READ|((address[1]&0x0C)>>1);
   uint8_t selectData[4];
   selectData[0] = address[0];
   selectData[1] = 0;
   selectData[2] = 0;
   selectData[3] = address[1]&0xF0;
   
   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   jtag_transition_shift(JTAG_SHIFT_IR);
   jtag_write(JTAG_EXIT_SHIFT_DR, ARM_JTAG_MASTER_IR_LENGTH, &dpAccSelectCommand);

   // Write operation/read status, stay in SHIFT-DR
   jtag_read_write(JTAG_STAY_SHIFT, 3, &writeSelect, &ack);
   // Write data, exit & enter SHIFT-IR afterwards
   jtag_write(JTAG_EXIT_SHIFT_IR, 32, selectData);
   if (ack != ACK_OK_FAULT) {
      return BDM_RC_ACK_TIMEOUT;
   }
   // Write APACC_SEL command to IR, move to JTAG_SHIFT_DR
   jtag_write(JTAG_EXIT_SHIFT_DR, ARM_JTAG_MASTER_IR_LENGTH, &apAccSelectCommand);

   increment = 0; // Don't increment address for 1st read (discard data)
   while (numWords > 0) {
      // Write operation/read status, stay in SHIFT-DR
      jtag_read_write(JTAG_STAY_SHIFT, 3,  &reg32RnW, &ack);
      if (ack != ACK_OK_FAULT) {
    	  // Complete failed transaction
    	  jtag_write(JTAG_EXIT_SHIFT_DR, 32, dummyValue);
	      if (ack != ACK_WAIT) {
	    	  return BDM_RC_NO_CONNECTION;
	      }
		  if (retry-- > 0) {
			 continue;
		  }
		  return BDM_RC_ACK_TIMEOUT;
      }
      if (numWords==1) {
         // Read data, exit & move to SHIFT-IR afterwards for last word
         jtag_read(JTAG_EXIT_SHIFT_IR, 32, data);
      }
      else {
          // Read data, exit & re-enter SHIFT-DR afterwards if not last word
    	  jtag_read(JTAG_EXIT_SHIFT_DR, 32, data);
      }
      data += increment;
      increment = 4;
      numWords--;
      retry = MAX_ARM_RETRY;
   }
   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   jtag_write(JTAG_EXIT_SHIFT_DR, ARM_JTAG_MASTER_IR_LENGTH, &dpAccSelectCommand);

   // Write operation/read status, stay in SHIFT-DR
   jtag_read_write(JTAG_STAY_SHIFT, 3,  &readStatus, &ack);
   // Read last data, exit & re-enter SHIFT-DR
   jtag_read(JTAG_EXIT_SHIFT_DR, 32, data);
   data += 4;
   if (ack != ACK_OK_FAULT) {
	   return BDM_RC_ACK_TIMEOUT;
   }
   // Write operation/read status, stay in SHIFT-DR
   jtag_read_write(JTAG_STAY_SHIFT, 3, &readRdBuff, &ack);
   // Read Control/Status data, exit & re-enter SHIFT-DR
   jtag_read(JTAG_EXIT_IDLE, 32, data);
   data += 4;
   if (ack != ACK_OK_FAULT) {
	   return BDM_RC_ACK_TIMEOUT;
   }
   return BDM_RC_OK;
}

//! Write multiple to AP register (sets AP_SELECT & APACC)
//!
//! @param  numWords - 8-bit number of words to write to address \n
//! @param  address  - 16-bit abbreviated address \n
//!    A[15:8]  => DP-AP-SELECT[31:24] (AP Select) \n
//!    A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
//!    A[3:2]   => APACC[3:2]          (Register select within bank)
//! @param dataOutPtr -  ptr to buffer to output to ARM device \n
//!    4*numWords => data-32 to write
//! @param status - Control/Status register value
//!
//! @return error code
//!
uint8_t arm_writeAP(uint8_t numWords, const uint8_t *address, uint8_t *dataOutPtr, uint8_t *status) {
   uint8_t ack;
   uint16_t retry = MAX_ARM_RETRY;
   uint8_t reg32RnW = DP_AP_WRITE|((address[1]&0x0C)>>1);
   uint8_t selectData[4];
   selectData[0] = address[0];
   selectData[1] = 0;
   selectData[2] = 0;
   selectData[3] = address[1]&0xF0;
	   
   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   jtag_transition_shift(JTAG_SHIFT_IR);
   jtag_write(JTAG_EXIT_SHIFT_DR, ARM_JTAG_MASTER_IR_LENGTH, &dpAccSelectCommand);

   // Write operation/read status, stay in SHIFT-DR
   jtag_read_write(JTAG_STAY_SHIFT, 3, &writeSelect, &ack);
   // Write data, exit & enter SHIFT-IR afterwards
   jtag_write(JTAG_EXIT_SHIFT_IR, 32, selectData);
   if (ack != ACK_OK_FAULT) {
      return BDM_RC_ACK_TIMEOUT;
   }
   // Write APACC_SEL command to IR, move to JTAG_SHIFT_DR
   jtag_write(JTAG_EXIT_SHIFT_DR, ARM_JTAG_MASTER_IR_LENGTH, &apAccSelectCommand);

   while (numWords > 0) {
      // Write operation/read status, stay in SHIFT-DR
      jtag_read_write(JTAG_STAY_SHIFT, 3, &reg32RnW, &ack);
      if (ack != ACK_OK_FAULT) {
    	 // Complete failed transaction
    	 jtag_write(JTAG_EXIT_SHIFT_DR, 32, dummyValue);
         if (ack != ACK_WAIT) {
            return BDM_RC_NO_CONNECTION;
	     }
		 if (retry-- > 0) {
            continue;
		 }
         return BDM_RC_ACK_TIMEOUT;
      }
      if (numWords==1) {
         // Write data, exit & move to SHIFT-IR afterwards for last word
         jtag_write(JTAG_EXIT_SHIFT_IR, 32, dataOutPtr);
      }
      else {
         // Write data, exit & re-enter SHIFT-DR afterwards if not last word
    	 jtag_write(JTAG_EXIT_SHIFT_DR, 32, dataOutPtr);
      }
      dataOutPtr += 4;
      numWords--;
      retry = MAX_ARM_RETRY;
   }
   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   jtag_write(JTAG_EXIT_SHIFT_DR, ARM_JTAG_MASTER_IR_LENGTH, &dpAccSelectCommand);
   
   // Write operation/read status, stay in SHIFT-DR
   jtag_read_write(JTAG_STAY_SHIFT, 3, &readStatus, &ack);
   // Complete transaction
   jtag_write(JTAG_EXIT_SHIFT_DR, 32, dummyValue);
   if (ack != ACK_OK_FAULT) {
      return BDM_RC_ACK_TIMEOUT;
   }
   // Write operation/read status, stay in SHIFT-DR
   jtag_read_write(JTAG_STAY_SHIFT, 3, &readRdBuff, &ack);
   // Read Control/Status data, exit & re-enter SHIFT-DR
   jtag_read(JTAG_EXIT_IDLE, 32, status);
   if (ack != ACK_OK_FAULT) {
	   return BDM_RC_ACK_TIMEOUT;
   }
   return BDM_RC_OK;
}

#endif

uint8_t arm_test(void) {
   return BDM_RC_OK;
}
#endif // HW_CAPABILITY && CAP_SWD_HW

