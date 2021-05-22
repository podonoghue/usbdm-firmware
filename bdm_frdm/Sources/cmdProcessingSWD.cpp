/*! \file
 *  \brief ARM-SWD Command processing
 *
 *  This file processes the commands received over the USB link from the host
 *
 *  \verbatim
 *
 *  USBDM
 *  Copyright (C) 2007  Peter O'Donoghue
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *  \endverbatim
 *
 *  \verbatim
 *  Change History
 *  +=========================================================================================================
 *  | 30 Aug 2016 | Rebuild                                                                             V5.0.0
 *  +=========================================================================================================
 *  \endverbatim
 */
#include <string>
#include <math.h>
#include "configure.h"
#include "commands.h"
//#include "targetDefines.h"
#include "cmdProcessing.h"
#include "cmdProcessingSWD.h"
#include "swd.h"
#include "bdmCommon.h"

namespace Swd {

/**
 *  SWD - Try to connect to the target
 *
 *  This will do the following:
 *  - Check target Vdd
 *  - Switch the interface to SWD mode
 *  - Read IDCODE
 *  - Clear any sticky errors
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode f_CMD_CONNECT(void) {

   USBDM_ErrorCode rc = checkTargetVdd();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   rc = Swd::connect();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   rc = Swd::clearStickyBits();
   return rc;
}

/*  Set communication speed in kHz
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  speed in kHz
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode f_CMD_SET_SPEED(void) {
   uint16_t freq = (commandBuffer[2]<<8)|commandBuffer[3]; // Get the new speed
   return Swd::setSpeed(1000*freq);
}

/*  Get communication speed in kHz
 *
 *  @note
 *   commandBuffer\n
 *    - [1..2]  =>  speed in kHz
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode f_CMD_GET_SPEED(void) {
   uint32_t freq = (uint32_t)round(Swd::getSpeed()/1000.0);
   commandBuffer[1] = (uint8_t)(freq>>8);
   commandBuffer[2] = (uint8_t)(freq);
   returnSize = 3;
   return BDM_RC_OK;
}

/**
 *  Write SWD DP register;
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  3-bit register number [MSB ignored]
 *    - [4..7]  =>  32-bit register value in BIG-ENDIAN order
 *
 *  @return BDM_RC_OK => success, error otherwise
 *
 *  @note Action depends on register (some responses are pipelined) \n
 *    SwdWrite_DP_ABORT   - Write value to ABORT register (accepted) \n
 *    SwdWrite_DP_CONTROL - Write value to CONTROL register (may be pending), FAULT on sticky error. \n
 *    SwdWrite_DP_SELECT  - Write value to SELECT register (may be pending), FAULT on sticky error. \n
 *    SwdWrite_AP_REGx    - Write to AP register.  May initiate action e.g. memory access.  Result is pending, FAULT on sticky error.
 */
USBDM_ErrorCode f_CMD_WRITE_DREG(void) {

   static const SwdWrite writeDP[] = {
         SwdWrite_DP_ABORT, SwdWrite_DP_CONTROL, SwdWrite_DP_SELECT, SwdWrite_DP_INVALID,
         SwdWrite_AP_REG0,  SwdWrite_AP_REG1,    SwdWrite_AP_REG2,   SwdWrite_AP_REG3, };

   return Swd::writeReg(writeDP[commandBuffer[3]&0x07], commandBuffer+4);
}

/**  Read SWD DP register;
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  3-bit register number [MSB ignored]
 *
 *  @return BDM_RC_OK => success          \n
 *                                        \n
 *   commandBuffer                        \n
 *    - [1..4]  =>  32-bit register value in BIG-ENDIAN order
 *
 *  @note Action and Data returned depends on register (some responses are pipelined) \n
 *    SwdRead_DP_IDCODE - Value from IDCODE reg \n
 *    SwdRead_DP_STATUS - Value from STATUS reg \n
 *    SwdRead_DP_RESEND - LAST value read (AP read or DP-RDBUFF), FAULT on sticky error    \n
 *    SwdRead_DP_RDBUFF - Value from last AP read and clear READOK flag in STRL/STAT, FAULT on sticky error \n
 *    SwdRead_AP_REGx   - Value from last AP read, clear READOK flag in STRL/STAT and INITIATE next AP read, FAULT on sticky error
 */
USBDM_ErrorCode f_CMD_READ_DREG(void) {
   static const SwdRead readDP[]  = {
         SwdRead_DP_IDCODE, SwdRead_DP_STATUS, SwdRead_DP_RESEND, SwdRead_DP_RDBUFF,
         SwdRead_AP_REG0,   SwdRead_AP_REG1,   SwdRead_AP_REG2,   SwdRead_AP_REG3, };
   returnSize = 5;
   return Swd::readReg(readDP[commandBuffer[3]&0x07], commandBuffer+1);
}

/**  Write to AP register (sets AP_SELECT & APACC)
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  16-bit register number = A  \n
 *     A[15:8]  => DP-AP-SELECT[31:24] (AP Select) \n
 *     A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
 *     A[3:2]   => APACC[3:2]          (Register select within bank)
 *
 *    - [4..7]  =>  32-bit register value in BIG-ENDIAN order
 *
 *  @return BDM_RC_OK => success, error otherwise
 *
 *  @note - Access is completed before return
 */
USBDM_ErrorCode f_CMD_WRITE_CREG(void) {
   // Write to AP register
   return Swd::writeAPReg(commandBuffer+2, commandBuffer+4);
}

/**  Read from AP register (sets AP_SELECT & APACC)
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  => 16-bit register number = A  \n
 *     A[15:8]  => DP-AP-SELECT[31:24] (AP Select) \n
 *     A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
 *     A[3:2]   => APACC[3:2]          (Register select within bank)
 *
 *    - [1..4]  =>  32-bit register value in BIG-ENDIAN order
 *
 *  @return BDM_RC_OK => success, error otherwise
 *
 *  @note - Access is completed before return
 */
USBDM_ErrorCode f_CMD_READ_CREG(void) {
   // Read from AP register
   returnSize = 5;
   return Swd::readAPReg(commandBuffer+2, commandBuffer+1);
}


/**  Write ARM-SWD Memory
 *
 *  @note
 *   commandBuffer\n
 *    - [2]     =>  size of data elements
 *    - [3]     =>  # of bytes
 *    - [4..7]  =>  Memory address in BIG-ENDIAN order
 *    - [8..N]  =>  Data to write
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode f_CMD_WRITE_MEM(void) {
   return Swd::writeMemory(commandBuffer[2], commandBuffer[3], pack32BE(commandBuffer+4), commandBuffer+8);
}

/**  Read ARM-SWD Memory
 *
 *  @note
 *   commandBuffer\n
 *    - [2]     =>  size of data elements
 *    - [3]     =>  # of bytes
 *    - [4..7]  =>  Memory address in BIG-ENDIAN order
 *
 *  @return
 *  BDM_RC_OK => success, error otherwise \n
 *                                        \n
 *   commandBuffer                        \n
 *    - [1..N]  =>  Data read
 */
USBDM_ErrorCode f_CMD_READ_MEM(void) {
   uint32_t size = commandBuffer[3];
   USBDM_ErrorCode rc = Swd::readMemory(commandBuffer[2], commandBuffer[3], pack32BE(commandBuffer+4), commandBuffer+1);
   if (rc == BDM_RC_OK) {
      // Return size including status byte
      returnSize = size+1;
   }
   return rc;
}

/** Maps register index into magic number for ARM device register */
static const uint8_t regIndexMap[] = {
      ARM_RegR0, ARM_RegR1, ARM_RegR2, ARM_RegR3, ARM_RegR4, ARM_RegR5, ARM_RegR6, ARM_RegR7,
      ARM_RegR8, ARM_RegR9, ARM_RegR10, ARM_RegR11, ARM_RegR12, ARM_RegSP, ARM_RegLR, ARM_RegPC,
      ARM_RegxPSR, ARM_RegMSP,  ARM_RegPSP, ARM_RegMISC,
      ARM_RegFPSCR,
      ARM_RegFPS0+0x00, ARM_RegFPS0+0x01, ARM_RegFPS0+0x02, ARM_RegFPS0+0x03,
      ARM_RegFPS0+0x04, ARM_RegFPS0+0x05, ARM_RegFPS0+0x06, ARM_RegFPS0+0x07,
      ARM_RegFPS0+0x08, ARM_RegFPS0+0x09, ARM_RegFPS0+0x0A, ARM_RegFPS0+0x0B,
      ARM_RegFPS0+0x0C, ARM_RegFPS0+0x0D, ARM_RegFPS0+0x0E, ARM_RegFPS0+0x0F,
      ARM_RegFPS0+0x10, ARM_RegFPS0+0x11, ARM_RegFPS0+0x12, ARM_RegFPS0+0x13,
      ARM_RegFPS0+0x14, ARM_RegFPS0+0x15, ARM_RegFPS0+0x16, ARM_RegFPS0+0x17,
      ARM_RegFPS0+0x18, ARM_RegFPS0+0x19, ARM_RegFPS0+0x1A, ARM_RegFPS0+0x1B,
      ARM_RegFPS0+0x1C, ARM_RegFPS0+0x1D, ARM_RegFPS0+0x1E, ARM_RegFPS0+0x1F,
};

/**  Read all core registers
 *
 *  @note
 *   commandBuffer\n
 *    - [2]  =>  flag - must be zero
 *    - [3]  =>  register index to start at
 *    - [4]  =>  register index to end at
 *
 *  @return BDM_RC_OK => success, error otherwise \n
 *                                                \n
 *   commandBuffer                                \n
 *    - [1..N]  =>  32-bit register values
 */
USBDM_ErrorCode f_CMD_READ_ALL_CORE_REGS(void) {
   if (commandBuffer[2] != 0) {
      // Check flag is zero
      return BDM_RC_ILLEGAL_PARAMS;
   }
   uint8_t  regIndex    = commandBuffer[3];
   uint8_t  endRegister = commandBuffer[4];
   uint8_t* outputPtr   = commandBuffer+1;
   returnSize = 1;
   while (regIndex<=endRegister) {
      uint8_t regValue[4];
      USBDM_ErrorCode rc = Swd::readCoreRegister(regIndexMap[regIndex], regValue);
      if (rc != BDM_RC_OK) {
         return rc;
      }
      // Write to buffer (target format - LITTLE-ENDIAN ARM)
      *outputPtr++ = regValue[3];
      *outputPtr++ = regValue[2];
      *outputPtr++ = regValue[1];
      *outputPtr++ = regValue[0];
      returnSize += 4;
      regIndex++;
   }
   return BDM_RC_OK;
}

/**  Read ARM-SWD core register
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  16-bit register number [MSB ignored]
 *
 *  @return BDM_RC_OK => success, error otherwise \n
 *                                                \n
 *   commandBuffer                                \n
 *    - [1..4]  =>  32-bit register value
 */
USBDM_ErrorCode f_CMD_READ_REG(void) {
   returnSize = 5;
   return Swd::readCoreRegister(commandBuffer[3], commandBuffer+1);
}

/**  Write ARM-SWD core register
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  16-bit register number [MSB ignored]
 *    - [4..7]  =>  32-bit register value
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode f_CMD_WRITE_REG(void) {
   return Swd::writeCoreReg(commandBuffer[3], commandBuffer+4);
}

/**  ARM-SWD -  Step over 1 instruction
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode f_CMD_TARGET_STEP(void) {
   // Preserve DHCSR_C_MASKINTS value
   return Swd::modifyDHCSR(DHCSR_C_MASKINTS, DHCSR_C_STEP|DHCSR_C_DEBUGEN);
}

/**  ARM-SWD -  Start code execution
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode f_CMD_TARGET_GO(void) {
   return modifyDHCSR(DHCSR_C_MASKINTS, DHCSR_C_DEBUGEN);
}

/* ARM-SWD -  Stop the target
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode f_CMD_TARGET_HALT(void) {
   return modifyDHCSR(DHCSR_C_MASKINTS, DHCSR_C_HALT|DHCSR_C_DEBUGEN);
}

}; // End namespace Swd
