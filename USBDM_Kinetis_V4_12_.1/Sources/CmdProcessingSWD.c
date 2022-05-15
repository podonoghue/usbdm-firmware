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
   Change History
   +=========================================================================================================
   | 27 Jul 2013 | Now returns BDM_RC_TARGET_BUSY on register reads while executing         V4.10.6.230 - pgo
   | 27 Jul 2013 | Added f_CMD_SWD_READ_ALL_CORE_REGS()                                     V4.10.6     - pgo
   | 22 Oct 2012 | Added modifyDHCSR() and associated changes                               V4.9.5
   | 30 Aug 2012 | ARM-JTAG & ARM-SWD Changes                                               V4.9.5
   +=========================================================================================================
 *  \endverbatim
 */
#include <string.h>
#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "TargetDefines.h"
#include "BDM.h"
//#include "BDMMacros.h"
#include "BDMCommon.h"
#include "CmdProcessing.h"
#include "CmdProcessingSWD.h"
#include "SWD.h"

#if TARGET_CAPABILITY & CAP_ARM_SWD

// DP_SELECT register value to access AHB_AP Bank #0 for memory read/write
static const uint8_t ARM_AHB_AP_BANK0[4] = {AHB_AP_NUM,  0,  0,  0};

// Initial value of AHB_SP_CSW register (msb)
static       uint8_t ahb_ap_csw_defaultValue_B0 = 0;

// Maps size (1,2,4 byes) to CSW control value (size+increment)
static const uint8_t cswValues[] = {
		0,
		0x40|AHB_AP_CSW_SIZE_BYTE|AHB_AP_CSW_INC_SINGLE,
		0x40|AHB_AP_CSW_SIZE_HALFWORD|AHB_AP_CSW_INC_SINGLE,
		0,
		0x40|AHB_AP_CSW_SIZE_WORD|AHB_AP_CSW_INC_SINGLE,
};
/**
 *  SWD - Try to connect to the target
 *
 *  This will do the following:
 *  - Switch the interface to SWD mode
 *  - Read IDCODE
 *  - Clear any sticky errors
 *
 *  @return
 *     == \ref BDM_RC_OK => success        \n
 *     != \ref BDM_RC_OK => error
 */
uint8_t f_CMD_SWD_CONNECT(void) {
   uint8_t rc;

   ahb_ap_csw_defaultValue_B0 = 0;

   rc = swd_connect();

   (void)swd_clearStickyError();
   return rc;
}

/**
 *  Write SWD DP register;
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  3-bit register number [MSB ignored]
 *    - [4..7]  =>  32-bit register value in BIG-ENDIAN order
 *
 *  @return
 *   == \ref BDM_RC_OK => success
 *
 *  @note Action depends on register (some responses are pipelined) \n
 *    SWD_WR_DP_ABORT   - Write value to ABORT register (accepted) \n
 *    SWD_WR_DP_CONTROL - Write value to CONTROL register (may be pending), FAULT on sticky error. \n
 *    SWD_WR_DP_SELECT  - Write value to SELECT register (may be pending), FAULT on sticky error. \n
 *    SWD_WR_AP_REGx    - Write to AP register.  May initiate action e.g. memory access.  Result is pending, FAULT on sticky error.
 */
uint8_t f_CMD_SWD_WRITE_DREG(void) {
   static const uint8_t writeDP[] = {
      SWD_WR_DP_ABORT, SWD_WR_DP_CONTROL, SWD_WR_DP_SELECT, 0,
      SWD_WR_AP_REG0,  SWD_WR_AP_REG1,    SWD_WR_AP_REG2,   SWD_WR_AP_REG3, };

   return swd_writeReg(writeDP[commandBuffer[3]&0x07], commandBuffer+4);
}

/**  Read SWD DP register;
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  3-bit register number [MSB ignored]
 *
 *  @return
 *   == \ref BDM_RC_OK => success         \n
 *                                        \n
 *   commandBuffer                        \n
 *    - [1..4]  =>  32-bit register value in BIG-ENDIAN order
 *
 *  @note Action and Data returned depends on register (some responses are pipelined) \n
 *    SWD_RD_DP_IDCODE - Value from IDCODE reg \n
 *    SWD_RD_DP_STATUS - Value from STATUS reg \n
 *    SWD_RD_DP_RESEND - LAST value read (AP read or DP-RDBUFF), FAULT on sticky error    \n
 *    SWD_RD_DP_RDBUFF - Value from last AP read and clear READOK flag in STRL/STAT, FAULT on sticky error \n
 *    SWD_RD_AP_REGx   - Value from last AP read, clear READOK flag in STRL/STAT and INITIATE next AP read, FAULT on sticky error
 */
uint8_t f_CMD_SWD_READ_DREG(void) {
   static const uint8_t readDP[]  = {
      SWD_RD_DP_IDCODE, SWD_RD_DP_STATUS, SWD_RD_DP_RESEND, SWD_RD_DP_RDBUFF,
	  SWD_RD_AP_REG0,   SWD_RD_AP_REG1,   SWD_RD_AP_REG2,   SWD_RD_AP_REG3, };
   returnSize = 5;
   return swd_readReg(readDP[commandBuffer[3]&0x07], commandBuffer+1);
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
 *  @return
 *   == \ref BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
uint8_t f_CMD_SWD_WRITE_CREG(void) {
   // Write to AP register
   return swd_writeAPReg(commandBuffer+2, commandBuffer+4);
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
 *  @return
 *   == \ref BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
uint8_t f_CMD_SWD_READ_CREG(void) {
   // Read from AP register
   returnSize = 5;
   return swd_readAPReg(commandBuffer+2, commandBuffer+1);
}

//! Write 32-bit value to ARM-SWD Memory
//!
//! @param address 32-bit memory address
//! @param data    32-bit data value
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors
//!
uint8_t swd_writeMemoryWord(const uint8_t *address, const uint8_t *data) {
	uint8_t  rc;
	uint8_t  temp[4];
   /* Steps
	*  - Set up to access AHB-AP register bank 0 (CSW,TAR,DRW)
	*  - Write AP-CSW value (auto-increment etc)
	*  - Write AP-TAR value (target memory address)
	*  - Write value to DRW (data value to target memory)
	*/
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = swd_writeReg(SWD_WR_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
	  return rc;
   }
   // Write CSW (word access etc)
   temp[0] = ahb_ap_csw_defaultValue_B0;
   temp[1] = 0;
   temp[2] = 0;
   temp[3] = 0x40|AHB_AP_CSW_SIZE_WORD;
   rc = swd_writeReg(SWD_WR_AHB_CSW, temp);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = swd_writeReg(SWD_WR_AHB_TAR, address);
   if (rc != BDM_RC_OK) {
	  return rc;
   }
   // Write data value
   rc = swd_writeReg(SWD_WR_AHB_DRW, data);
   if (rc != BDM_RC_OK) {
	  return rc;
   }
   // Dummy read to get status
   return swd_readReg(SWD_RD_DP_RDBUFF, temp);
}

//! Read 32-bit value from ARM-SWD Memory
//!
//! @param address 32-bit memory address
//! @param data    32-bit data value from last read!
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors
//!
uint8_t swd_readMemoryWord(const uint8_t *address, uint8_t *data) {
uint8_t  rc;
uint8_t  temp[4];

   /* Steps
    *  - Set up to DP_SELECT to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (starting target memory address)
    *  - Initiate read by reading from DRW (dummy value)
    *  - Read data value from DP-READBUFF
    */
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = swd_writeReg(SWD_WR_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write memory access control to CSW
   temp[0] = ahb_ap_csw_defaultValue_B0;
   temp[1] = 0;
   temp[2] = 0;
   temp[3] = 0x40|AHB_AP_CSW_SIZE_WORD;
   rc = swd_writeReg(SWD_WR_AHB_CSW, temp);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = swd_writeReg(SWD_WR_AHB_TAR, address);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Initial read of DRW (dummy data)
   rc = swd_readReg(SWD_RD_AHB_DRW, temp);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read memory data
   return swd_readReg(SWD_RD_DP_RDBUFF, data);
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
 *  @return \n
 *   == \ref BDM_RC_OK => success         \n
 *   != \ref BDM_RC_OK => various errors
 */
uint8_t f_CMD_SWD_WRITE_MEM(void) {
	uint8_t  elementSize = commandBuffer[2];  // Size of the data writes
	uint8_t  count       = commandBuffer[3];  // # of bytes
	uint8_t  addrLSB     = commandBuffer[7];  // Address in target memory
	uint8_t  *data_ptr   = commandBuffer+8;   // Where the data is
	uint8_t  rc;
	uint8_t  temp[4];

   /* Steps
	*  - Set up to access AHB-AP register bank 0 (CSW,TAR,DRW)
	*  - Write AP-CSW value (auto-increment etc)
	*  - Write AP-TAR value (target memory address)
	*  - Loop
	*    - Pack data
	*    - Write value to DRW (data value to target memory)
	*/
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = swd_writeReg(SWD_WR_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
	  return rc;
   }
   if (ahb_ap_csw_defaultValue_B0 == 0) {
      // Read initial AHB-AP.csw register value as device dependent
      // Do posted read - dummy data returned
      rc = swd_readReg(SWD_RD_AHB_CSW, temp);
      if (rc != BDM_RC_OK) {
         return rc;
      }
      // Get actual data
      rc = swd_readReg(SWD_RD_DP_RDBUFF, temp);
      if (rc != BDM_RC_OK) {
         return rc;
      }
      ahb_ap_csw_defaultValue_B0 = temp[0];
    }
   // Write CSW (auto-increment etc)
   temp[0] = ahb_ap_csw_defaultValue_B0;
   temp[1] = 0;
   temp[2] = 0;
   temp[3] = cswValues[elementSize];
   rc = swd_writeReg(SWD_WR_AHB_CSW, temp);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = swd_writeReg(SWD_WR_AHB_TAR, commandBuffer+4);
   if (rc != BDM_RC_OK) {
	  return rc;
   }
   switch (elementSize) {
   case MS_Byte:
	  while (count > 0) {
		 switch (addrLSB&0x3) {
		 case 0: temp[3] = *data_ptr++; break;
		 case 1: temp[2] = *data_ptr++; break;
		 case 2: temp[1] = *data_ptr++; break;
		 case 3: temp[0] = *data_ptr++; break;
		 }
		 rc = swd_writeReg(SWD_WR_AHB_DRW, temp);
		 if (rc != BDM_RC_OK) {
		    return rc;
		 }
		 addrLSB++;
		 count--;
	  }
      break;
   case MS_Word:
      count >>= 1;
      while (count > 0) {
         switch (addrLSB&0x2) {
         case 0:  temp[3] = *data_ptr++;
    	          temp[2] = *data_ptr++; break;
         case 2:  temp[1] = *data_ptr++;
    	          temp[0] = *data_ptr++; break;
         }
    	 rc = swd_writeReg(SWD_WR_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
      	    return rc;
         }
         addrLSB  += 2;
         count--;
      }
	  break;
   case MS_Long:
      count >>= 2;
      while (count-- > 0) {
     	 temp[3] = *data_ptr++;
    	 temp[2] = *data_ptr++;
    	 temp[1] = *data_ptr++;
    	 temp[0] = *data_ptr++;
    	 rc = swd_writeReg(SWD_WR_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
      	    return rc;
         }
      }
 	  break;
   }
   // Dummy read to obtain status from last write
   return swd_readReg(SWD_RD_DP_RDBUFF, temp);
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
 *   == \ref BDM_RC_OK => success         \n
 *   != \ref BDM_RC_OK => various errors  \n
 *                                        \n
 *   commandBuffer                        \n
 *    - [1..N]  =>  Data read
 */
uint8_t f_CMD_SWD_READ_MEM(void) {
uint8_t  elementSize = commandBuffer[2];          // Size of the data writes
uint8_t  count       = commandBuffer[3];          // # of data bytes
uint8_t  addrLSB     = commandBuffer[7];          // LSB of Address in target memory
uint8_t *data_ptr    = commandBuffer+1;           // Where in buffer to write the data
uint8_t  rc;
uint8_t  temp[4];

   /* Steps
    *  - Set up to DP_SELECT to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (starting target memory address)
    *  - Loop
    *    - Read value from DRW (data value from target memory)
    *      Note: 1st value read from DRW is discarded
    *      Note: Last value is read from DP-READBUFF
    *    - Copy to buffer adjusting byte order
    */
   if (count>MAX_COMMAND_SIZE-1) {
      return BDM_RC_ILLEGAL_PARAMS;  // requested block+status is too long to fit into the buffer
   }
#ifdef HACK
   {
   uint32_t address = (commandBuffer[4]<<24)+(commandBuffer[5]<<16)+(commandBuffer[6]<<8)+commandBuffer[7];
   memcpy(data_ptr, (void*)address, count);
   returnSize = count+1;
   return BDM_RC_OK;
   }
#else
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = swd_writeReg(SWD_WR_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   if (ahb_ap_csw_defaultValue_B0 == 0) {
      // Read initial AHB-AP.csw register value as device dependent
      // Do posted read - dummy data returned
   	  rc = swd_readReg(SWD_RD_AHB_CSW, temp);
      if (rc != BDM_RC_OK) {
   	     return rc;
      }
      // Get actual data
      rc = swd_readReg(SWD_RD_DP_RDBUFF, temp);
      if (rc != BDM_RC_OK) {
   	     return rc;
      }
      ahb_ap_csw_defaultValue_B0 = temp[0];
   }
   // Write CSW (auto-increment etc)
   temp[0] = ahb_ap_csw_defaultValue_B0;
   temp[1] = 0;
   temp[2] = 0;
   temp[3] = cswValues[elementSize];
   rc = swd_writeReg(SWD_WR_AHB_CSW, temp);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = swd_writeReg(SWD_WR_AHB_TAR, commandBuffer+4);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Return size including status byte
   returnSize = count+1;

   // Initial read of DRW (dummy data)
   rc = swd_readReg(SWD_RD_AHB_DRW, temp);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   switch (elementSize) {
   case MS_Byte:
      do {
         count--;
         if (count == 0) {
            // Read data from RDBUFF for final read
            rc = swd_readReg(SWD_RD_DP_RDBUFF, temp);
         }
         else {
            // Start next read and collect data from last read
            rc = swd_readReg(SWD_RD_AHB_DRW, temp);
         }
         if (rc != BDM_RC_OK) {
            return rc;
         }
         // Save data
         switch (addrLSB&0x3) {
         case 0: *data_ptr++ = temp[3];  break;
         case 1: *data_ptr++ = temp[2];  break;
         case 2: *data_ptr++ = temp[1];  break;
         case 3: *data_ptr++ = temp[0];  break;
         }
         addrLSB++;
      } while (count > 0);
      break;
   case MS_Word:
      count >>= 1;
	  do {
         count--;
		 if (count == 0) {
			// Read data from RDBUFF for final read
			rc = swd_readReg(SWD_RD_DP_RDBUFF, temp);
		 }
		 else {
			// Start next read and collect data from last read
			rc = swd_readReg(SWD_RD_AHB_DRW, temp);
		 }
		 if (rc != BDM_RC_OK) {
			return rc;
		 }
         // Save data
		 switch (addrLSB&0x2) {
		 case 0: *data_ptr++ = temp[3];
		         *data_ptr++ = temp[2];  break;
		 case 2: *data_ptr++ = temp[1];
		         *data_ptr++ = temp[0];  break;
		 }
		 addrLSB+=2;
	  } while (count > 0);
	  break;
   case MS_Long:
	  count >>= 2;
	  do {
		 count--;
		 if (count == 0) {
			// Read data from RDBUFF for final read
			rc = swd_readReg(SWD_RD_DP_RDBUFF, temp);
		 }
		 else {
			// Start next read and collect data from last read
			rc = swd_readReg(SWD_RD_AHB_DRW, temp);
		 }
		 if (rc != BDM_RC_OK) {
			return rc;
		 }
         // Save data
		 *data_ptr++ = temp[3];
		 *data_ptr++ = temp[2];
		 *data_ptr++ = temp[1];
		 *data_ptr++ = temp[0];
//		 addrLSB+=4;
	  } while (count > 0);
 	  break;
   }
   return rc;
#endif
}

// Memory addresses of debug/core registers
static const uint8_t DHCSR_ADDR[] = {0xE0, 0x00, 0xED, 0xF0}; // RW Debug Halting Control and Status Register
static const uint8_t DCRSR_ADDR[] = {0xE0, 0x00, 0xED, 0xF4}; // WO Debug Core Selector Register
static const uint8_t DCRDR_ADDR[] = {0xE0, 0x00, 0xED, 0xF8}; // RW Debug Core Data Register

#define DCRSR_WRITE_B1         (1<<(16-16))
#define DCRSR_READ_B1          (0<<(16-16))
#define DCRSR_REGMASK_B0       (0x7F)

#define DHCSR_DBGKEY_B0       (0xA0<<(24-24))
#define DHCSR_DBGKEY_B1       (0x5F<<(16-16))
#define DHCSR_S_RESET_ST_B0   (1<<(25-24)
#define DHCSR_S_RETIRE_ST_B0  (1<<(24-24))
#define DHCSR_S_LOCKUP_B1     (1<<(19-16))
#define DHCSR_S_SLEEP_B1      (1<<(18-16))
#define DHCSR_S_HALT_B1       (1<<(17-16))
#define DHCSR_S_REGRDY_B1     (1<<(16-16))
#define DHCSR_C_SNAPSTALL_B3  (1<<5)
#define DHCSR_C_MASKINTS_B3   (1<<3)
#define DHCSR_C_STEP_B3       (1<<2)
#define DHCSR_C_HALT_B3       (1<<1)
#define DHCSR_C_DEBUGEN_B3    (1<<0)

//! Initiates core register operation (read/write) and
//! waits for completion
//!
//! @param DCRSRvalue - value to write to DCSRD register to control operation
//!
//! @note DCRSRvalue is used as scratch buffer so must be ram
//!
static uint8_t swd_coreRegisterOperation(uint8_t *DCRSRvalue) {
   uint8_t retryCount = 40U;
   uint8_t rc;

   // Write operation+regNo
   rc = swd_writeMemoryWord(DCRSR_ADDR, DCRSRvalue);
   if (rc != BDM_RC_OK) {
	  return rc;
   }
   // Wait for transfer complete
   do {
	  if (retryCount-- == 0) {
	     // Assume target busy
		 return BDM_RC_ARM_ACCESS_ERROR;
	  }
	  // Check complete (use DCRSRvalue as scratch)
	  rc = swd_readMemoryWord(DHCSR_ADDR, DCRSRvalue);
	  if (rc != BDM_RC_OK) {
		 return rc;
	  }
	  if ((DCRSRvalue[3] & DHCSR_C_HALT_B3) == 0) {
        // Target must be in DEBUG mode
       return BDM_RC_TARGET_BUSY;
	  }
   } while ((DCRSRvalue[1] & DHCSR_S_REGRDY_B1) == 0);
   return BDM_RC_OK;
}

/*!
 *  Read target register
 *
 *  @param regNo 	  Number of register to read
 *  @param outptr   Where to place data read (in big-endian order)
 *
 *  @return error code
 */
static uint8_t readCoreRegister(uint8_t regNo, uint8_t *outptr) {
   uint8_t rc;
   // Set up command
   uint8_t command[4] = {0, DCRSR_READ_B1, 0, 0};
   command[3] = regNo;
   // Execute register transfer command
   rc = swd_coreRegisterOperation(command);
   if (rc != BDM_RC_OK) {
	  return rc;
   }
   // Read register value from DCRDR holding register (Big-endian) (command is used as buffer)
   return swd_readMemoryWord(DCRDR_ADDR, outptr);
}

#if (HW_CAPABILITY&CAP_CORE_REGS)
// Insufficient memory on some chips!

// Maps register index into magic number for ARM device register number
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
 *  @return
 *   == \ref BDM_RC_OK => success         \n
 *                                        \n
 *   commandBuffer                        \n
 *    - [1..N]  =>  32-bit register values
 */
uint8_t f_CMD_SWD_READ_ALL_CORE_REGS(void) {
   uint8_t  rc;
   uint8_t  regIndex    = commandBuffer[3];
   uint8_t  endRegister = commandBuffer[4];
   uint8_t* outputPtr   = commandBuffer+1;
   uint8_t  command[4];
   returnSize = 1;
   if (commandBuffer[2] != 0) {
      // Check flag is zero
      return BDM_RC_ILLEGAL_PARAMS;
   }
   while (regIndex<=endRegister) {
      rc = readCoreRegister(regIndexMap[regIndex], command);
      if (rc != BDM_RC_OK) {
         return rc;
      }
      // Write to buffer (target format - Little-endian ARM)
      *outputPtr++ = command[3];
      *outputPtr++ = command[2];
      *outputPtr++ = command[1];
      *outputPtr++ = command[0];
      returnSize += 4;
      regIndex++;
   }
   return BDM_RC_OK;
}
#endif

/**  Read ARM-SWD core register
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  16-bit register number [MSB ignored]
 *
 *  @return
 *   == \ref BDM_RC_OK => success         \n
 *                                        \n
 *   commandBuffer                        \n
 *    - [1..4]  =>  32-bit register value
 */
uint8_t f_CMD_SWD_READ_REG(void) {
   returnSize = 5;
   return readCoreRegister(commandBuffer[3], commandBuffer+1);
}

/**  Write ARM-SWD core register
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  16-bit register number [MSB ignored]
 *    - [4..7]  =>  32-bit register value
 *
 *  @return
 *   == \ref BDM_RC_OK => success
 */
uint8_t f_CMD_SWD_WRITE_REG(void) {
   uint8_t rc;

   // Write data value to DCRDR holding register
   rc = swd_writeMemoryWord(DCRDR_ADDR, commandBuffer+4);
   if (rc != BDM_RC_OK) {
	  return rc;
   }
   // Use commandBuffer as scratch
   commandBuffer[4+0] = 0;
   commandBuffer[4+1] = DCRSR_WRITE_B1;
   commandBuffer[4+2] = 0;
   commandBuffer[4+3] = commandBuffer[3];
   // Execute register transfer
   return swd_coreRegisterOperation(commandBuffer+4);
}

//! ARM-SWD -  Modifies value in LSB of DHCSR
//!  DHCSR.lsb = (DHCSR&preserveBits)|setBits
//!
//! @param preserveBits - Bits to preserve (done first)
//! @param setBits      - Bits to set (done last)
//!
//! @return
//!    == \ref BDM_RC_OK => success       \n
//!    != \ref BDM_RC_OK => error         \n
//!
static uint8_t modifyDHCSR(uint8_t preserveBits, uint8_t setBits) {
	   uint8_t debugStepValue[4];
	   uint8_t rc;

	   rc = swd_readMemoryWord(DHCSR_ADDR, debugStepValue);
	   if (rc != BDM_RC_OK) {
	      return rc;
	   }
	   debugStepValue[0]  = DHCSR_DBGKEY_B0;
	   debugStepValue[1]  = DHCSR_DBGKEY_B1;
	   debugStepValue[2]  = 0;
	   debugStepValue[3] &= preserveBits;
	   debugStepValue[3] |= setBits;
	   return swd_writeMemoryWord(DHCSR_ADDR, debugStepValue);
}

/**  ARM-SWD -  Step over 1 instruction
 *
 *  @return
 *     == \ref BDM_RC_OK => success       \n
 *     != \ref BDM_RC_OK => error         \n
 */
uint8_t f_CMD_SWD_TARGET_STEP(void) {
   // Preserve DHCSR_C_MASKINTS value
   return modifyDHCSR(DHCSR_C_MASKINTS_B3, DHCSR_C_STEP_B3|DHCSR_C_DEBUGEN_B3);
}

/**  ARM-SWD -  Start code execution
 *
 *  @return
 *     == \ref BDM_RC_OK => success       \n
 *     != \ref BDM_RC_OK => error         \n
 */
uint8_t f_CMD_SWD_TARGET_GO(void) {
   return modifyDHCSR(DHCSR_C_MASKINTS_B3, DHCSR_C_DEBUGEN_B3);
}

/* ARM-SWD -  Stop the target
 *
 *  @return
 *     == \ref BDM_RC_OK => success       \n
 *     != \ref BDM_RC_OK => error         \n
 */
uint8_t f_CMD_SWD_TARGET_HALT(void) {
   return modifyDHCSR(DHCSR_C_MASKINTS_B3, DHCSR_C_HALT_B3|DHCSR_C_DEBUGEN_B3);
}
#endif
