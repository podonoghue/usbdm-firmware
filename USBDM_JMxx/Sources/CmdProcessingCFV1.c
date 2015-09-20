/*! \file
    \brief USBDM - Coldfire V1 BDM commands.

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
   | 18 Jul 2014 | Added HCS12ZVM support                                             - pgo V4.10.6.170
   | 28 Feb 2014 | Improved error checking on memory read/write           V4.10.6.120 - pgo
   | 27 Jul 2013 | Added f_CMD_CF_READ_ALL_CORE_REGS()                    V4.10.6     - pgo
   | 15 Feb 2011 | Masked address value for CFV1                          V4.5        - pgo
   | 14 Apr 2010 | Fixed f_CMD_CF_READ_DREG for MC51AC256_HACK                        - pgo
   | 01 Apr 2010 | Fixed byte read/writes to CSR2 etc                                 - pgo
   |    Oct 2009 | Added byte read/writes to CSR2 etc                                 - pgo
   |    Sep 2009 | Major changes for V2                                               - pgo
   -=======================================================================================
   | 10 Dec 2008 | Added MC51AC256 Hack                                               - pgo
   | 13 Jun 2008 | Changed mem_write/read [coldfire]                                  - pgo
   +=======================================================================================
   \endverbatim
*/
#include <string.h>
#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "BDM.h"
#include "BDMMacros.h"
#include "BDMCommon.h"
#include "CmdProcessing.h"
#include "CmdProcessingHCS.h"
#include "TargetDefines.h"
#include "CmdProcessingCFV1.h"

//======================================================================
//======================================================================
//======================================================================
#if (TARGET_CAPABILITY&CAP_CFV1) || (TARGET_CAPABILITY&CAP_S12Z)
//! Write CFV1 Memory
//!
//! @note
//!  commandBuffer\n
//!   - [2]     =>  size of data elements
//!   - [3]     =>  # of bytes
//!   - [4..7]  =>  Memory address [MSB ignored]
//!   - [8..N]  =>  Data to write
//!
//! @return
//!   BDM_RC_OK
//!
U8 f_CMD_CF_WRITE_MEM(void) {
U8  elementSize = commandBuffer[2];          // Size of the data writes
U8  count       = commandBuffer[3];          // # of bytes
U32 addr        = *(U32*)(commandBuffer+4);  // Address in target memory
U8  *data_ptr   = commandBuffer+8;           // Where the data is
U8  rc          = BDM_RC_OK;

   if (cable_status.speed == SPEED_NO_INFO) {
      return BDM_RC_NO_CONNECTION;
   }
   if (count > 0) {
      switch (elementSize) {
         case 1:
            rc = BDMCF_CMD_WRITE_MEM_B(addr, *data_ptr);
            count--;
            while ((count > 0) && (rc == BDM_RC_OK)) {
               data_ptr       += 1;
               addr           += 1;
               count--;
               rc = BDMCF_CMD_FILL_MEM_B(*data_ptr);
            }
            break;
         case 2:
            rc = BDMCF_CMD_WRITE_MEM_W(addr, *(U16 *)data_ptr);
            count >>= 1;
            count--;
            while ((count > 0) && (rc == BDM_RC_OK)) {
               data_ptr       += 2;
               addr           += 2;
               count--;
               rc = BDMCF_CMD_FILL_MEM_W(*(U16 *)data_ptr);
            }
            break;
         case 4:
            rc = BDMCF_CMD_WRITE_MEM_L(addr, (U32 *)data_ptr);
            count >>= 2;
            count--;
            while ((count > 0) && (rc == BDM_RC_OK)) {
               data_ptr       += 4;
               addr           += 4;
               count--;
               rc = BDMCF_CMD_FILL_MEM_L(*(U32 *)data_ptr);
            }
            break;
         default:
            return BDM_RC_ILLEGAL_PARAMS;
      }
   }
   return rc;
}

//! Read CFV1 Memory
//!
//! @note
//!  commandBuffer\n
//!   - [2]     =>  size of data elements
//!   - [3]     =>  # of bytes
//!   - [4..7]  =>  Memory address [MSB ignored]
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!  != \ref BDM_RC_OK => various errors  \n
//!                                       \n
//!  commandBuffer                        \n
//!   - [1..N]  =>  Data read
//!
U8 f_CMD_CF_READ_MEM(void) {
U8  elementSize = commandBuffer[2];          // Size of the data writes
U8  count       = commandBuffer[3];          // # of data bytes
U32 addr        = *(U32*)(commandBuffer+4);  // Address in target memory
U8 *data_ptr    = commandBuffer+1;            // Where in buffer to write the data
U8 rc           = BDM_RC_OK;

   if (cable_status.speed == SPEED_NO_INFO)
      return BDM_RC_NO_CONNECTION;

   if (count>MAX_COMMAND_SIZE-1)
      return BDM_RC_ILLEGAL_PARAMS;  // requested block+status is too long to fit into the buffer

   returnSize = count+1;
   if (count >0) {
      switch (elementSize) {
         case 1:
            rc = BDMCF_CMD_READ_MEM_B(addr, data_ptr);
            count--;
            while ((count > 0) && (rc == BDM_RC_OK)) {
               data_ptr       += 1;
               addr           += 1;
               count--;
               rc = BDMCF_CMD_DUMP_MEM_B(data_ptr);
            }
            break;
         case 2:
            rc = BDMCF_CMD_READ_MEM_W(addr, (U16 *)data_ptr);
            count >>= 1;
            count--;
            while ((count > 0) && (rc == BDM_RC_OK)) {
               data_ptr       += 2;
               addr           += 2;
               count--;
               rc = BDMCF_CMD_DUMP_MEM_W((U16*)data_ptr);
            }
            break;
         case 4:
            rc = BDMCF_CMD_READ_MEM_L(addr, (U32*)data_ptr);
            count >>= 2;
            count--;
            while ((count > 0) && (rc == BDM_RC_OK)) {
               data_ptr       += 4;
               addr           += 4;
               count--;
               rc = BDMCF_CMD_DUMP_MEM_L((U32*)data_ptr);
            }
            break;
         default:
            return BDM_RC_ILLEGAL_PARAMS;
      }
   }
   return rc;
}

//! Read CFV1 core register (or DREG if MSB=1)
//!
//! @note
//!  commandBuffer\n
//!   - [2..3]  =>  16-bit register number [MSB ignored]
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!                                       \n
//!  commandBuffer                        \n
//!   - [1..4]  =>  32-bit register value
//!
U8 f_CMD_CF_READ_REG(void) {

   returnSize = 5;
   return BDMCF_CMD_READ_REG(commandBuffer[3]&0x9F,(U32*)(commandBuffer+1));
}

//! Write CFV1 core register (or DREG if MSB=1)
//!
//! @note
//!  commandBuffer\n
//!   - [2..3]  =>  8-bit register number [MSB ignored - CRG/CRN byte]
//!   - [4..7]  =>  32-bit register value
//!
//! @return
//!  == \ref BDM_RC_OK => success
//!
U8 f_CMD_CF_WRITE_REG(void) {
   return BDMCF_CMD_WRITE_REG(commandBuffer[3]&0x9F,(*(U32 *)(commandBuffer+4)));
}

#endif

//======================================================================
//======================================================================
//======================================================================

#if (TARGET_CAPABILITY&CAP_CFV1)

#if HW_CAPABILITY&CAP_CORE_REGS
// Insufficient memory !

// Maps register index into magic number for ARM device register number
static const uint8_t regIndexMap[] = {
   CFV1_RegD0, CFV1_RegD1, CFV1_RegD2, CFV1_RegD3, CFV1_RegD4, CFV1_RegD5, CFV1_RegD6, CFV1_RegD7, 
   CFV1_RegA0, CFV1_RegA1, CFV1_RegA2, CFV1_RegA3, CFV1_RegA4, CFV1_RegA5, CFV1_RegA6, CFV1_RegA7,
   CFV1_RegSR, CFV1_RegPC,
   };
//! Read all core registers
//!
//! @note
//!  commandBuffer\n
//!   - [2]  =>  flag - must be zero
//!   - [3]  =>  register index to start at
//!   - [4]  =>  register index to end at
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!                                       \n
//!  commandBuffer                        \n
//!   - [1..N]  =>  32-bit register values
//!
uint8_t f_CMD_CF_READ_ALL_CORE_REGS(void) {
   uint8_t rc;
   uint8_t regIndex    = commandBuffer[3];
   uint8_t endRegister = commandBuffer[4];
   uint8_t* outputPtr  = commandBuffer+1;
   returnSize = 1;
   if (commandBuffer[2] != 0) {
	   // Check flag is zero
	   return BDM_RC_ILLEGAL_PARAMS;
   }
   while (regIndex<=endRegister) {
	   // Write to buffer (target format - Big-endian)
	   rc = BDMCF_CMD_READ_REG(regIndexMap[regIndex],(U32*)(outputPtr));
	   if (rc != BDM_RC_OK) {
		   return rc;
	   }
	   outputPtr  += 4;
	   returnSize += 4;
	   regIndex++;
   }
   return BDM_RC_OK;
}
#endif

//! Write CFV1 debug register;
//!
//! @note
//!  commandBuffer\n
//!   - [2..3]  =>  16-bit register number
//!   - [4..7]  =>  32-bit register value
//!
//! @return
//!  == \ref BDM_RC_OK => success
//!
U8 f_CMD_CF_WRITE_DREG(void) {
U16 regNo = *(U16*)(commandBuffer+2);

   if (regNo >= CFV1_ByteRegs) {
      switch (regNo) {
         case CFV1_DRegXCSRbyte : BDMCF_CMD_WRITE_XCSR(commandBuffer[7]); break;
         case CFV1_DRegCSR2byte : BDMCF_CMD_WRITE_CSR2(commandBuffer[7]); break;
         case CFV1_DRegCSR3byte : BDMCF_CMD_WRITE_CSR3(commandBuffer[7]); break;
         default :                return BDM_RC_ILLEGAL_PARAMS;           break;
      }
      return BDM_RC_OK;
   }
   else {
#ifdef MC51AC256_HACK
      if (regNo == CFV1_DRegCSR)
         commandBuffer[5] |= (CFV1_CSR_VBD>>16); // Hack for MC51AC256
#endif
      return BDMCF_CMD_WRITE_DREG(commandBuffer[3]&0x1F, (*(U32 *)(commandBuffer+4)));
   }
}

//! Read CFV1 debug register;
//!
//! @note
//!  commandBuffer\n
//!   - [2..3]  =>  16-bit register number
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!                                       \n
//!  commandBuffer                        \n
//!   - [1..4]  =>  32-bit register value
//!
U8 f_CMD_CF_READ_DREG(void) {
U16 regNo = *(U16*)(commandBuffer+2);
U8 rc = BDM_RC_OK;

   returnSize = 5;
   
   if (regNo >= CFV1_ByteRegs) {
      commandBuffer[1] = 0x00;
      commandBuffer[2] = 0x00;
      commandBuffer[3] = 0x00;
      switch (regNo) {
         case CFV1_DRegXCSRbyte : BDMCF_CMD_READ_XCSR(commandBuffer+4); break;
         case CFV1_DRegCSR2byte : BDMCF_CMD_READ_CSR2(commandBuffer+4); break;
         case CFV1_DRegCSR3byte : BDMCF_CMD_READ_CSR3(commandBuffer+4); break;
         default :
            return BDM_RC_ILLEGAL_PARAMS;
      }
   }
   else {
      rc = BDMCF_CMD_READ_DREG((U8)regNo&0x1F,(U32*)(commandBuffer+1));
#ifdef MC51AC256_HACK
      if (regNo == CFV1_DRegCSR)
         commandBuffer[2] &= ~(CFV1_CSR_VBD>>16); // Hack for MC51AC256
#endif
   }
   return rc;
}

//!  Write CFV1 control register;
//!
//! @note
//!  commandBuffer\n
//!   - [2..3]  =>  16-bit register number [MSB ignored]
//!   - [4..7]  =>  32-bit register value
//!
//! @return
//!  == \ref BDM_RC_OK => success
//!
U8 f_CMD_CF_WRITE_CREG(void) {
   return BDMCF_CMD_WRITE_CREG(commandBuffer[3]&0x1F, (*(U32 *)(commandBuffer+4)));
}

//! Read CFV1 control register;
//!
//! @note
//!  commandBuffer\n
//!   - [2..3]  =>  16-bit register number [MSB ignored]
//!
//! @return
//!  == \ref BDM_RC_OK => success         \n
//!                                       \n
//!  commandBuffer                        \n
//!   - [1..4]  =>  32-bit register value
//!
U8 f_CMD_CF_READ_CREG(void) {
   returnSize = 5;
   return BDMCF_CMD_READ_CREG(commandBuffer[3]&0x1F, (U32*)(commandBuffer+1));
}


//======================================================================
//======================================================================
//======================================================================

//! @return
//!  == \ref BDM_RC_OK => success
//!
U8 f_CMD_CF_WRITE_CSR2(void) {
   BDMCF_CMD_WRITE_CSR2(commandBuffer[2]);
   return BDM_RC_OK;
}

//! @return
//!  == \ref BDM_RC_OK => success
//!
U8 f_CMD_CF_READ_CSR2(void) {
   BDMCF_CMD_READ_CSR2(commandBuffer+1);;
   return BDM_RC_OK;
}

//! @return
//!  == \ref BDM_RC_OK => success
//!
U8 f_CMD_CF_WRITE_CSR3(void) {
   BDMCF_CMD_WRITE_CSR3(commandBuffer[2]);
   return BDM_RC_OK;
}

//! @return
//!  == \ref BDM_RC_OK => success
//!
U8 f_CMD_CF_READ_CSR3(void) {
   BDMCF_CMD_READ_CSR3(commandBuffer+1);;
   return BDM_RC_OK;
}

#endif
