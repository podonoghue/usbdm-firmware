/*! \file
    \brief USBDM - Main command procedure for executing BDM commands.

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
   +===============================================================================================
   | 20 May 2012 | Extended firmware version information                                    V4.9.5
   | 20 Apr 2011 | Added DE to f_CMD_USBDM_CONTROL_PINS                               - pgo V4.7
   | 20 Apr 2011 | Added CMD_USBDM_SET_VDD                                            - pgo V4.6
   | 20 Apr 2011 | Added CMD_USBDM_CONTROL_PINS                                       - pgo V4.6
   | 31 Mar 2011 | Added command toggle                                               - pgo V4.6
   | 24 Feb 2011 | Extended auto-connect options                                      - pgo V4.6
   | 26 Nov 2010 | Modified f_CMD_SET_OPTIONS so changes do NOT have immediate effect - pgo V4.3
   |  8 Aug 2010 | Added supported target check                                       - pgo 
   |  8 Aug 2010 | Re-arranged target execution tables & added DSC                    - pgo 
   | 21 Jun 2010 | Added T_MC56F800xx Type                                            - pgo
   | 20 May 2010 | Added JTAG as separate command table                               - pgo
   | 10 May 2010 | Many changes in Vpp/RS08 code (deleted!)                           - pgo
   |  1 May 2010 | Action option changes immediately in f_CMD_SET_OPTIONS()           - pgo
   |    Oct 2009 | Extended status information available                              - pgo
   |    Sep 2009 | Major changes for V2                                               - pgo
   -=======================================================================================
   |  3 Feb 2009 | Extended bdm_setInterfaceLeve[]                                    - pgo
   | 20 Jan 2009 | Merged HCS/CFV1 code from USBDM                                    - pgo
   | 13 Jan 2009 | Moved CFVx commands to separate file                               - pgo
   | 12 Jan 2009 | Changed to dispatch table & individual functions                   - pgo
   |  1 Jan 2009 | Ported to JMxx from JB16 TBLCF code                                - pgo
   +=======================================================================================
   \endverbatim
*/
#include <string.h>
#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "BDMCommon.h"
#include "BDM_CF.h"
#include "BDM_RS08.h"
#include "BDM.h"
#include "USB.h"
#include "ICP.h"
#include "CmdProcessing.h"
#include "CmdProcessingHCS.h"
#include "CmdProcessingCFVx.h"
#include "CmdProcessingCFV1.h"

#ifdef __HC08__
#pragma DATA_SEG __SHORT_SEG Z_PAGE
#endif // __HC08__
//! Status of the BDM
//!
//! see \ref CableStatus_t
CableStatus_t cable_status = { T_OFF };

//! Options for the BDM
//!
//! see \ref BDM_Option_t
BDM_Option_t bdm_option = {
   BDM_TARGET_VDD_OFF,  //!< Target Vdd (off, 3.3V or 5V)
   FALSE,               //!< Cycle target Power when resetting
   FALSE,               //!< Cycle target Power if connection problems (when resetting?)
   FALSE,               //!< Leave target power on when exiting
   AUTOCONNECT_STATUS,  //!< Automatically re-connect to target (for speed change)
   TRUE,                //!< Guess speed for target w/o ACKN
   CS_DEFAULT,          //!< Use alternative BDM clock source in target
   FALSE,               //!< Use RESET signal on BDM interface
   {0}                  //   Reserved
};
#pragma DATA_SEG DEFAULT

//! Buffer for Rx and Tx of commands & results
U8  commandBuffer[MAX_COMMAND_SIZE];

#ifdef __HC08__
#pragma DATA_SEG __SHORT_SEG Z_PAGE
#endif // __HC08__
static U8  commandStatus;      //!< Error code from last/current command
U8  returnSize;                //!< Size of command result
#pragma DATA_SEG DEFAULT

//==========================================================================
// Modeless commands
//==========================================================================

//! Creates status byte
//!
//! @return 16-bit status byte \ref StatusBitMasks_t
//!
U16 makeStatusWord(void) {
U16 status = 0;

   // Target specific checks
   switch (cable_status.target_type) {
#if (HW_CAPABILITY&CAP_CFVx_HW) && (TARGET_CAPABILITY&CAP_PST)
   case T_CFVx : 
	  if (ALLPST_IS_HIGH) {
         status |= S_HALT;  // Target is halted
	  }
      break;
#endif
#if HW_CAPABILITY&CAP_BDM	   
   case T_HC12:
   case T_HCS08:
   case T_RS08:
   case T_CFV1:
	  if (cable_status.ackn==ACKN) {  // Target supports ACKN and the feature is enabled ?
		 status |= S_ACKN;
	  }
      switch (cable_status.speed) {
//      case SPEED_NO_INFO       : status |= S_NOT_CONNECTED;  break; 
        case SPEED_USER_SUPPLIED : status |= S_USER_DONE;      break; 
        case SPEED_SYNC          : status |= S_SYNC_DONE;      break; 
        case SPEED_GUESSED       : status |= S_GUESS_DONE;     break; 
      }
	  break;
#endif	  
   }
#if (HW_CAPABILITY&CAP_RST_IO)
   if (RESET_IS_HIGH) {
      status |= S_RESET_STATE;   // The RSTO pin is currently high
   }
   if (cable_status.reset==RESET_DETECTED) {
      status |= S_RESET_DETECT;                    // The target was recently reset externally
      if (RESET_IS_HIGH) {
         cable_status.reset = NO_RESET_ACTIVITY;   // Clear the flag if reset pin has returned high
      }
   }
#endif
#if (HW_CAPABILITY&CAP_VDDSENSE)
   switch (cable_status.power) {    // Target has power ?
//    case BDM_TARGET_VDD_NONE : status |= S_POWER_NONE; break;
      case BDM_TARGET_VDD_ERR  : status |= S_POWER_ERR;  break;
      case BDM_TARGET_VDD_INT  : status |= S_POWER_INT;  break;
      case BDM_TARGET_VDD_EXT  : status |= S_POWER_EXT;  break;
   }
#else
   // Assume power present
   status |= S_POWER_EXT;
#endif
#if (HW_CAPABILITY&CAP_FLASH)
   switch (cable_status.flashState) {
//    case BDM_TARGET_VPP_OFF     : status |= S_VPP_OFF;      break;
      case BDM_TARGET_VPP_STANDBY : status |= S_VPP_STANDBY;  break;
      case BDM_TARGET_VPP_ON      : status |= S_VPP_ON;       break;
      case BDM_TARGET_VPP_ERROR   : status |= S_VPP_ERR;      break;
   }
#endif
   return status;
}

//! Optionally re-connects with target
//!
//! @param when indicates situation in which the routine is being called\n
//!       AUTOCONNECT_STATUS  - being called during status query
//!	      AUTOCONNECT_ALWAYS, - being called before command execution
//!
//! @return
//!    == \ref BDM_RC_OK => success       \n
//!    != \ref BDM_RC_OK => error 
//! 
U8 optionalReconnect(U8 when) {
U8 rc = BDM_RC_OK;
#if HW_CAPABILITY&CAP_BDM	 
   switch (cable_status.target_type) {
   case T_HC12:
	  if (cable_status.speed == SPEED_USER_SUPPLIED) //   User has specified speed
         break;
	  // Fall through
   case T_RS08:
   case T_HCS08:
   case T_CFV1:
	   if (bdm_option.autoReconnect == when) // If auto re-connect enabled at this time AND
		  rc = bdm_physicalConnect();        // Make sure of connection
	   break;
   default: ;
   }
#endif
   return rc;
}

//! Dummy routine for unused slots in command table
//!
//! @return
//!    BDM_RC_ILLEGAL_COMMAND => illegal command
//!
U8 f_CMD_ILLEGAL(void) {
   return BDM_RC_ILLEGAL_COMMAND;
}

//! Return status from last command received
//!
//! @return
//!   status from last command
//!
U8 f_CMD_GET_COMMAND_STATUS(void) {
   return commandStatus;
}

//! Various debugging & testing commands
//!
//! @note
//!   commandBuffer           \n
//!    - [1..N] = various commands
//!
//! @return
//!    error code
//!
U8 f_CMD_DEBUG(void) {

DebugSubCommands subCommand = commandBuffer[2];

   switch ((U8)subCommand) {
#if (HW_CAPABILITY&CAP_BDM)     
      case BDM_DBG_ACKN: // try the ACKN feature
         bdm_acknInit();
         commandBuffer[1] = (U8) makeStatusWord(); // return the status byte
         returnSize = 2;
         return BDM_RC_OK;

      case BDM_DBG_SYNC: { // try the sync feature
         U8 rc;
         rc = bdm_syncMeasure();
         if (rc != BDM_RC_OK)
            return rc;
         commandBuffer[1] = (U8)makeStatusWord(); // return the status byte
         (*(U16*)(commandBuffer+2)) = cable_status.sync_length;
         returnSize = 4;
         }
         return BDM_RC_OK;

      case BDM_DBG_TESTPORT: // Check port I/O timing - hangs USB interface
         bdm_checkTiming();
         return BDM_RC_OK;
#endif
#if (HW_CAPABILITY & CAP_FLASH)
      case BDM_DBG_VPP_OFF: // Programming voltage off
         return bdmSetVpp(BDM_TARGET_VPP_OFF );

      case BDM_DBG_VPP_ON:
         // Programming voltage on (requires FLASH12V on first)
         return bdmSetVpp(BDM_TARGET_VPP_ON );

      case BDM_DBG_FLASH12V_OFF: // 12V charge pump off
         return bdmSetVpp(BDM_TARGET_VPP_OFF );

      case BDM_DBG_FLASH12V_ON:  // 12V charge pump on
         return bdmSetVpp(BDM_TARGET_VPP_STANDBY );
#endif
#if (HW_CAPABILITY & CAP_VDDCONTROL)
      case BDM_DBG_VDD_OFF: // Target Vdd voltage off
         VDD_OFF();
         return BDM_RC_OK;

      case BDM_DBG_VDD3_ON: // Target Vdd voltage on
         VDD3_ON();
         return BDM_RC_OK;

      case BDM_DBG_VDD5_ON: // Target Vdd voltage on
         VDD5_ON();
         return BDM_RC_OK;

      case BDM_DBG_CYCLE_POWER: // Cycle power to target
         return bdm_cycleTargetVdd(RESET_SPECIAL);
#endif
#if (HW_CAPABILITY & CAP_VDDSENSE)
      case BDM_DBG_MEASURE_VDD: // Measure Target Vdd
         (*(U16*)(commandBuffer+1)) = bdm_targetVddMeasure(); // return the value
         returnSize = 3;
         return BDM_RC_OK;
#endif
#if defined(INLINE_ACKN)
      case BDM_DBG_TESTWAITS:
         bdm_checkWaitTiming();
         return BDM_RC_OK;
#endif

      case BDM_DBG_TESTALTSPEED:
         return BDM_RC_OK;

#if (DEBUG&STACK_DEBUG)
      case BDM_DBG_STACKSIZE: // Measure stack size
         {
         extern char  __SEG_START_SSTACK[];     // Bottom of stack space
         extern char  __SEG_END_SSTACK[];       // Top of stack space
         char *stackProbe = __SEG_START_SSTACK; // Probe for stack RAM

         while (*++stackProbe == 0) { // Find 1st used (non-zero) byte on stack
         }
         (*(U16*)(commandBuffer+1)) = (U16) __SEG_END_SSTACK - (U16) stackProbe;
         returnSize = 3;
         return BDM_RC_OK;
         }
#endif // (DEBUG&STACK_DEBUG)
#if (HW_CAPABILITY&CAP_BDM)     
      case BDM_DBG_TESTBDMTX: // Test BDM Tx routine
         return bdm_testTx(commandBuffer[3]);
#endif
   } // switch
   return BDM_RC_ILLEGAL_PARAMS;
}

//! Set various options
//!
//! @note
//!   commandBuffer\n
//!   - [2..N] = options (image of \ref BDM_Option_t)
//!
//!  @return
//!    BDM_RC_OK => success
//!
U8 f_CMD_SET_OPTIONS(void) {
U8 rc = BDM_RC_OK;
   // Save BDM Options
   (void)memcpy((U8*)&bdm_option, commandBuffer+2, sizeof(bdm_option));
   return rc;
}

//! Returns capability vector for hardware
//! @return
//!  commandBuffer                                                \n
//!   - [1..2] = BDM capability, see \ref HardwareCapabilities_t  \n
//!   - [3..4] = Maximum command buffer size
//!
U8 f_CMD_GET_CAPABILITIES(void) {
	// Note: CAP_HCS08 & CAP_CFV1 are returned inverted for backwards compatibility
	// Inversion is hidden by driver!
   *(U16*)(commandBuffer+1) = TARGET_CAPABILITY^(CAP_HCS08|CAP_CFV1);  // Returns 16-bit value
   *(U16*)(commandBuffer+3) = MAX_COMMAND_SIZE;
   commandBuffer[5] = VERSION_MAJOR;             // Extended firmware version number nn.nn.nn
   commandBuffer[6] = VERSION_MINOR;
   commandBuffer[7] = VERSION_MICRO;
   returnSize = 8;
   return BDM_RC_OK;
}

//! Return status of BDM communication
//!
//! @note
//!   commandBuffer\n
//!  - [1..2] = BDM status
//!
U8 f_CMD_GET_BDM_STATUS(void) {
U16 word;

   // Update power status
   (void)bdm_checkTargetVdd();
   
   word = makeStatusWord();

   commandBuffer[1] = (U8) (word>>8);
   commandBuffer[2] = (U8) word;
   returnSize  = 3;

   return BDM_RC_OK;
}

//! Directly control pins
//!
//! @note
//!   commandBuffer\n
//!     Entry: [2..3] = control value\n
//!     Exit:  [1..2] = pin values (MSB unused) - not yet implemented
//!
U8 f_CMD_CONTROL_PINS(void) {

   U16 control = *(U16*)(commandBuffer+2);
   
   // Set up for OK return
   commandBuffer[1] = 0;
   commandBuffer[2] = 0;
   returnSize  = 3;
   
   if (control == PIN_RELEASE) {
	   switch (cable_status.target_type) {
#if HW_CAPABILITY&CAP_BDM	 	   
	   case T_HC12 :  
	   case T_HCS08 :
	   case T_RS08 :
	   case T_CFV1 :
		   bdmHCS_interfaceIdle();
		   break;
#endif		   
#if (HW_CAPABILITY&CAP_CFVx_HW)
	   case T_CFVx :
		   bdmcf_interfaceIdle();
		   break;
#endif
#if (HW_CAPABILITY&CAP_JTAG_HW)
	   case T_JTAG :
	   case T_ARM_JTAG :
	   case T_MC56F80xx :
		   jtag_interfaceIdle();
		   break;
#endif
	   case T_OFF :
	   default:
		   return BDM_RC_ILLEGAL_COMMAND;
	   }
	   return BDM_RC_OK;
   }
   // Restrict control to mode specific active pins
   switch (cable_status.target_type) {
   case T_HC12 :  
   case T_HCS08 :
   case T_RS08 :
   case T_CFV1 :
	   if (control & ~(PIN_BKGD|PIN_RESET))
		   return BDM_RC_ILLEGAL_PARAMS;
	   break;
#if (HW_CAPABILITY&CAP_CFVx_HW)
   case T_CFVx :
	   if (control & ~(PIN_TA|PIN_RESET|PIN_BKPT))
		   return BDM_RC_ILLEGAL_PARAMS;
	   break;
#endif
#if (HW_CAPABILITY&CAP_JTAG_HW)
   case T_MC56F80xx :
	   if (control & ~(PIN_TRST|PIN_RESET|PIN_DE))
		   return BDM_RC_ILLEGAL_PARAMS;
	   break;
   case T_JTAG :
   case T_ARM_JTAG :
	   if (control & ~(PIN_TRST|PIN_RESET))
		   return BDM_RC_ILLEGAL_PARAMS;
	   break;
#endif	   
   case T_OFF :
   default:
	   return BDM_RC_ILLEGAL_PARAMS;
   }

#if (HW_CAPABILITY & CAP_BDM)
   switch (control & PIN_BKGD) {
   case PIN_BKGD_3STATE : 
       DATA_PORT      = BDM_DIR_Rx_RD;                // Disable BKGD buffer, BKGD = X
       DATA_PORT_DDR  = BDM_DIR_Rx_MASK;              // Enable port pins
       break;
   case PIN_BKGD_LOW :
       DATA_PORT      = BDM_DIR_Rx_WR;                // Enable BKGD buffer,  BKGD = 0
       DATA_PORT_DDR  = BDM_DIR_Rx_MASK|BDM_OUT_MASK; // Enable port pins
       break;
   case PIN_BKGD_HIGH :
       DATA_PORT      = BDM_DIR_Rx_WR  |BDM_OUT_MASK; // Enable BKGD buffer,  BKGD = 1
       DATA_PORT_DDR  = BDM_DIR_Rx_MASK|BDM_OUT_MASK; // Enable port pins
       break;
   }
#endif
#if (HW_CAPABILITY & CAP_RST_IO)
   switch (control & PIN_RESET) {
   case PIN_RESET_3STATE : 
	   RESET_3STATE(); 
	   break;
   case PIN_RESET_LOW :
	   RESET_LOW(); 
	   break;
   }
#endif
   
#if (HW_CAPABILITY&CAP_JTAG_HW)
   switch(control&PIN_TRST) {
   case PIN_TRST_3STATE:
	   TRST_3STATE();
	   break;
   case PIN_TRST_LOW:
	   TRST_LOW();
	   break;
   }
#endif
#if (HW_CAPABILITY&CAP_CFVx_HW)
   switch (control & PIN_TA) {
   case PIN_TA_3STATE : 
	   TA_3STATE(); 
	   break;
   case PIN_TA_LOW :
	   TA_LOW(); 
	   break;
   }

   switch(control&PIN_BKPT) {
   case PIN_BKPT_3STATE:
	   BKPT_HIGH(); // should be 3-state!
	   break;
   case PIN_BKPT_LOW:
	   BKPT_LOW();
	   break;
   }
#endif
   
   return BDM_RC_OK;
}

//! Directly control Target Vdd
//!
//! @note
//!   commandBuffer\n
//!     Entry: [2..3] = control value (MSB unused)\n
//!     Exit:  none
//!
U8 f_CMD_SET_VDD(void) {
   U8 rc;
#if (HW_CAPABILITY&CAP_VDDCONTROL)
   bdm_option.targetVdd = commandBuffer[3];
   rc = bdm_setTargetVdd();
#else
   rc = bdm_checkTargetVdd();
#endif
   // It's OK if there is no external power at the moment
   if ((commandBuffer[3] == BDM_TARGET_VDD_OFF) && 
       (rc == BDM_RC_VDD_NOT_PRESENT))
	   rc = BDM_RC_OK;
   return rc;
}

//=================================================
// Command Dispatch code
//=================================================
//! Ptr to command function
typedef U8 (*FunctionPtr)(void);

//! Structure representing a set of function ptrs 
typedef struct {
   U8 firstCommand;					//!< First command value accepted
   U8 size;             			//!< Size of command structure
   const FunctionPtr *functions; 	//!< Ptr to commands
} FunctionPtrs;

extern U8 f_CMD_SET_TARGET(void);

static const FunctionPtr commonFunctionPtrs[] = {
   // Common to all targets
   f_CMD_GET_COMMAND_STATUS         ,//= 0,  CMD_USBDM_GET_COMMAND_STATUS
   f_CMD_SET_TARGET                 ,//= 1,  CMD_USBDM_SET_TARGET
   f_CMD_SET_VDD                    ,//= 2,  CMD_USBDM_SET_VDD
#if (DEBUG&DEBUG_COMMANDS)
   f_CMD_DEBUG                      ,//= 3,  CMD_USBDM_DEBUG
#else
   f_CMD_ILLEGAL                    ,//= 3,  CMD_USBDM_DEBUG - disabled
#endif
   f_CMD_GET_BDM_STATUS             ,//= 4,  CMD_USBDM_GET_BDM_STATUS
   f_CMD_GET_CAPABILITIES           ,//= 5,  CMD_USBDM_GET_CAPABILITIES
   f_CMD_SET_OPTIONS                ,//= 6,  CMD_USBDM_SET_OPTIONS
// f_CMD_GET_SETTINGS               ,//= 7,  CMD_USBDM_GET_SETTINGS
   f_CMD_ILLEGAL                    ,//= 7,  Reserved
   f_CMD_CONTROL_PINS               ,//= 8,  CMD_USBDM_CONTROL_PINS
   f_CMD_ILLEGAL                    ,//= 9,  Reserved
   f_CMD_ILLEGAL                    ,//= 10, Reserved
   f_CMD_ILLEGAL                    ,//= 11, Reserved
   f_CMD_ILLEGAL                    ,//= 12, CMD_USBDM_GET_VER (EP0)
   f_CMD_ILLEGAL                    ,//= 13, Reserved
   f_CMD_ILLEGAL                    ,//= 14, CMD_USBDM_ICP_BOOT (EP0)
};

#if (TARGET_CAPABILITY&CAP_HCS12)
static const FunctionPtr HCS12functionPtrs[] = {
   // Target specific versions
   f_CMD_CONNECT                    ,//= 15, CMD_USBDM_CONNECT
   f_CMD_SET_SPEED                  ,//= 16, CMD_USBDM_SET_SPEED
   f_CMD_GET_SPEED                  ,//= 17, CMD_USBDM_GET_SPEED

   f_CMD_CONTROL_INTERFACE          ,//= 18, CMD_USBDM_CONTROL_INTERFACE
   f_CMD_ILLEGAL                    ,//= 19, RESERVED

   f_CMD_READ_STATUS_REG            ,//= 20, CMD_USBDM_READ_STATUS_REG
   f_CMD_WRITE_CONTROL_REG          ,//= 21, CMD_USBDM_WRITE_CONTROL_REG

   f_CMD_RESET                      ,//= 22, CMD_USBDM_TARGET_RESET
   f_CMD_STEP                       ,//= 23, CMD_USBDM_TARGET_STEP
   f_CMD_GO                         ,//= 24, CMD_USBDM_TARGET_GO
   f_CMD_HALT                       ,//= 25, CMD_USBDM_TARGET_HALT

   f_CMD_HCS12_WRITE_REG            ,//= 26, CMD_USBDM_WRITE_REG
   f_CMD_HCS12_READ_REG             ,//= 27, CMD_USBDM_READ_REG

   f_CMD_ILLEGAL                    ,//= 28, CMD_USBDM_WRITE_CREG
   f_CMD_ILLEGAL                    ,//= 29, CMD_USBDM_READ_CREG

   f_CMD_WRITE_BD                   ,//= 30, CMD_USBDM_WRITE_DREG
   f_CMD_READ_BD                    ,//= 31, CMD_USBDM_READ_DREG

   f_CMD_HCS12_WRITE_MEM            ,//= 32, CMD_USBDM_WRITE_MEM
   f_CMD_HCS12_READ_MEM             ,//= 33, CMD_USBDM_READ_MEM
   };
static const FunctionPtrs HCS12FunctionPointers = {CMD_USBDM_CONNECT,
                                                   sizeof(HCS12functionPtrs)/sizeof(FunctionPtr),
                                                   HCS12functionPtrs};
#endif

#if (TARGET_CAPABILITY&(CAP_HCS08|CAP_RS08))
static const FunctionPtr HCS08functionPtrs[] = {
   // Target specific versions
   f_CMD_CONNECT                    ,//= 15, CMD_USBDM_CONNECT
   f_CMD_SET_SPEED                  ,//= 16, CMD_USBDM_SET_SPEED
   f_CMD_GET_SPEED                  ,//= 17, CMD_USBDM_GET_SPEED

   f_CMD_CONTROL_INTERFACE          ,//= 18, CMD_USBDM_CONTROL_INTERFACE
   f_CMD_ILLEGAL                    ,//= 19, RESERVED

   f_CMD_READ_STATUS_REG            ,//= 20, CMD_USBDM_READ_STATUS_REG
   f_CMD_WRITE_CONTROL_REG          ,//= 21, CMD_USBDM_WRITE_CONTROL_REG

   f_CMD_RESET                      ,//= 22, CMD_USBDM_TARGET_RESET
   f_CMD_STEP                       ,//= 23, CMD_USBDM_TARGET_STEP
   f_CMD_GO                         ,//= 24, CMD_USBDM_TARGET_GO
   f_CMD_HALT                       ,//= 25, CMD_USBDM_TARGET_HALT

   f_CMD_HCS08_WRITE_REG            ,//= 26, CMD_USBDM_WRITE_REG
   f_CMD_HCS08_READ_REG             ,//= 27, CMD_USBDM_READ_REG

   f_CMD_ILLEGAL                    ,//= 28, CMD_USBDM_WRITE_CREG
   f_CMD_ILLEGAL                    ,//= 29, CMD_USBDM_READ_CREG

   f_CMD_WRITE_BKPT                 ,//= 30, CMD_USBDM_WRITE_DREG
   f_CMD_READ_BKPT                  ,//= 31, CMD_USBDM_READ_DREG

   f_CMD_HCS08_WRITE_MEM            ,//= 32, CMD_USBDM_WRITE_MEM
   f_CMD_HCS08_READ_MEM             ,//= 33, CMD_USBDM_READ_MEM

#if (TARGET_CAPABILITY & CAP_RS08)
   f_CMD_ILLEGAL                    ,//= 34, CMD_USBDM_TRIM_CLOCK - obsolete
   f_CMD_ILLEGAL                    ,//= 35, CMD_USBDM_RS08_FLASH_ENABLE - obsolete
   f_CMD_ILLEGAL                    ,//= 36, CMD_USBDM_RS08_FLASH_STATUS - obsolete
   f_CMD_ILLEGAL                    ,//= 37, CMD_USBDM_RS08_FLASH_DISABLE - obsolete
   f_CMD_ILLEGAL                    ,//= 38, CMD_USBDM_JTAG_GOTORESET
   f_CMD_ILLEGAL                    ,//= 39, CMD_USBDM_JTAG_GOTOSHIFT
   f_CMD_ILLEGAL                    ,//= 40, CMD_USBDM_JTAG_WRITE
   f_CMD_ILLEGAL                    ,//= 41, CMD_USBDM_JTAG_READ
   f_CMD_SET_VPP                    ,//= 42, CMD_USBDM_SET_VPP
#endif
   };
static const FunctionPtrs HCS08FunctionPointers = {CMD_USBDM_CONNECT,
                                                   sizeof(HCS08functionPtrs)/sizeof(FunctionPtr),
                                                   HCS08functionPtrs};
#endif

#if (TARGET_CAPABILITY&CAP_CFV1)
static const FunctionPtr CFV1functionPtrs[] = {
   // Target specific versions
   f_CMD_CONNECT                    ,//= 15, CMD_USBDM_CONNECT
   f_CMD_SET_SPEED                  ,//= 16, CMD_USBDM_SET_SPEED
   f_CMD_GET_SPEED                  ,//= 17, CMD_USBDM_GET_SPEED

   f_CMD_CONTROL_INTERFACE          ,//= 18, CMD_USBDM_CONTROL_INTERFACE
   f_CMD_ILLEGAL                    ,//= 19, RESERVED

   f_CMD_READ_STATUS_REG            ,//= 20, CMD_USBDM_READ_STATUS_REG
   f_CMD_WRITE_CONTROL_REG          ,//= 21, CMD_USBDM_WRITE_CONTROL_REG

   f_CMD_RESET                      ,//= 22, CMD_USBDM_TARGET_RESET
   f_CMD_STEP                       ,//= 23, CMD_USBDM_TARGET_STEP
   f_CMD_GO                         ,//= 24, CMD_USBDM_TARGET_GO
   f_CMD_HALT                       ,//= 25, CMD_USBDM_TARGET_HALT

   f_CMD_CF_WRITE_REG               ,//= 26, CMD_USBDM_WRITE_REG
   f_CMD_CF_READ_REG                ,//= 27  CMD_USBDM_READ_REG

   f_CMD_CF_WRITE_CREG              ,//= 28  CMD_USBDM_WRITE_CREG
   f_CMD_CF_READ_CREG               ,//= 29  CMD_USBDM_READ_CREG

   f_CMD_CF_WRITE_DREG              ,//= 30  CMD_USBDM_WRITE_DREG
   f_CMD_CF_READ_DREG               ,//= 31  CMD_USBDM_READ_DREG

   f_CMD_CF_WRITE_MEM               ,//= 32  CMD_USBDM_WRITE_MEM
   f_CMD_CF_READ_MEM                ,//= 33  CMD_USBDM_READ_MEM
   };
static const FunctionPtrs CFV1FunctionPointers  = {CMD_USBDM_CONNECT,
                                                   sizeof(CFV1functionPtrs)/sizeof(FunctionPtr),
                                                   CFV1functionPtrs};
#endif

#if (TARGET_CAPABILITY&CAP_CFVx)
static const FunctionPtr CFVxfunctionPtrs[] = {
   // Target specific versions
   f_CMD_CFVx_RESYNC                ,//= 15, CMD_USBDM_CONNECT
   f_CMD_SPI_SET_SPEED              ,//= 16, CMD_USBDM_SET_SPEED
   f_CMD_SPI_GET_SPEED              ,//= 17, CMD_USBDM_GET_SPEED

   f_CMD_CFVx_CONTROL_INTERFACE     ,//= 18, CMD_USBDM_CONTROL_INTERFACE
   f_CMD_ILLEGAL                    ,//= 19, RESERVED

   f_CMD_CFVx_READ_STATUS_REG       ,//= 20, CMD_USBDM_READ_STATUS_REG
   f_CMD_ILLEGAL                    ,//= 21, CMD_USBDM_WRITE_CONTROL_REG

   f_CMD_CFVx_RESET                 ,//= 22, CMD_USBDM_TARGET_RESET
   f_CMD_CFVx_STEP                  ,//= 23, CMD_USBDM_TARGET_STEP
   f_CMD_CFVx_GO                    ,//= 24, CMD_USBDM_TARGET_GO
   f_CMD_CFVx_HALT                  ,//= 25, CMD_USBDM_TARGET_HALT

   f_CMD_CFVx_WRITE_REG             ,//= 26, CMD_USBDM_WRITE_REG
   f_CMD_CFVx_READ_REG              ,//= 27  CMD_USBDM_READ_REG

   f_CMD_CFVx_WRITE_CREG            ,//= 28  CMD_USBDM_WRITE_CREG
   f_CMD_CFVx_READ_CREG             ,//= 29  CMD_USBDM_READ_CREG

   f_CMD_CFVx_WRITE_DREG            ,//= 30  CMD_USBDM_WRITE_DREG
   f_CMD_CFVx_READ_DREG             ,//= 31  CMD_USBDM_READ_DREG

   f_CMD_CFVx_WRITE_MEM             ,//= 32  CMD_USBDM_WRITE_MEM
   f_CMD_CFVx_READ_MEM              ,//= 33  CMD_USBDM_READ_MEM

   f_CMD_ILLEGAL                    ,//= 34, CMD_USBDM_TRIM_CLOCK
   f_CMD_ILLEGAL                    ,//= 35, CMD_USBDM_RS08_FLASH_ENABLE
   f_CMD_ILLEGAL                    ,//= 36, CMD_USBDM_RS08_FLASH_STATUS
   f_CMD_ILLEGAL                    ,//= 37, CMD_USBDM_RS08_FLASH_DISABLE
   };
static const FunctionPtrs CFVxFunctionPointers  = {CMD_USBDM_CONNECT,
                                                   sizeof(CFVxfunctionPtrs)/sizeof(FunctionPtr),
                                                   CFVxfunctionPtrs};
#endif 

#if (TARGET_CAPABILITY&(CAP_DSC|CAP_JTAG|CAP_ARM_JTAG))
static const FunctionPtr JTAGfunctionPtrs[] = {
   // Target specific versions
   f_CMD_ILLEGAL                    ,//= 15, CMD_USBDM_CONNECT
   f_CMD_SPI_SET_SPEED              ,//= 16, CMD_USBDM_SET_SPEED
   f_CMD_SPI_GET_SPEED              ,//= 17, CMD_USBDM_GET_SPEED
   f_CMD_CFVx_CONTROL_INTERFACE     ,//= 18, CMD_USBDM_CONTROL_INTERFACE
   f_CMD_ILLEGAL                    ,//= 19, RESERVED
   f_CMD_ILLEGAL                    ,//= 20, CMD_USBDM_READ_STATUS_REG
   f_CMD_ILLEGAL                    ,//= 21, CMD_USBDM_WRITE_CONTROL_REG
   f_CMD_JTAG_RESET                 ,//= 22, CMD_USBDM_TARGET_RESET
   f_CMD_ILLEGAL                    ,//= 23, CMD_USBDM_TARGET_STEP
   f_CMD_ILLEGAL                    ,//= 24, CMD_USBDM_TARGET_GO
   f_CMD_ILLEGAL                    ,//= 25, CMD_USBDM_TARGET_HALT
   f_CMD_ILLEGAL                    ,//= 26, CMD_USBDM_WRITE_REG
   f_CMD_ILLEGAL                    ,//= 27  CMD_USBDM_READ_REG
   f_CMD_ILLEGAL                    ,//= 28  CMD_USBDM_WRITE_CREG
   f_CMD_ILLEGAL                    ,//= 29  CMD_USBDM_READ_CREG
   f_CMD_ILLEGAL                    ,//= 30  CMD_USBDM_WRITE_DREG
   f_CMD_ILLEGAL                    ,//= 31  CMD_USBDM_READ_DREG
   f_CMD_ILLEGAL                    ,//= 32  CMD_USBDM_WRITE_MEM
   f_CMD_ILLEGAL                    ,//= 33  CMD_USBDM_READ_MEM
   f_CMD_ILLEGAL                    ,//= 34, CMD_USBDM_TRIM_CLOCK
   f_CMD_ILLEGAL                    ,//= 35, CMD_USBDM_RS08_FLASH_ENABLE
   f_CMD_ILLEGAL                    ,//= 36, CMD_USBDM_RS08_FLASH_STATUS
   f_CMD_ILLEGAL                    ,//= 37, CMD_USBDM_RS08_FLASH_DISABLE
   f_CMD_JTAG_GOTORESET             ,//= 38, CMD_USBDM_JTAG_GOTORESET
   f_CMD_JTAG_GOTOSHIFT             ,//= 39, CMD_USBDM_JTAG_GOTOSHIFT
   f_CMD_JTAG_WRITE                 ,//= 40, CMD_USBDM_JTAG_WRITE
   f_CMD_JTAG_READ                  ,//= 41, CMD_USBDM_JTAG_READ
   f_CMD_ILLEGAL                    ,//= 42, CMD_USBDM_SET_VPP
   f_CMD_JTAG_READ_WRITE            ,//= 43, CMD_USBDM_JTAG_READ_WRITE
   f_CMD_JTAG_EXECUTE_SEQUENCE      ,//= 44, CMD_JTAG_EXECUTE_SEQUENCE
   };
static const FunctionPtrs JTAGFunctionPointers   = {CMD_USBDM_CONNECT,
                                                    sizeof(JTAGfunctionPtrs)/sizeof(FunctionPtr),     
                                                    JTAGfunctionPtrs};
#endif 

//! Ptr to function table for current target type
static const FunctionPtrs *currentFunctions = NULL; // default to empty

//! Set target type
//! Initialise interface for given target
//! @note
//!   commandBuffer        \n
//!   - [2] = target type
//!
U8 f_CMD_SET_TARGET(void) {
U8 target = commandBuffer[2];

   switch (target) {
#if (TARGET_CAPABILITY&CAP_HCS12)
   case T_HC12:
         currentFunctions = &HCS12FunctionPointers;
         break;
#endif
#if (TARGET_CAPABILITY&CAP_HCS08)
      case T_HCS08:
         currentFunctions = &HCS08FunctionPointers;
         break;
#endif
#if (TARGET_CAPABILITY & CAP_RS08)
      case T_RS08:
          currentFunctions = &HCS08FunctionPointers;
          break;
#endif
#if (TARGET_CAPABILITY & CAP_CFV1)
      case T_CFV1:
         currentFunctions = &CFV1FunctionPointers;
         break;
#endif         
#if (TARGET_CAPABILITY&CAP_CFVx)
      case T_CFVx:
         currentFunctions = &CFVxFunctionPointers;
         break;
#endif
#if (TARGET_CAPABILITY&CAP_JTAG)
      case T_JTAG:
         currentFunctions = &JTAGFunctionPointers;
         break;
#endif
#if (TARGET_CAPABILITY&CAP_DSC)
      case T_MC56F80xx:
         currentFunctions = &JTAGFunctionPointers;
         break;
#endif
#if (TARGET_CAPABILITY&CAP_ARM_JTAG)
      case T_ARM_JTAG:
         currentFunctions = &JTAGFunctionPointers;
         break;
#endif
      case T_OFF:
         currentFunctions = NULL;
         break;
         
      default:
         currentFunctions = NULL;
         (void)bdm_setTarget(T_OFF);
         return BDM_RC_UNKNOWN_TARGET; 
   }
   return bdm_setTarget(target);
}

//!  Processes all commands received over USB
//!
//!  The command is expected to be in \ref commandBuffer[1..N]
//!
//!  @return Number of bytes left in commandBuffer to be sent back as response.\n
//!         commandBuffer[0]    = result code, BDM_RC_OK => success, else failure error code\n
//!         commandBuffer[1..N] = command results
//!
U8 commandExec(void) {
BDMCommands command    = commandBuffer[1];  // Command is 1st byte
FunctionPtr commandPtr = f_CMD_ILLEGAL;     // Default to illegal command

#ifdef DEBUG_PIN
   DEBUG_PIN_DDR = 1;
   DEBUG_PIN     = 1;
#endif

   // Check if modeless command
   if ((U8)command < sizeof(commonFunctionPtrs)/sizeof(FunctionPtr)) {
      // Modeless command
      commandPtr = commonFunctionPtrs[(U8)command];
   }
   else {
      // Target specific command
      if (currentFunctions != NULL) {
         int commandIndex = (U8)command - currentFunctions->firstCommand;
         if ((commandIndex >= 0) && (commandIndex < currentFunctions->size))
            commandPtr = currentFunctions->functions[commandIndex];
      }
   }
   // Execute the command
   // Note: returnSize & commandBuffer may be updated by command
   //       returnSize has a default value of 1
   //       commandStatus has a default value of BDM_RC_OK
   //       On error, returnSize is forced to 1
   returnSize       = 1;
   commandStatus = BDM_RC_OK;
   if (command >= CMD_USBDM_READ_STATUS_REG) {
	   // Check if re-connect needed before most commands (always)
	   commandStatus = optionalReconnect(AUTOCONNECT_ALWAYS);
   }
   if (commandStatus == BDM_RC_OK)
	   commandStatus = commandPtr();   // Execute command & update command status
   commandBuffer[0] = commandStatus;  // return command status

   if (commandStatus != BDM_RC_OK) {
      returnSize = 1;  // Return a single byte error code
      // Do any common cleanup here
#if (TARGET_CAPABILITY&CAP_CFVx)
      if (cable_status.target_type == T_CFVx) {
         (void)bdmcf_complete_chk_rx(); //  Send at least 2 NOPs to purge the BDM
         (void)bdmcf_complete_chk_rx(); //  of the offending command
      }
#endif
   }
#ifdef DEBUG_PIN
   DEBUG_PIN_DDR = 1;
   DEBUG_PIN     = 0;
#endif
   return returnSize;
}

#if (VERSION_HW!=(HW_JB+TARGET_HARDWARE))
void commandLoop(void) {
U8 size;
// Define to discard commands at random for command retry testing
//#define TESTDISCARD

   static U8 commandToggle = 0;
#ifdef TESTDISCARD
   static U8 doneErrorFlag = FALSE;
   RTCSC = (2<<RTCSC_RTCLKS_BITNUM)|(8<<RTCSC_RTCPS_BITNUM);
   RTCMOD = 0xFF;
#endif
   
#if 1
   for(;;) {
      enableInterrupts();
      (void)receiveUSBCommand( MAX_COMMAND_SIZE, commandBuffer );
#ifdef TESTDISCARD
      if (RTCCNT == 128) {
    	  if (!doneErrorFlag) {
    	     doneErrorFlag = TRUE;
    	     continue;
    	  }
      }
      else
    	  doneErrorFlag = FALSE;
#endif
      commandToggle = commandBuffer[1] & 0x80;
      commandBuffer[1] &= 0x7F;
      size = commandExec();
      commandBuffer[0] |= commandToggle;
	  sendUSBResponse( size, commandBuffer );
   }
#elif 0
   for(;;) {
      enableInterrupts();
      (void)receiveUSBCommand( MAX_COMMAND_SIZE, commandBuffer );
      if (commandBuffer[1] == CMD_USBDM_GET_CAPABILITIES)
    	  commandToggle = 0;
      commandBuffer[1] &= 0x7F;
      size = commandExec();
      if (commandToggle)
    	 commandBuffer[0] |= 0x80;
      else
     	 commandBuffer[0] &= ~0x80;
      commandToggle += 0x80;
      sendUSBResponse( size, commandBuffer );
   }
#else
   for(;;) {
      enableInterrupts();
      (void)receiveUSBCommand( MAX_COMMAND_SIZE, commandBuffer );
      size = commandExec();
      sendUSBResponse( size, commandBuffer );
   }
#endif
}
#endif
