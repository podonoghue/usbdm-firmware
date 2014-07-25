/*! \file
    \brief USBDM - HCS12, HCS08, RS08 & CFV1 low level BDM communication.

   \verbatim
   This software was modified from \e TBDML software

   USBDM
   Copyright (C) 2007  Peter O'Donoghue

   Turbo BDM Light (TBDML)
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

-=======================================================================================
| 18 Jul 2014 | Added HCS12ZVM support                                             - pgo V4.10.6.170
| 28 Feb 2013 | Changed Timeouts for slow target clocks e.g. 32kHz                 - pgo V4.10.6.120
| 26 Dec 2012 | Changed Reset handling to prevent USB timeouts                     - pgo V4.10.4
| 18 Sep 2012 | Removed output glitch in bdm_rxGeneric() etc (BKGD_3STATE_MASK)    - pgo V4.10.2
|  5 May 2011 | Modified bdm_enableBDM() to be more careful in modifying BDM reg   - pgo V4.6
|  7 Jan 2010 | Modified bdmHC12_confirmSpeed() to reduce unnecessary probing      - pgo V4.3
|  7 Dec 2010 | changed BDM_CMD_0_0_T() etc to leave interrupts disabled           - pgo V4.3
| 14 Apr 2010 | bdmHC12_confirmSpeed() - added FDATA method                        - pgo
| 11 Dec 2009 | Increased size of BDM_SYNC_REQ to cope with 32.5kHz clock          - pgo
| 19 Oct 2009 | Modified Timer code - Folded together with JS16 code               - pgo
| 22 Sep 2009 | Made ACK enabling/disabling explicitly after Connect{}             - pgo
| 15 Sep 2009 | Corrected clearing of ACKN in bdm_physicalConnect{}                - pgo
| 15 Sep 2009 | Increased size of BDM_SYNC_REQ                                     - pgo
| 15 Sep 2009 | Moved bdm_aknInit{} from bdm_physicalConnect{}to bdm_connect{}     - pgo
|    Sep 2009 | Major changes for V2                                               - pgo
-=======================================================================================
|  2 Jun 2009 | Merged USBDM/USBDM_CF versions                                     - pgo
| 15 Apr 2009 | Updated BRSET offsets to be correct with compiler fix MTWX31284    - pgo
|  3 Apr 2009 | Made Rx/TxEmpty empty (were indicating failure even if retry OK!)  - pgo
|  3 Apr 2009 | Minor change to bdmConnect{}- no connection reported if too fast   - pgo
|  1 Apr 2009 | Changes to bdmConnect{} & bdm_enableBDM{} to improve Alt CLK       - pgo
| 30 Mar 2009 | Changes to bdm_setInterfaceLevel{} to remove glitch                - pgo
|  3 Feb 2009 | Extended bdm_setInterfaceLevel{}                                   - pgo
| 27 Jan 2009 | Introduced doACKN_WAIT64{} & doACKN_WAIT150{}                      - pgo
| 21 Jan 2009 | Minor changes to bdm_writeBDMControl{}                             - pgo
| 15 Jan 2009 | Moved shared routines to BDMCommon.c [TBLCF version]               - pgo
| 14 Jan 2009 | Moved timer routines & macros to Timer.c/Timer.h                   - pgo
| 27 Dec 2008 | Added bdm_setInterfaceLevel{}                                      - pgo
|  9 Dec 2008 | Added hack for MC51AC256                                           - pgo
|  9 Dec 2008 | Removed BKGD check in bdm_cycleTargetVddOn{}                       - pgo
| 11 Nov 2008 | Unified Timer MACROS                                               - pgo
|  5 Nov 2008 | Re-arrangement of Target Vdd control routines                      - pgo
|  1 Nov 2008 | Minor mods & corrected comments in bdm_syncMeasure{}               - pgo
|  1 Nov 2008 | Removed glitch in bdm_syncMeasure{} (LVC125 only)                  - pgo
| 23 Oct 2008 | Reviewed stack size                                                - pgo
| 22 Oct 2008 | Increased cable_status.waitX size                                  - pgo
| 22 Oct 2008 | Changed RESET macros - more consistent                             - pgo
|  4 Oct 2008 | Renamed some MACROs for consistency                                - pgo
|  4 Oct 2008 | Ported to UF32                                                     - pgo
|  1 Sep 2008 | Fixed several bugs related use of 1T45 buffers                     - pgo
| 29 Aug 2008 | Disabled ACMP0 as it clashes with Vdd5_En* on CLC                  - pgo
| 10 Aug 2008 | Fixed bug in multi-line MACROs                                     - pgo
| 10 Jul 2008 | Modified Speed Guessing routine (now smarter!)                     - pgo
| 10 Jul 2008 | Modified Reset code slightly                                       - pgo
|  9 Jul 2008 | Modified Rx routine mask to remove glitch                          - pgo
|  7 Jul 2008 | Modified Reset sequence to properly Cycle Vdd                      - pgo
|  2 Jul 2008 | More thorough checking for target Vdd                              - pgo
|  2 Jul 2008 | Modified using Alt BDM clock handling                              - pgo
| 19 Jun 2008 | Expanded Flash programming options of RS08                         - pgo
| 14 Jun 2008 | Added RS08 Trim value calculation                                  - pgo
| 10 Jun 2008 | Added Colfire V1 interface                                         - pgo
|  6 May 2008 | Ported to JB16 from JM60 - lotsa changes                           - pgo
| 12 Apr 2008 | Added reconnect before software reset                              - pgo
|  7 Apr 2008 | Fixed WAIT_WITH_TIMEOUT.. macros                                   - pgo
|  7 Apr 2008 | Fixed glitch in bdmHCS_init                                        - pgo
|  6 Apr 2008 | Added Vdd change detect handler & POR                              - pgo
| 23 Mar 2008 | Fixed several BMD commands not using ACKN                          - pgo
|  3 Mar 2008 | Started changes for JM60 - lots                                    - pgo
+=======================================================================================
\endverbatim
*/

#include <hidef.h>          /* common defines and macros */
#include "Common.h"
#include "Configure.h"
#include "Commands.h"
#include "BDM.h"
#include "BDMMacros.h"

#include "TargetDefines.h"
#include "CmdProcessing.h"
#include "BDMCommon.h"
#include "BDM_RS08.h"

#if __VERSION__ < 5029
#error "BDM.c requires a Compiler fix (MTWX31284) that is only available in Versions later than 5.0.29"
#endif

//=============================================================================================================================================
#define BDM_SYNC_REQms            1U //!< ms - length of the longest possible SYNC REQUEST pulse (128 BDM cycles @ 400kHz = 320us plus some extra time)
#define SYNC_TIMEOUTus          460U //!< us - longest time for the target to completed a SYNC pulse (16+128+margin cycles @ 400kHz = 375us)
#define ACKN_TIMEOUTus         2500U //!< us - longest time after which the target should produce ACKN pulse (150 cycles @ 400kHz = 375us)
#define SOFT_RESETus          10000U //!< us - longest time needed for soft reset of the BDM interface (512 BDM cycles @ 400kHz = 1280us)
#define RESET_LENGTHms          100U //!< ms - time of RESET assertion
#define RESET_INITIAL_WAITms     10U //!< ms - max time to wait for the RESET pin to come high after release
#define RESET_RELEASE_WAITms    270U //!< ms - max time to wait for the RESET pin to come high after release

//! Wait for reset rise with timeout
//!
void bdm_WaitForResetRise(void) {
#if (HW_CAPABILITY&CAP_RST_IO)
   if (bdm_option.useResetSignal) {
      // Wait with timeout until RESET is high
      WAIT_WITH_TIMEOUT_MS(RESET_INITIAL_WAITms, RESET_IS_HIGH);
      if (RESET_IN==0) {
         // May take a while
         setBDMBusy();
      }
      WAIT_WITH_TIMEOUT_MS(RESET_RELEASE_WAITms-RESET_INITIAL_WAITms, RESET_IS_HIGH);
   }
#endif
}

#if (HW_CAPABILITY&CAP_BDM)

/* Function prototypes */
       U8   bdm_syncMeasure(void);
       void bdmHCS_interfaceIdle(void);
static U8   bdmHC12_alt_speed_detect(void);

//========================================================
//
#pragma DATA_SEG __SHORT_SEG Z_PAGE
// MUST be placed into the direct segment (assumed in ASM code).
extern volatile U8 bitDelay;  //!< Used as a general purpose variable in the bdm_Tx{} & bdm_Rx{}etc.
extern volatile U8 rxTiming1; //!< bdm_Rx timing constant #1
static volatile U8 rxTiming2; //!< bdm_Rx timing constant #2
static volatile U8 rxTiming3; //!< bdm_Rx timing constant #3
extern volatile U8 txTiming1; //!< bdm_Tx timing constant #1
static volatile U8 txTiming2; //!< bdm_Tx timing constant #2
static volatile U8 txTiming3; //!< bdm_Tx timing constant #3

#define bitCount bitDelay

// pointers to current bdm_Rx & bdm_Tx routines
U8   (*bdm_rx_ptr)(void) = bdm_rxEmpty; //!< pointers to current bdm_Rx routine
void (*bdm_tx_ptr)(U8)   = bdm_txEmpty; //!< pointers to current bdm_Tx routine

//========================================================
//
#pragma DATA_SEG DEFAULT

#pragma MESSAGE DISABLE C5909 // Disable warnings about Assignment in condition
#pragma MESSAGE DISABLE C4000 // Disable warnings about Condition always true

//! Read Target BDM status
//!
//!  Depending on target architecture this reads from
//!  - BDCSC,
//!  - BDMSTS,
//!  - XCSR.
//!
//! @param bdm_sts value read
//! @return
//!   \ref BDM_RC_OK             => success  \n
//!   \ref BDM_RC_UNKNOWN_TARGET => unknown target
//!
U8 bdm_readBDMStatus(U8 *bdm_sts) {

   switch (cable_status.target_type) {
#if TARGET_CAPABILITY & CAP_S12Z
      case T_HCS12Z  : {
    	  uint16_t temp;
    	  BDMZ12_CMD_READ_BDCCSR(&temp);
    	  *bdm_sts = temp>>8;
      	  }
	  return BDM_RC_OK;
#endif
      case T_HC12:
         BDM12_CMD_BDREADB(HC12_BDMSTS,bdm_sts);
         return BDM_RC_OK;
      case T_HCS08:
      case T_RS08:
         BDM08_CMD_READSTATUS(bdm_sts);
         return BDM_RC_OK;
      case T_CFV1:
         BDMCF_CMD_READ_XCSR(bdm_sts);
         return BDM_RC_OK;
      default:
         return BDM_RC_UNKNOWN_TARGET; // Don't know how to check status on this one!
   }
}

//! Write Target BDM control register
//!
//!  Depending on target architecture this writes to \n
//!   - BDCSC,  \n
//!   - BDMSTS, \n
//!   - XCSR.
//!
//!  @param value => value to write
//!
static void writeBDMControl(U8 value) {

   switch (cable_status.target_type) {
#if TARGET_CAPABILITY & CAP_S12Z
      case T_HCS12Z  :
    	  BDMZ12_CMD_WRITE_BDCCSR((value<<8)|0xFF);
         break;
#endif
      case T_HC12:
         BDM12_CMD_BDWRITEB(HC12_BDMSTS,value);
         break;
      case T_HCS08:
      case T_RS08:
         BDM08_CMD_WRITECONTROL(value);
         break;
      case T_CFV1:
         BDMCF_CMD_WRITE_XCSR(value);
         break;
   }
}

//! Write Target BDM control register [masked value]
//!
//!  Depending on target architecture this writes to \n
//!   - BDCSC,  \n
//!   - BDMSTS, \n
//!   - XCSR.   \n
//!
//! @note - This routine may modify the value written if bdm_option.useAltBDMClock is active
//!
//! @return
//!   \ref BDM_RC_OK             => success  \n
//!   \ref BDM_RC_UNKNOWN_TARGET => unknown target
//!
U8 bdm_writeBDMControl(U8 bdm_ctrl) {
U8 statusClearMask; // Bits to clear in control value
U8 statusSetMask;   // Bits to set in control value
U8 statusClkMask;   // The position of the CLKSW bit in control register

   // Get clock select mask for this target (CLKSW bit in BDM control register)
   switch (cable_status.target_type) {
#if TARGET_CAPABILITY & CAP_S12Z
      case T_HCS12Z  :
#endif
      case T_HC12:
         statusClkMask = HC12_BDMSTS_CLKSW;
         break;
      case T_HCS08:
      case T_RS08:
         statusClkMask = HC08_BDCSCR_CLKSW;
         break;
      case T_CFV1:
         statusClkMask = CFV1_XCSR_CLKSW;
         break;
      default:
         return BDM_RC_UNKNOWN_TARGET; // Don't know how to check status on this one!
   }

   // default - no modification of value given
   statusClearMask = 0xFF; // Clear no bits
   statusSetMask   = 0x00; // Set no bits

   // Construct the masks to modify control value
   switch (bdm_option.useAltBDMClock) {
      case CS_ALT:     statusClearMask = ~statusClkMask;  break; // Force CLKSW = 0
      case CS_NORMAL:  statusSetMask   =  statusClkMask;  break; // Force CLKSW = 1
   }

   bdm_ctrl &= statusClearMask;
   bdm_ctrl |= statusSetMask;

   writeBDMControl(bdm_ctrl);

   return BDM_RC_OK;
}

//! If BDM mode is not enabled in target yet, enable it so it can be made active
//!
//! @return
//!   \ref BDM_RC_OK             => success  \n
//!   \ref BDM_RC_UNKNOWN_TARGET => unknown target \n
//!   \ref BDM_RC_BDM_EN_FAILED  => enabling BDM failed (target not connected or wrong speed ?)
//!
U8 bdm_enableBDM() {
U8 bdm_sts;
U8 rc;

   rc = bdm_readBDMStatus(&bdm_sts); // Get current status
   if (rc != BDM_RC_OK) {
      return rc;
   }
   if (cable_status.target_type==T_CFV1) {
      // CFV1
      if ((bdm_sts & CFV1_XCSR_ENBDM) == 0) {
         // Try to enable BDM
         bdm_sts = (bdm_sts & CFV1_XCSR_CLKSW)|CFV1_XCSR_ENBDM;
         writeBDMControl(bdm_sts);
         rc = bdm_readBDMStatus(&bdm_sts); // Get current status
      }
      // (bdm_sts==0xFF) often indicates BKGD pin is disabled etc
      if ((bdm_sts==0xFF) || (bdm_sts & CFV1_XCSR_ENBDM) == 0) {
         return BDM_RC_BDM_EN_FAILED;
      }
   }
   else {
      // RS08/HCS12/HCS08
      if ((bdm_sts & HC12_BDMSTS_ENBDM) == 0) {
         // Try to enable BDM
         bdm_sts |= HC12_BDMSTS_ENBDM;
         writeBDMControl(bdm_sts);
         rc = bdm_readBDMStatus(&bdm_sts); // Get current status
      }
      // (bdm_sts==0xFF) often indicates BKGD pin is disabled etc.
      if ((bdm_sts==0xFF) || (bdm_sts & HC12_BDMSTS_ENBDM) == 0) {
         return BDM_RC_BDM_EN_FAILED;
      }
   }
   return rc;
}
#pragma MESSAGE DEFAULT C4000 // Restore warnings about Condition always true
#pragma MESSAGE DEFAULT C5909 // Restore warnings about Assignment in condition

#if 0
//! Used before some commands to ensure there is a useful connection to target available.
//!
//! Some non-intrusive commands still require a running processor e.g. READ_BYTE
//! This function will force a stopped CPU into active BDM mode
//!
//! @return
//!   \ref BDM_RC_OK  => success  \n
//!   otherwise       => various errors
//!
U8 bdm_makeActiveIfStopped(void) {

U8 rc = BDM_RC_OK;
U8 bdm_sts;

   if (cable_status.target_type == T_HC12) // Not supported on HC12
      return BDM_RC_OK; // not considered an error

   if (bdm_option.autoReconnect &&            // Auto re-connect enabled &
      (cable_status.speed==SPEED_SYNC)) { // Target supports Sync
      rc = bdm_syncMeasure(); // Check connection speed
      if (rc == 0)
         rc = bdm_RxTxSelect();
      }
   if (rc != BDM_RC_OK)
      return rc;

   rc = bdm_readBDMStatus(&bdm_sts);
   if (rc != BDM_RC_OK)
      return rc;

   // If HC08/RS08 processor is WAITing or STOPped then force background
   if (bdm_sts&HC08_BDCSCR_WS)
      (void)bdm_halt();

   return BDM_RC_OK;
}
#endif

void bdm_clearConnection(void) {
//   cable_status.reset = NO_RESET_ACTIVITY; // Clear the reset flag
   cable_status.speed   = SPEED_NO_INFO;   // No connection
   bdm_rx_ptr           = bdm_rxEmpty;     // Clear the Tx/Rx pointers
   bdm_tx_ptr           = bdm_txEmpty;     //    i.e. no com. routines found
}

//!  Tries to connect to target - doesn't try other strategies such as reset.
//!  This function does a basic connect sequence and tries to enable ACKN.
//!  It doesn't configure the BDM registers on the target.
//!
//! @return
//!    == \ref BDM_RC_OK                  => success                             \n
//!    == \ref BDM_RC_VDD_NOT_PRESENT     => no target power present             \n
//!    == \ref BDM_RC_RESET_TIMEOUT_RISE  => RESET signal timeout - remained low \n
//!    == \ref BDM_RC_BKGD_TIMEOUT        => BKGD signal timeout - remained low  \n
//!    != \ref BDM_RC_OK                  => other failures
//!
U8 bdm_physicalConnect(void) {
U8 rc;

   bdmHCS_interfaceIdle(); // Make sure interface is idle
   bdm_clearConnection();  // Assume we know nothing about connection technique & speed
   
   // Target has power?
   rc = bdm_checkTargetVdd();
   if (rc != BDM_RC_OK) {
      return rc;
   }
#if (HW_CAPABILITY&CAP_RST_IO)
   // Wait with timeout until both RESET and BKGD  are high
   if (bdm_option.useResetSignal) {
	   if (RESET_IN==0) {
		  // May take a while
		  setBDMBusy();
	      WAIT_WITH_TIMEOUT_MS(RESET_RELEASE_WAITms,(RESET_IN!=0)&&(BDM_IN!=0));
	  }
      if (RESET_IN==0) {
         return(BDM_RC_RESET_TIMEOUT_RISE);  // RESET timeout
      }
   }
#else
   // Wait with timeout until BKGD is high
   WAIT_WITH_TIMEOUT_MS(RESET_RELEASE_WAITms,(BDM_IN!=0));
#endif
   if (BDM_IN == 0) {
      return(BDM_RC_BKGD_TIMEOUT);  // BKGD timeout
   }
   // Try SYNC method
   rc = bdm_syncMeasure();
   if (rc != BDM_RC_OK) {
      // try again
      rc = bdm_syncMeasure();
   }
   if (rc == BDM_RC_OK) {
      rc = bdm_RxTxSelect();
      if (rc == BDM_RC_OK) {
         cable_status.speed = SPEED_SYNC;
      }
   }
   else if ((bdm_option.guessSpeed) &&          // Try alternative method if enabled
       (cable_status.target_type == T_HC12)) { // and HC12 target
      rc = bdmHC12_alt_speed_detect();     // Try alternative method (guessing!)
   }
   if (rc == BDM_RC_OK) {
      bdm_acknInit();  // Try the ACKN feature
   }
   return(rc);
}

//! Connect to target
//!
//!  This function may cycle the target power in attempting to connect. \n
//!  It enables BDM on the target if connection is successful.
//!
//! @return
//!    == \ref BDM_RC_OK               => success  \n
//!    == \ref BDM_RC_VDD_NOT_PRESENT  => no target power present \n
//!    == \ref BDM_RC_RESET_TIMEOUT_RISE    => RESET signal timeout - remained low \n
//!    == \ref BDM_RC_BKGD_TIMEOUT     => BKGD signal timeout - remained low \n
//!    != \ref BDM_RC_OK               => other failures \n
//!
U8 bdm_connect(void) {
U8 rc;

   if (cable_status.speed != SPEED_USER_SUPPLIED) {
      rc = bdm_physicalConnect();
      if ((rc != BDM_RC_OK) && bdm_option.cycleVddOnConnect) {
   	     // No connection to target - cycle power if allowed
   	     (void)bdm_cycleTargetVdd(RESET_SPECIAL); // Ignore errors
   	     rc = bdm_physicalConnect(); // Try connect again
      }
      if (rc != BDM_RC_OK) {
   	     return rc;
      }
   }
   // Try to enable BDM
   return bdm_enableBDM();
}

#if (HW_CAPABILITY&CAP_RST_IO)

//! Resets the target using the BDM reset line.
//! @note
//!   Not all targets have a reset pin or support reset to BDM mode with a reset pin.
//!
//! @param mode
//!    - \ref RESET_SPECIAL => Reset to special mode,
//!    - \ref RESET_NORMAL  => Reset to normal mode
//!
//! @return
//!   \ref BDM_RC_OK  => Success \n
//!   \ref BDM_RC_BKGD_TIMEOUT     => BKGD pin stuck low \n
//!   \ref BDM_RC_RESET_TIMEOUT_RISE    => RESET pin stuck low \n
//!
U8 bdm_hardwareReset(U8 mode) {

   if (!bdm_option.useResetSignal)
      return BDM_RC_ILLEGAL_PARAMS;
      // Doesn't return BDM_RC_ILLEGAL_COMMAND as method is controlled by a parameter

#ifdef DISABLE_RESET_SENSE_INT
   DISABLE_RESET_SENSE_INT(); // Mask RESET IC interrupts
#endif
   
   bdmHCS_interfaceIdle();  // Make sure BDM interface is idle

   mode &= RESET_MODE_MASK;

   // Wait with timeout until both RESET and BKGD are high
//   WAIT_WITH_TIMEOUT_MS(RESET_WAIT, (RESET_IN!=0)&&(BDM_IN!=0));
   WAIT_WITH_TIMEOUT_S(2, (RESET_IN!=0)&&(BDM_IN!=0));

   if (RESET_IN==0) return(BDM_RC_RESET_TIMEOUT_RISE);
   if (BDM_IN==0)   return(BDM_RC_BKGD_TIMEOUT);

   bdm_txPrepare(); // BKGD & RESET 3-state control on DIR_PORT

   if (mode==RESET_SPECIAL) {
      BDM_LOW();  // Drive BKGD low
      WAIT_MS(RESET_SETTLEms); // Wait for signals to settle
   }

   RESET_LOW();

   WAIT_MS(RESET_LENGTHms);  // Wait for reset pulse duration

   RESET_3STATE();

   // Wait with timeout until RESET is high
   bdm_WaitForResetRise();

   // Assume RESET risen - check later after cleanUp

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN   = 0;
   DEBUG_PIN   = 1;
#endif

   // Wait 1 ms before releasing BKGD
   WAIT_MS(1);  // Wait for Target to start up after reset

   bdmHCS_interfaceIdle(); // Place interface in idle state

   // Wait recovery time before allowing anything else to happen on the BDM
   WAIT_MS(RESET_RECOVERYms);  // Wait for Target to start up after reset

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   if (RESET_IN==0) {     // RESET failed to rise
      return(BDM_RC_RESET_TIMEOUT_RISE);
   }
#ifdef CLEAR_RESET_SENSE_FLAG
   CLEAR_RESET_SENSE_FLAG(); // Clear RESET IC Event
#endif
#ifdef ENABLE_RESET_SENSE_INT
   ENABLE_RESET_SENSE_INT(); // Enable RESET IC interrupts
#endif
   
   return(BDM_RC_OK);
}
#endif //(HW_CAPABILITY&CAP_RST_IO)

//! Resets the target using BDM commands
//!
//! @note
//!    Not all targets support reset using BDM commands
//!
//! @param mode
//!    - \ref RESET_SPECIAL => Reset to special mode,
//!    - \ref RESET_NORMAL  => Reset to normal mode
//!
//!
//! @return
//!    \ref BDM_RC_OK                     => Success \n
//!    \ref BDM_RC_BKGD_TIMEOUT           => BKGD pin stuck low \n
//!    \ref BDM_RC_RESET_TIMEOUT_RISE     => RESET pin stuck low \n
//!    \ref BDM_RC_UNKNOWN_TARGET         => Don't know how to reset this type of target! \n
//!
U8 bdm_softwareReset(U8 mode) {
U8 rc;

   if (cable_status.target_type == T_HC12) { // Doesn't support s/w reset
      return BDM_RC_ILLEGAL_PARAMS; // HC12 doesn't have s/w reset
      // Doesn't return BDM_RC_ILLEGAL_COMMAND as method is controlled by a parameter
   }
   mode &= RESET_MODE_MASK;

   // Make sure of connection
   rc = bdm_connect();

   // Make sure Active background mode (in case target is stopped!)
   if (bdm_halt() != BDM_RC_OK) {
      rc = bdm_connect();
   }
   switch (cable_status.target_type) {
      case T_HCS08:
         BDM08_CMD_RESET();
         break;
      case T_RS08:
         BDMRS08_CMD_RESET();
         break;
      case T_CFV1:
         // Force reset (& set up to halt in BDM mode on various errors)
         if (mode == RESET_SPECIAL) {
            BDM_CMD_1B_0_T(_BDMCF_WRITE_CSR2_BYTE,
                  CFV1_CSR2_BDFR|CFV1_CSR2_COPHR|CFV1_CSR2_IOPHR|CFV1_CSR2_IADHR|CFV1_CSR2_BFHBR);
         }
         else {
            BDM_CMD_1B_0_T(_BDMCF_WRITE_CSR2_BYTE,
                  CFV1_CSR2_BDFR|CFV1_CSR2_COPHR|CFV1_CSR2_IOPHR|CFV1_CSR2_IADHR);
         }
         break;
      default:
         return BDM_RC_UNKNOWN_TARGET; // Don't know how to reset this one!
   }
   if (mode == RESET_SPECIAL) {  // Special mode - need BKGD held low out of reset
      BDM_LOW(); // drive BKGD low (out of reset)
   }
   enableInterrupts();
   
#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
#endif

   WAIT_MS(RESET_SETTLEms);   // Wait for target to start reset (and possibly assert reset)

#if (HW_CAPABILITY&CAP_RST_IO)
   if (bdm_option.useResetSignal) {
      // Wait with timeout until RESET is high (may be held low by processor)
      bdm_WaitForResetRise();
      // Assume RESET risen - check later after cleanup
   }
#endif // (HW_CAPABILITY&CAP_RST_IO)

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   if (mode == 0) {   // Special mode - release BKGD
      WAIT_US(BKGD_WAITus);      // Wait for BKGD assertion time after reset rise
      BDM_3STATE();
   }

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN   = 0;
   DEBUG_PIN   = 1;
#endif

   // Wait recovery time before allowing anything else to happen on the BDM
   WAIT_MS(RESET_RECOVERYms);   // Wait for Target to start up after reset

   bdmHCS_interfaceIdle();      // Place interface in idle state

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   //  bdm_halt();  // For RS08?
   return BDM_RC_OK;
}

//! Resets the target
//!
//! Resets the target using any of the following as needed:
//! - Vdd power cycle - POR,
//! - Software BDM command or
//! - Hardware reset.
//!
//! @param mode
//!    - \ref RESET_SPECIAL => Reset to special mode,
//!    - \ref RESET_NORMAL  => Reset to normal mode
//!
//! @return
//!    == \ref BDM_RC_OK  => Success \n
//!    != \ref BDM_RC_OK  => various errors
//
U8 bdm_targetReset( U8 mode ) {
U8 rc = BDM_RC_OK;

   // Power-cycle-reset - applies to all chips
   if (bdm_option.cycleVddOnReset)
      rc = bdm_cycleTargetVdd(mode);

   if (rc != BDM_RC_OK)
      return rc;

   // Software (BDM Command) reset - HCS08, RS08 & Coldfire
   rc = bdm_softwareReset(mode);

#if (HW_CAPABILITY&CAP_RST_IO)
   // Hardware (RESET pin) reset
   // HC12s and some HCS08s/RS08s & CFv1 (may not result in BDM mode) support this
   if ((rc != BDM_RC_OK) && bdm_option.useResetSignal)
      rc = bdm_hardwareReset(mode);
#endif //(HW_CAPABILITY&CAP_RST_IO)

   return rc;
}

//!  Called when target power has been externally cycled.
//!  Holds RESET & BKGD low while waiting for Vdd to rise.
//!
//! @return
//!    == \ref BDM_RC_OK              => Success               \n
//!    == \ref BDM_RC_VDD_NOT_PRESENT => Vdd failed to rise    \n
//!    == \ref BDM_RC_RESET_TIMEOUT_RISE   => Reset failed to rise  \n
//!    == \ref BDM_RC_BKGD_TIMEOUT    => BKGD failed to rise
//!
U8 bdmHCS_powerOnReset(void) {
U8 rc = BDM_RC_OK;

#if (HW_CAPABILITY&CAP_VDDSENSE)

   bdmHCS_interfaceIdle();  // Make sure BDM interface is idle

   // BKGD pin=L, RESET=L,  wait...
   BDM_LOW();
#if (HW_CAPABILITY&CAP_RST_IO)
   RESET_LOW();
#endif
   WAIT_MS(RESET_SETTLEms);

#if (HW_CAPABILITY&CAP_RST_IO)
   // Release reset
   RESET_3STATE();
#endif
   
#if (HW_CAPABILITY&CAP_RST_IO)
   // Wait for Vdd to rise within 50% of 3V and RESET to return high
   // RESET rise may be delayed by target POR
   WAIT_WITH_TIMEOUT_MS( 250 /* ms */, (bdm_targetVddMeasure()>75)&&
                                       (!bdm_option.useResetSignal)||(RESET_IN!=0));
#else   
   // Wait for Vdd to rise within 50% of 3V and RESET to return high
   // RESET rise may be delayed by target POR
   WAIT_WITH_TIMEOUT_MS( 250 /* ms */, (bdm_targetVddMeasure()>75));
#endif
   
   // Let signals settle & CPU to finish reset (with BKGD held low)
   WAIT_US(BKGD_WAITus);

   if (bdm_targetVddMeasure()<=70) // Vdd didn't turn on!
      rc = BDM_RC_VDD_NOT_PRESENT;

#if (HW_CAPABILITY&CAP_RST_IO)
   if (bdm_option.useResetSignal && (RESET_IN==0)) // RESET didn't rise
      rc = BDM_RC_RESET_TIMEOUT_RISE;
#endif
   bdmHCS_interfaceIdle();  // Make sure BDM interface is idle (BKGD now high)

   // Let signals settle
   WAIT_US(RESET_SETTLEms);

   if (BDM_IN==0) // BKGD didn't rise!
      rc = BDM_RC_BKGD_TIMEOUT;

   cable_status.reset = RESET_DETECTED;    // Record the fact that reset was asserted

#endif // (HW_CAPABILITY&CAP_VDDSENSE)
   return(rc);
}

////!  Directly set the interface levels
////!
////! @param level see \ref InterfaceLevelMasks_t
////!
////! @return
////!    == \ref BDM_RC_OK     => Success \n
////!    != \ref BDM_RC_OK     => various errors
////
//U8  bdm_setInterfaceLevel(U8 level) {
//
//   switch (level&SI_BKGD) {
//      case SI_BKGD_LOW :  // BKGD pin=L
//         BDM_LOW();
//         break;
//      case SI_BKGD_HIGH : // BKGD pin=H
//         BDM_HIGH();
//         break;
//      default :           // BKGD pin=3-state
//         BDM_3STATE();
//         break;
//   }
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
//
//#if (HW_CAPABILITY & CAP_RST_IO)
//   return (RESET_IN?SI_RESET_3STATE:SI_RESET_LOW)|(BDM_IN?SI_BKGD_3STATE:SI_BKGD_LOW);
//#else
//   return (BDM_IN?SI_BKGD:0);
//#endif
//}

//! \brief Measures the SYNC length and writes the result into cable_status structure
//!
//! Method :
//!   - One timer channel is set up for input capture on BKGD.
//!   - A second timer channel is used for timeout.
//!   - Trigger SYNC response from target by manually taking BKGD low and then pulsed briefly high.
//!   - The timer channel captures the falling edge and rising edges from the target.
//!   - SYNC duration is calculated from above times.
//!
//! @return
//!   \ref BDM_RC_OK               => Success \n
//!   \ref BDM_RC_BKGD_TIMEOUT     => BKGD pin stuck low or no response
//!
U8 bdm_syncMeasure(void) {
U16 time;

   cable_status.speed = SPEED_NO_INFO;  // Indicate that we do not have a clue about target speed at the moment...

   bdmHCS_interfaceIdle();        // Make sure BDM interface is idle

   // Wait with timeout until BKGD is high
   WAIT_WITH_TIMEOUT_MS(BDM_SYNC_REQms, (BDM_IN!=0));

   if (BDM_IN==0) {
      return(BDM_RC_BKGD_TIMEOUT); // Timeout !
   }
   BDM_LOW();
   WAIT_MS(BDM_SYNC_REQms);     // Wait SYNC request time

#if (DEBUG&SYNC_DEBUG)
   DEBUG_PIN   = 0;
   DEBUG_PIN   = 1;
#endif

   // Set up Input capture & timeout timers
   BKGD_TPMxCnSC        = BKGD_TPMxCnSC_FALLING_EDGE_MASK;  // TPMx.CHb : Input capture, falling edge on pin

   TIMEOUT_TPMxCnVALUE  = TPMCNT+TIMER_MICROSECOND(SYNC_TIMEOUTus);  // Set Timeout value
   BKGD_TPMxCnSC_CHF    = 0;                                         // TPMx.CHa : Clear capture flag
   TIMEOUT_TPMxCnSC_CHF = 0;                                         // TPMx.CHb : Clear timeout flag
   asm {
      LDX   #BDM_OUT_MASK|BDM_EN_WR_MASK  // Mask to Drive BKGD high
      LDA   #BDM_OUT_MASK|BDM_EN_RD_MASK  // Mask to 3-state BKGD
      STX   DATA_PORT                     // [3 wpp]  Drive BKGD high (for 125 ns)
      STA   DATA_PORT                     // [3 wpp]  3-state BKGD
      // It took 125 ns cycles from bringing BKGD high to 3-state (3 cycles/24 MHz)
      // Target drives BKGD line after 16 BDM clock cycles
      // Fast enough up to approx 125ns/(16 BDM cycles) = 128 MHz BDM Frequency
   }
   while ((BKGD_TPMxCnSC_CHF==0)&&(TIMEOUT_TPMxCnSC_CHF==0)) {   // Wait for capture or timeout
   }
   time          = BKGD_TPMxCnVALUE;                 // TPMx.Chx : Save time of start of the SYNC interval
   BKGD_TPMxCnSC = BKGD_TPMxCnSC_RISING_EDGE_MASK;   // TPMx.Chx : Clear IC flag, configure for IC rising edge

   // It takes 23 cycles to re-enable capture (worst case) which is good enough up to 128*24/23 = 130 MHz!
   while ((BKGD_TPMxCnSC_CHF==0)&&(TIMEOUT_TPMxCnSC_CHF==0)) {   // Wait for capture or timeout
   }
   time = BKGD_TPMxCnVALUE-time;                     // Calculate length of the SYNC pulse

#if (DEBUG&SYNC_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   if (TIMEOUT_TPMxCnSC_CHF==1) {
      return(BDM_RC_SYNC_TIMEOUT);         // Timeout !
   }
#if (TIMER_FREQ==3000000UL)
   cable_status.sync_length=(time<<2)+(time<<4);  // multiply by 20 to get the time in 60MHz ticks
#elif (TIMER_FREQ==6000000UL)
   cable_status.sync_length=(time<<1)+(time<<3);  // multiply by 10 to get the time in 60MHz ticks
#elif (TIMER_FREQ==24000000UL)
   cable_status.sync_length=(time<<1)+(time>>1);  // multiply by 2.5 to get the time in 60MHz ticks
#elif (TIMER_FREQ==30000000UL)
   cable_status.sync_length=(time<<1);            // multiply by 2 to get the time in 60MHz ticks
#elif (TIMER_FREQ==15000000UL)
   cable_status.sync_length=(time<<2);            // multiply by 4 to get the time in 60MHz ticks
#else
   #error "Please fix SYNC length calculation for new Bus Frequency"
#endif
   cable_status.speed = SPEED_SYNC;           // SYNC feature is supported by the target
   return(0);
}

//! Enables ACKN and prepares the timer for easy ACKN timeout use
//!
void bdm_acknInit(void) {
U8 rc;

   // Set up Input capture & timeout timers
   BKGD_TPMxCnSC = BKGD_TPMxCnSC_RISING_EDGE_MASK;  // TPMx.CHb : Input capture, falling edge on pin

#if (DEBUG&ACK_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
   DEBUG_PIN     = 0;
#endif

   cable_status.ackn = ACKN;              // Switch ACKN on

   // Send the ACK enable command to the target
   if ((cable_status.target_type==T_CFV1)
#if TARGET_CAPABILITY & CAP_S12Z
		   ||(cable_status.target_type==T_HCS12Z  )
#endif	   
   )
      rc = BDMCF_CMD_ACK_ENABLE();
   else
      rc = BDM_CMD_ACK_ENABLE();

   // If ACKN fails turn off ACKN (RS08 or early HCS12 target)
   if (rc == BDM_RC_ACK_TIMEOUT) {
      cable_status.ackn = WAIT;  // Switch the ackn feature off
   }
}

//! Wait for 64 target clock cycles
//!
void bdm_wait64() {
   asm {
		 cli
		 ldhx  cable_status.wait64_cnt    // Number of loop iterations to wait
	  loop:
		 aix   #-1      ; [2]
		 cphx  #0       ; [3]
		 bne   loop     ; [3] 8 cycles / iteration
   }
}
//! Wait for 150 target clock cycles
//!
void bdm_wait150() {
   asm {
		 cli
		 ldhx  cable_status.wait150_cnt    // Number of loop iterations to wait
	  loop:
		 aix   #-1      ; [2]
		 cphx  #0       ; [3]
		 bne   loop     ; [3] 8 cycles / iteration
   }
}
#if 0
//! Depending on ACKN mode this function:       \n
//!   - Waits for ACKN pulse with timeout.
//!      OR
//!   - Busy waits for 64 target CPU clocks
//!
//! @return
//!   \ref BDM_RC_OK           => Success \n
//!   \ref BDM_RC_ACK_TIMEOUT  => No ACKN detected [timeout]
//!
U8 doACKN_WAIT64(void) {
   if (cable_status.ackn==ACKN) {
      TIMEOUT_TPMxCnVALUE  = TPMCNT+TIMER_MICROSECOND(ACKN_TIMEOUTus);  // Set ACKN Timeout value
      TIMEOUT_TPMxCnSC_CHF = 0;                                         // TPMx.CHb : Clear timeout flag
      //enableInterrupts(); //xxx
      // Wait for pin capture or timeout
      while ((BKGD_TPMxCnSC_CHF==0)&&(TIMEOUT_TPMxCnSC_CHF==0)) {
      }
      if (BKGD_TPMxCnSC_CHF==0) {    // Timeout - Changed as USB may delay this
         return BDM_RC_ACK_TIMEOUT;  //   Return timeout error
      }
   }
   else {
      bdm_wait64();
   }
   return BDM_RC_OK;
}

//! Depending on ACKN mode this function:       \n
//!   - Waits for ACKN pulse with timeout.
//!      OR
//!   - Busy waits for 150 target CPU clocks
//!
//! @return
//!   \ref BDM_RC_OK           => Success \n
//!   \ref BDM_RC_ACK_TIMEOUT  => No ACKN detected [timeout]
//!
U8 doACKN_WAIT150(void) {
   if (cable_status.ackn==ACKN) {
      TIMEOUT_TPMxCnVALUE  = TPMCNT+TIMER_MICROSECOND(ACKN_TIMEOUTus);  // Set ACKN Timeout value
      TIMEOUT_TPMxCnSC_CHF = 0;                                         // TPMx.CHb : Clear timeout flag
      //enableInterrupts(); //xxx
      // Wait for pin capture or timeout
      while ((BKGD_TPMxCnSC_CHF==0)&&(TIMEOUT_TPMxCnSC_CHF==0)) {
      }
      if (BKGD_TPMxCnSC_CHF==0) {    // Timeout - Changed as USB may delay this
         return BDM_RC_ACK_TIMEOUT;  //   Return timeout error
      }
   }
   else asm {
      // Wait for 150 target clock cycles
         cli
         ldhx  cable_status.wait150_cnt    // Number of loop iterations to wait
      loop:
         aix   #-1      ; [2]
         cphx  #0       ; [3]
         bne   loop     ; [3] 8 cycles / iteration
   }
   return BDM_RC_OK;
}
#endif

//! Depending on ACKN mode this function:       \n
//!   - Waits for ACKN pulse with timeout.
//!      OR
//!   - Busy waits for 64 target CPU clocks
//!
//! @return
//!   \ref BDM_RC_OK           => Success \n
//!   \ref BDM_RC_ACK_TIMEOUT  => No ACKN detected [timeout]
//!
//! @note  Modified to extend timeout for very slow bus clocks e.g. 32kHz
U8 doACKN_WAIT64(void) {
   if (cable_status.ackn==ACKN) {
      // Wait for pin capture or timeout
	  enableInterrupts();
      WAIT_WITH_TIMEOUT_US(ACKN_TIMEOUTus, (BKGD_TPMxCnSC_CHF!=0));
	  if (BKGD_TPMxCnSC_CHF==0) {
		 return BDM_RC_ACK_TIMEOUT;  //   Return timeout error
	  }
   }
   else {
      bdm_wait64();
   }
   return BDM_RC_OK;
}

//! Depending on ACKN mode this function:       \n
//!   - Waits for ACKN pulse with timeout.
//!      OR
//!   - Busy waits for 150 target CPU clocks
//!
//! @return
//!   \ref BDM_RC_OK           => Success \n
//!   \ref BDM_RC_ACK_TIMEOUT  => No ACKN detected [timeout]
//!
U8 doACKN_WAIT150(void) {
   if (cable_status.ackn==ACKN) {
      // Wait for pin capture or timeout
      enableInterrupts();
      WAIT_WITH_TIMEOUT_US(ACKN_TIMEOUTus, (BKGD_TPMxCnSC_CHF!=0));
      if (BKGD_TPMxCnSC_CHF==0) {
         return BDM_RC_ACK_TIMEOUT;  //   Return timeout error
      }
   }
   else {
      bdm_wait150();
   }
   return BDM_RC_OK;
}

//!  Halts the processor - places in background mode
//!
U8 bdm_halt(void) {

   if ((cable_status.target_type==T_CFV1)
#if TARGET_CAPABILITY & CAP_S12Z
	  ||(cable_status.target_type==T_HCS12Z  )
#endif
	  )
      return BDMCF_CMD_BACKGROUND();
   else
      return BDM_CMD_BACKGROUND();
}

//! Commences full-speed execution on the target
//!
U8 bdm_go(void) {
U32 csr;

   if (cable_status.target_type == T_CFV1) {
      // Clear Single-step mode
      (void)BDMCF_CMD_READ_DREG(CFV1_CSR, &csr);
      csr &= ~CFV1_CSR_SSM;
#ifdef MC51AC256_HACK
      csr |= CFV1_CSR_VBD; // Hack for MC51AC256 - turn off Bus visibility
#endif
      (void)BDMCF_CMD_WRITE_DREG(CFV1_CSR,csr);

      return BDMCF_CMD_GO();
      }
   else {
      return BDM_CMD_GO();
   }
}

//!  Executes a single instruction on the target
//!
U8 bdm_step(void) {
U32 csr;

   if (cable_status.target_type == T_CFV1) {
      // Set Single-step mode
      (void)BDMCF_CMD_READ_DREG(CFV1_CSR, &csr);
      csr |= CFV1_CSR_SSM;
#ifdef MC51AC256_HACK
      csr |= CFV1_CSR_VBD; // Hack for MC51AC256 - turn off Bus visibility
#endif
      (void)BDMCF_CMD_WRITE_DREG(CFV1_CSR, csr);

      return BDMCF_CMD_GO();
      }
#if TARGET_CAPABILITY & CAP_S12Z
   else if (cable_status.target_type == T_HCS12Z  ) {
      return BDMZ12_CMD_TRACE1();
   }
#endif
   else {
      return BDM_CMD_TRACE1();
   }
}

//!  Turns off the BDM interface
//!
//!  Depending upon settings, may leave target power on.
//!
void bdmHCS_off( void ) {
#if ((HW_CAPABILITY & CAP_FLASH) != 0)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif
   if (!bdm_option.leaveTargetPowered) {
      VDD_OFF();
   }
   bdmHCS_interfaceIdle();
}

//!  Sets the BDM interface to a suspended state
//!
//!  - All signals undriven \n
//!  - All voltages off.
//!
void bdmHCS_suspend(void) {
   bdmHCS_off();
   VDD_OFF();
}

//!  Sets the BDM hardware interface to an idle condition
//!
//!  BKGD   signal undriven, PUP \n
//!  RESET  signal undriven, PUP \n
//!  VPP,Flash voltage etc unchanged. \n
//!
//! @note Assumes that \ref bdmHCS_init{} has been previously called at least once.
//!
void bdmHCS_interfaceIdle(void) {
#if (HW_CAPABILITY&CAP_RST_IO)
   RESET_3STATE();
#endif   
   BDM_3STATE();

//   - BKGD & RESET pins on BDM cable are 3-state and may be read through BDM_IN & RESET_IN
//   - BDM_OUT is 3-state
//   - RESET_OUT is undriven (but has PUP so is high) (needed if O/C buffer or FET is used as driver on RESET pin)
//   - RESET_DIR is inactive (high or low as required) i.e. external 3-state buffer/transceiver is disabled/input.
}

//!  Initialises the BDM hardware interface
//!
//!  Initialises required PUPs on port pins
//!  BKGD   signal undriven, PUP \n
//!  RESET  signal undriven, PUP \n
//!  VPP,Flash voltage etc unchanged. \n
//!
void bdmHCS_init(void) {

   // Enable pull-ups on I/O lines (RST_IO & BKGD have double PUPs)
#ifdef DATA_PORT_PEBIT
   // A single bit used to control PUPs on entire port
   DATA_PORT_PEBIT = 1;
#else
   // Individually controlled PUPs
   BDM_OUT_PER     = 1;     // Prevent drive
   BDM_EN_PER      = 1;     // Keep driver idle
#ifdef RESET_IN_PER
   RESET_IN_PER    = 1;     // Needed for input level translation to 5V
#endif
#ifdef RESET_OUT_PER
   RESET_OUT_PER   = 1;     // Holds RESET_OUT inactive when unused
#endif
#endif
   bdmHCS_interfaceIdle();
}

//============================================================
// Masks to modify BKGD
#define BKGD_LOW_MASK    (BDM_EN_WR_MASK|     0      )
#define BKGD_HIGH_MASK   (BDM_EN_WR_MASK|BDM_OUT_MASK)
#define BKGD_3STATE_MASK (BDM_EN_RD_MASK|     0      )

//! Prepares for transmission of BDM data
//!
//! Interrupts need to be disabled for the duration of all BDM commands. \n
//! It is up to the caller to see to this.
//!
void bdm_txPrepare(void) {
   disableInterrupts();

   BDM_HIGH(); // BKGD enabled & driven high 

   // BKGD is now controlled via BDM_OUT (DATA_PORT.x) with 3-state control on BDM_EN (DATA_PORT.y).
   // Other bits of DATA_PORT are configured as inputs and have no effect when written.
   // DATA_PORT = BDM_EN_WR_MASK|0            => BKGD driven low
   // DATA_PORT = BDM_EN_WR_MASK|BDM_OUT_MASK => BKGD driven high
   // DATA_PORT = BDM_EN_RD_MASK|0            => BKGD 3-state
   // DATA_PORT = BDM_EN_RD_MASK|BDM_OUT_MASK => BKGD 3-state (not used ?)
  
   //    Macros used to enable/disable BKGD drive
   //      BDM_ENABLE_ASM/BDM_ENABLE()            // Enable BKGD
   //      BDM_3STATE_ASM/BDM_3STATE()            // 3-state BKGD
}

//==============================================================
// Tx Routines bdm_tx..
//==============================================================
// Transmit 8 bits of data, MSB first
//
// Leaves BDM_OUT 3-state via BDM_DIR
//

//=========================================================================
#pragma MESSAGE DISABLE C5703 // Disable warnings about unused parameter
//! Dummy BDM Tx routine
//!
void bdm_txEmpty(U8 data) {
   // If BDM command is executed with this routine set command failed...
//   commandBuffer[0] = BDM_RC_NO_CONNECTION;
}

//=========================================================================
// 3,4,14/15
void bdm_tx1(U8 data) {
   asm {
//      BDM_ENABLE_ASM                          // Enable BKGD (high)
      SEC                                     // Set sentinel for 1st ROLA
      BRA   Entry
      
   Loop:
      BCS   DoBit                             // [3   ppp]  data!=0? - OK skip
      LDX   #BKGD_LOW_MASK                    // [2    pp]  Make data bit=0
   DoBit:
      MOV   #BKGD_LOW_MASK,DATA_PORT          // [4  pwpp]  Drive BKGD low
                                              // --- 15/16
      STX   DATA_PORT                         // [3   wpp]  Drive BKGD data
                                              // --- 3
      MOV   #BKGD_HIGH_MASK,DATA_PORT         // [4  pwpp]  Drive BKGD high
                                              // --- 4
      CLC                                     // [1     p]  Clear sentinel for ROLA
   Entry:
      LDX   #BKGD_HIGH_MASK                   // [2    pp]  Assume data bit=1
      ROLA                                    // [1     p]  Test data
      BNE   Loop                              // [3   ppp]  Done? - exit

      BDM_3STATE_ASM                          // [5      ]   3-state BKGD
      BKGD_TPM_SETUP_ASM                      // [5 rfwpp]   Set up for ACKN
      // ACKN is ready within about 10 bus cycles <500ns
      // Required to be ready 32+16 BDC cycles
      // Adequate for 100MHz+ !
      }
}

//=========================================================================
// 5,6,14/15
void bdm_tx2(U8 data) {
   asm {
//      BDM_ENABLE_ASM                          // Enable BKGD (high)
      SEC                                     // Set sentinel for 1st ROLA
      BRA   Entry
      
   Loop:
      BCS   DoBit                             // [3   ppp]  data!=0? - OK skip
      LDX   #BDM_EN_WR_MASK                   // [2    pp]  Make data bit=0
   DoBit:
      MOV   #BDM_EN_WR_MASK,DATA_PORT         // [4  pwpp]  Drive BKGD low
                                              // --- 15/16
      NOP                                     // [1     p]
      NOP                                     // [1     p]
      STX   DATA_PORT                         // [3   wpp]  Drive BKGD data
                                              // --- 5
      NOP                                     // [1     p]
      NOP                                     // [1     p]
      MOV   #BKGD_HIGH_MASK,DATA_PORT         // [4  pwpp]  Drive BKGD high
                                              // --- 6
      CLC                                     // [1     p]  Clear sentinel for ROLA
   Entry:
      LDX   #BKGD_HIGH_MASK                   // [2    pp]  Assume data bit=1
      ROLA                                    // [1     p]  Test data
      BNE   Loop                              // [3   ppp]  Done? - exit

      BDM_3STATE_ASM                          // [5      ]   3-state BKGD
      BKGD_TPM_SETUP_ASM                      // [5 rfwpp]   Set up for ACKN
      // ACKN is ready within about 10 bus cycles <500ns
      // Required to be ready 32+16 BDC cycles
      // Adequate for 100MHz+ !
      }
}

//=========================================================================
// 7,8,14/15
void bdm_tx3(U8 data) {
   asm {
//    BDM_ENABLE_ASM                          // Enable BKGD (high)
      SEC                                     // Set sentinel for 1st ROLA
      BRA   Entry
      
   Loop:
      BCS   DoBit                             // [3   ppp]  data!=0? - OK skip
      LDX   #BDM_EN_WR_MASK                   // [2    pp]  Make data bit=0
   DoBit:
      MOV   #BDM_EN_WR_MASK,DATA_PORT         // [4  pwpp]  Drive BKGD low
                                              // --- 15/16
      NOP                                     // [1     p]
      BRN   *+0                               // [3   ppp]
      STX   DATA_PORT                         // [3   wpp]  Drive BKGD data
                                              // --- 7
      NOP                                     // [1     p]
      BRN   *+0                               // [3   ppp]
      MOV   #BKGD_HIGH_MASK,DATA_PORT         // [4  pwpp]  Drive BKGD high
                                              // --- 8
      CLC                                     // [1     p]  Clear sentinel for ROLA
   Entry:
      LDX   #BKGD_HIGH_MASK                   // [2    pp]  Assume data bit=1
      ROLA                                    // [1     p]  Test data
      BNE   Loop                              // [3   ppp]  Done? - exit

      BDM_3STATE_ASM                          // [5      ]   3-state BKGD
      BKGD_TPM_SETUP_ASM                      // [5 rfwpp]   Set up for ACKN
      }
}

//=========================================================================
// >=10,>=11,>=24/25
//! Generic BDM Tx routine - used for a range of speeds
//!
void bdm_txGeneric(U8 data) {
   asm {
//   BDM_ENABLE_ASM       // Enable BKGD (high)
     SEC                  // Set sentinel for 1st ROLA
     BRA   Entry
     
   Loop:
      PSHA                                    // [2      ]
      BCS   DoBit                             // [3   ppp]  data!=0? - OK skip
      LDX   #BDM_EN_WR_MASK                   // [2    pp]  Make data bit=0
   DoBit:
      LDA   txTiming3                         // [3      ]
      DBNZA *+0                               // [4n     ]
      MOV   #BDM_EN_WR_MASK,DATA_PORT         // [4  pwpp]  Drive BKGD low
                                              // --- 21/22+4n   (>=23)
      LDA   txTiming1                         // [3      ]
      DBNZA *+0                               // [4n     ]
      STX   DATA_PORT                         // [3   wpp]   Drive data to BDM
                                              // ---  6+4n   (>=10)
      LDA   txTiming2                         // [3      ]
      DBNZA *+0                               // [4n     ]
      MOV   #BKGD_HIGH_MASK,DATA_PORT         // [4  pwpp]  Drive BKGD high
                                              // ---  7+4n   (>=11)
      PULA                                    // [3      ]
      CLC                                     // [1      ]  For ROLA
   Entry:
      LDX   #BKGD_HIGH_MASK                   // [2    pp]  Assume data bit=1
      ROLA                                    // [1     p]  Test data
      BNE   Loop                              // [3   ppp]  Done? - exit
     
                                              // ===========
      BDM_3STATE_ASM                          // [5      ]   3-state BKGD
      BKGD_TPM_SETUP_ASM                      // [5 rfwpp]   Set up for ACKN
      }
}
#if 0
//=========================================================================
// >=10,>=12,>=18
//! Generic BDM Tx routine - used for a range of speeds
//!
void bdm_txGeneric(U8 data) {
   asm {
      BDM_ENABLE_ASM                // Enable BKGD (high)
      MOV  #8,bitCount              // # of bits to send

   Loop:
      LDX   txTiming3               // [3      ]
      DBNZX *+0                     // [4n     ]
      MOV   #0,DATA_PORT            // [4  pwpp]   Drive BKGD low
                                    // --- 14+4n   (>=18)
      LDX   txTiming1               // [3      ]
      DBNZX *+0                     // [4n     ]
      STA   DATA_PORT               // [3   wpp]   Drive data to BDM
                                    // ---  6+4n   (>=10)
      LDX   txTiming2               // [3      ]
      DBNZX *+0                     // [4n     ]
      LSLA                          // [1      ]   Next bit
      MOV   #BDM_OUT_MASK,DATA_PORT // [4  pwpp]   Drive BKGD high
                                    // ---  8+4n   (>=12)
      DBNZ  bitCount,Loop           // [7      ]
                                    // ===========
      BDM_3STATE_ASM                // [5      ]   3-state BKGD
      BKGD_TPM_SETUP_ASM            // [5 rfwpp]   Set up for ACKN
      }
}

#endif
#pragma MESSAGE DEFAULT C5703 // Restore warnings about unused parameter

//==============================================================
//  Rx Routines bdm_rx..
//==============================================================
//  Receive 8 bit of data, MSB first
//  These routines assume the following:
//     BDM direction to be controlled by BDM_DIR_Rx in DATA_PORT
//     BDM output value may be controlled by DATA_PORT.7 hard coded in bdm_rxN
//     BDM input value may be read from DATA_PORT.0 hard coded in rxStackDecode
//     The rest of PTA must be disabled - configured as inputs
//  Leaves BDM_OUT 3-state via BDM_DIR_Rx
//  Note: These routines have been re-timed to allow for the phase difference b/w read and write instructions.

#pragma MESSAGE DISABLE C20001 // Disable warnings about stackpointer

//!  Decodes values recorded by RX functions
//!
//!  Expects LSB data in X and remaining 7 bytes on stack. \n
//!  It is expected that caller will JUMP into this routine.
//!
static void rxStackDecode(void) {
   asm {
      MOV   #8,bitCount       // # of bits
decode:
#if (BDM_IN_BIT >= 4)         // Rotate left
#if (BDM_IN_BIT <= 4)
      ROLX                    // Get the interesting bit into C
#endif
#if (BDM_IN_BIT <= 5)
      ROLX
#endif
#if (BDM_IN_BIT <= 6)
      ROLX
#endif
      ROLX
#else // Rotate right
#if (BDM_IN_BIT >= 3)
      RORX                     // Get the interesting bit into C
#endif
#if (BDM_IN_BIT >= 2)
      RORX
#endif
#if (BDM_IN_BIT >= 1)
      RORX
#endif
      RORX
#endif
      RORA                     // and rotate it into A from the top
      PULX                     // get the next value from stack
      DBNZ  bitCount,decode
      PSHX                     // that was one pop too many, so push the value back
   }
}
#pragma MESSAGE DEFAULT C20001 // Restore warnings about stackpointer

//! Dummy BDM Rx routine
//!
//! When no function appropriate for the target speed can be found the following
//! routine is selected.  This is just to make things safe and to make sure
//! there is a place to jump to in such a case
//!
U8 bdm_rxEmpty( void ) {
   // if BDM command is executed with this routine set command failed...
//   commandBuffer[0] = BDM_RC_NO_CONNECTION;
   return(0);
}

#pragma MESSAGE DISABLE C1404  // Disable warnings about no return statement

#if (HW_CAPABILITY&CAP_CFVx_HW) && 0 // CF Version in JM16 pressed for space - Smaller code
//=======================================================================
// 3,5,15
#pragma MESSAGE DISABLE C20001 // Disable warnings about different stack ptr
U8 bdm_rx1(void) {
#pragma NO_RETURN
   asm {
      SEI
      LDHX  @DATA_PORT           // Point HX at DATA_PORT
      MOV   #7,bitCount          // [3  pwp]   Drive BKGD low

      LDA   #BKGD_3STATE_MASK    // Writing A to DATA_PORT 3-states BKGD
      STA   ,X                   // 3-state BKGD pin

   Loop:
      /* bit 7..1 */
      MOV   #BKGD_LOW_MASK,DATA_PORT // [3  pwp]   Drive BKGD low
                                 // ------- 15
                                 // [1+   p]
      STA   ,X                   // [2   wp]   3-state BKGD pin
                                 // ------- 3
      BRN   *+2                  // [3     ]
      LDA   ,X                   // [2   rf]   Sample BDM_IN
                                 // ------- 5
                                 // [1+   p]
      PSHA                       // [2     ]   Save sample on stack
      LDA   #BKGD_3STATE_MASK    // [2     ]   Restore A value
      DBNZ  bitCount,Loop        // [7     ]

      /* bit 0 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      NOP
      LDX   ,X                   // Last sample in X

      //CLI
      // now get the bit values (last value is in X, previous 7 on stack)
      JMP   rxStackDecode
   }
}
#pragma MESSAGE DEFAULT C20001 // Restore warnings about different stack ptr
                       
#else

//=======================================================================
// 3,5,8
U8 bdm_rx1(void) {
#pragma NO_RETURN
   asm {
      SEI
      LDHX  @DATA_PORT                // Point HX at DATA_PORT

      LDA   #BKGD_3STATE_MASK         // Writing A to DATA_PORT 3-states BDM
      STA   ,X                        // 3-state BKGD pin

      /* bit 7 (MSB) */
      MOV   #BKGD_LOW_MASK,DATA_PORT  // [3  pwp]   Drive BKGD low
                                      // ------- 8
                                      // [+1   p]
      STA   ,X                        // [2   wp]   3-state BKGD pin
                                      // ------- 3
      BRN   *+2                       // [3     ]
      LDA   ,X                        // [2   rf]   Sample BDM_IN
                                      // ------- 5
                                      // [1+   p]
      PSHA                            // [2     ]   Save sample on stack
      LDA   #BKGD_3STATE_MASK         // [2     ]   Restore A value

      /* bit 6 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 5 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 4 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 3 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 2 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 1 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 0 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      LDX   ,X

      //CLI
      // now get the bit values (last value is in X, previous 7 on stack)
      JMP   rxStackDecode
   }
}
#endif

#if (HW_CAPABILITY&CAP_CFVx_HW) && 0 // CF Version in JM16 pressed for space - Smaller code
//=======================================================================
// 3,6,15
#pragma MESSAGE DISABLE C20001 // Disable warnings about different stack ptr
U8 bdm_rx2(void) {
#pragma NO_RETURN
   asm {
      SEI
      LDHX  @DATA_PORT           // Point HX at DATA_PORT
      MOV   #7,bitCount          // [3  pwp]   Drive BKGD low

      LDA   #BKGD_3STATE_MASK    // Writing A to DATA_PORT 3-states BKGD
      STA   ,X                   // 3-state BKGD pin

   Loop:
      /* bit 7..1 */
      MOV   #BKGD_LOW_MASK,DATA_PORT // [3  pwp]   Drive BKGD low
                                 // ------- 15
                                 // [1+   p]
      STA   ,X                   // [2   wp]   3-state BKGD pin
                                 // ------- 3
      BRN   *+2                  // [3     ]
      NOP                        // [1     ]
      LDA   ,X                   // [2   rf]   Sample BDM_IN
                                 // ------- 6
                                 // [1+   p]
      PSHA                       // [2     ]   Save sample on stack
      LDA   #BKGD_3STATE_MASK    // [2     ]   Restore A value
      DBNZ  bitCount,Loop        // [7     ]

      /* bit 0 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      NOP
      LDX   ,X                   // Last sample in X

      //CLI
      // now get the bit values (last value is in X, previous 7 on stack)
      JMP   rxStackDecode
   }
}
#pragma MESSAGE DEFAULT C20001 // Restore warnings about different stack ptr
#else

//=======================================================================
// 3,6,8
U8 bdm_rx2(void) {
#pragma NO_RETURN
   asm {
      SEI
      LDHX  @DATA_PORT               // Point HX at DATA_PORT

      LDA   #BKGD_3STATE_MASK        // Writing A to DATA_PORT 3-states BKGD
      STA   ,X                       // 3-state BKGD pin

      /* bit 7 (MSB) */
      MOV   #BKGD_LOW_MASK,DATA_PORT // [3  pwp]   Drive BKGD low
                                     // ------- 8
                                     // [1+   p]
      STA   ,X                       // [2   wp]   3-state BKGD pin
                                     // ------- 3
      BRN   *+2                      // [3     ]
      NOP                            // [1     ]
      LDA   ,X                       // [2   rf]   Sample BDM_IN
                                     // ------- 6
                                     // [1+   p]
      PSHA                           // [2     ]   Save sample on stack
      LDA   #BKGD_3STATE_MASK        // [2     ]   Restore A value

      /* bit 6 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      NOP
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 5 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      NOP
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 4 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      NOP
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 3 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      NOP
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 2 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      NOP
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 1 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      NOP
      LDA   ,X
      PSHA
      LDA   #BKGD_3STATE_MASK

      /* bit 0 */
      MOV   #BKGD_LOW_MASK,DATA_PORT
      STA   ,X
      BRN   *+2
      NOP
      LDX   ,X

      //CLI
      // now get the bit values (last value is in X, previous 7 on stack)
      JMP   rxStackDecode
   }
}
#endif

//=======================================================================
// 4,6,10
U8 bdm_rx3(void) {
   asm {
      SEI
      LDA   #0x01                       // Value used as sentinel

   Loop:
      MOV   #BKGD_LOW_MASK,DATA_PORT    // [4  pwpp]   Drive BKGD low
                                        // --- 10
      MOV   #BKGD_3STATE_MASK,DATA_PORT // [4  pwpp]   3-state BKGD pin
                                        // ---  4
      BRN   *+2                         // [3     ]
      BRSET BDM_IN_BIT,DATA_PORT,*+3    // [3   rpp]   Sample BDM_IN
                                        // ---  6
                                        // [2+   pp]
      ROLA                              // [1      ]   Save sample
      BCC   Loop                        // [3      ]
      //CLI
   }
}

//=======================================================================
// 5,6,10
U8 bdm_rx4(void) {
   asm {
      SEI
      LDA   #0x01                        // Value used as sentinel

   Loop:
      MOV   #BKGD_LOW_MASK,DATA_PORT     // [4  pwpp]   Drive BKGD low
                                         // --- 10
      NOP                                // [1     ]
      MOV   #BKGD_3STATE_MASK,DATA_PORT  // [4  pwpp]   3-state BKGD pin
                                         // ---  5
      BRN   *+2                          // [3     ]
      BRSET BDM_IN_BIT,DATA_PORT,*+3     // [3   rpp]   Sample BDM_IN
                                         // ---  6
                                         // [2+   pp]
      ROLA                               // [1      ]   Save sample
      BCC   Loop                         // [3      ]
      //CLI
   }
}

//=======================================================================
// 5,8,10
U8 bdm_rx5(void) {
   asm {
      SEI
      LDA   #0x01                        // Value used as sentinel

   Loop:
      MOV   #BKGD_LOW_MASK,DATA_PORT     // [4  pwpp]   Drive BKGD low
                                         // --- 10
      NOP                                // [1     ]
      MOV   #BKGD_3STATE_MASK,DATA_PORT  // [4  pwpp]   3-state BKGD pin
                                         // ---  5
      BRN   *+2                          // [3     ]
      NOP                                // [1     ]
      NOP                                // [1     ]
      BRSET BDM_IN_BIT,DATA_PORT,*+3     // [3   rpp]   Sample BDM_IN
                                         // ---  8
                                         // [2+   pp]
      ROLA                               // [1      ]   Save sample
      BCC   Loop                         // [3      ]
      //CLI
   }
}

//=======================================================================
// 6,9,10
U8 bdm_rx6(void) {
   asm {
      SEI
      LDA   #0x01                        // Value used as sentinel

   Loop:
      MOV   #BKGD_LOW_MASK,DATA_PORT     // [4  pwpp]   Drive BKGD low
                                         // --- 10
      NOP                                // [1     ]
      NOP                                // [1     ]
      MOV   #BKGD_3STATE_MASK,DATA_PORT  // [4  pwpp]   3-state BKGD pin
                                         // ---  6
      BRN   *+2                          // [3     ]
      BRN   *+2                          // [3     ]
      BRSET BDM_IN_BIT,DATA_PORT,*+3     // [3   rpp]   Sample BDM_IN
                                         // ---  9
                                         // [2+   pp]
      ROLA                               // [1      ]   Save sample
      BCC   Loop                         // [3      ]
      //CLI
   }
}

//=======================================================================
// >8,>10,>20
//! Generic BDM Rx routine - used for a range of speeds
//!
U8 bdm_rxGeneric(void) {
   asm {
      SEI
      LDA   #0x01                        // Value used as sentinel

   Loop:
      LDX   rxTiming1                    // [3      ]
      MOV   #BKGD_LOW_MASK,DATA_PORT     // [4  pwpp]   Drive BKGD low
                                         // ---  16+4n
      DBNZX *+0                          // [4n     ]
      MOV   #BKGD_3STATE_MASK,DATA_PORT  // [4  pwpp]   3-state BKGD pin
                                         // ---  4+4n
      LDX   rxTiming2                    // [3      ]
      DBNZX *+0                          // [4n     ]
      BRSET BDM_IN_BIT,DATA_PORT,*+3     // [3   rpp]   Sample BDM_IN
                                         // ---  6+4n
                                         // [2+   pp]
      LDX   rxTiming3                    // [3      ]
      DBNZX *+0                          // [4n     ]
      ROLA                               // [1      ]   Save sample
      BCC   Loop                         // [3      ]
      //CLI
   }
}

#pragma MESSAGE DEFAULT C1404  // Restore warnings about no return statement

//===============================================================================
//  When the SYNC length expressed in 60MHz ticks is ABOVE OR EQUAL to the value
//  in the table, the corresponding pointer is selected
//  If SYNC is shorter than the second entry, the target runs too fast
//  If SYNC is longer or equal to the last entry, the target runs too slow
//
//! Structure describing Tx configuration
typedef struct {
   U16   syncThreshold;       //!< Threshold to use this function
//   void  (*txFunc)(U8 data);  //!< Ptr to selected function
   U8    time1,time2,time3;   //!< Timing Parameters for function use
} TxConfiguration;

typedef void (*TxConfigurationPtrs)(U8 data);     //!< Ptr to selected function

const TxConfigurationPtrs txPtrs[] = {
	bdm_txEmpty,
	bdm_tx1,
	bdm_tx2,
	bdm_tx3,
};
//! Information for each Tx configuration
//!
const TxConfiguration txConfiguration[] =
{
{ 0,  0, 0, 0 },//>68 MHz - Max Fequency
{ 113,  1, 0, 0 },//37.71 - 68 MHz, (3,4,15)
{ 196,  2, 0, 0 },//24 - 40.8 MHz, (5,6,15)
{ 290,  3, 0, 0 },//17.6 - 29.14 MHz, (7,8,15)
{ 405,  1, 1, 1 },//12.57 - 20.4 MHz, (10,11,25)
{ 567,  2, 2, 1 },//9.1 - 14.57 MHz, (14,15,25)
{ 779,  3, 5, 1 },//5.87 - 10.67 MHz, (18,27,25)
{ 1216,  6, 7, 1 },//4.27 - 6.8 MHz, (30,35,25)
{ 1686,  9, 11, 7 },//2.84 - 4.86 MHz, (42,51,49)
{ 2344,  12, 12, 9 },//2.42 - 3.78 MHz, (54,55,57)
{ 2631,  13, 17, 11 },//1.98 - 3.52 MHz, (58,75,65)
{ 3285,  17, 22, 16 },//1.56 - 2.76 MHz, (74,95,85)
{ 4348,  24, 26, 19 },//1.24 - 2 MHz, (102,111,97)
{ 5244,  28, 30, 24 },//1.08 - 1.73 MHz, (118,127,117)
{ 6337,  36, 40, 34 },//0.83 - 1.36 MHz, (150,167,157)
{ 7966,  44, 47, 40 },//0.7 - 1.12 MHz, (182,195,181)
{ 9418,  52, 55, 47 },//0.6 - 0.95 MHz, (214,227,209)
{ 10949,  61, 65, 56 },//0.51 - 0.82 MHz, (250,267,245)
{ 12854,  71, 78, 67 },//0.43 - 0.7 MHz, (290,319,289)
{ 15120,  84, 93, 79 },//0.37 - 0.6 MHz, (342,379,337)
{ 17680,  98, 105, 92 },//0.32 - 0.51 MHz, (398,427,389)
{ 20239,  113, 120, 105 },//0.28 - 0.45 MHz, (458,487,441)
{ 23241,  128, 140, 120 },//0.24 - 0.39 MHz, (518,567,501)
{ 26499,  146, 166, 140 },//0.21 - 0.35 MHz, (590,671,581)
{ 36571,  0, 0 ,0  },//<0.21 MHz - Min. Frequency
};

//! Structure describing Rx configuration
typedef struct {
   U16   syncThreshold;       //!< Threshold to use this function
//   U8    txFunc;              //!< Ptr to selected function
   U8    time1,time2,time3;   //!< Timing Parameters for function use
} RxConfiguration;

typedef U8    (*RxConfigurationPtrs)(void);     //!< Ptr to selected function

const RxConfigurationPtrs rxPtrs[] = {
	bdm_rxEmpty,
	bdm_rx1,
	bdm_rx2,
	bdm_rx3,
	bdm_rx4,
	bdm_rx5,
	bdm_rx6,
};

//! Information for each Rx configuration
//!
const RxConfiguration rxConfiguration[] =
{
{ 0,  0, 0, 0 },//>56 MHz - Max Fequency
{ 137,  1, 0, 0 },//48.56 - 56 MHz, (3,5,8)
{ 152,  2, 0, 0 },//39.65 - 52.86 MHz, (3,6,8)
{ 188,  3, 0, 0 },//33.5 - 42 MHz, (4,6,10)
{ 229,  4, 0, 0 },//29 - 33.6 MHz, (5,6,10)
{ 258,  5, 0, 0 },//22.86 - 30.48 MHz, (5,8,10)
{ 320,  6, 0, 0 },//18.87 - 25.16 MHz, (6,9,10)
{ 396,  1, 1, 1 },//14.95 - 19.93 MHz, (8,10,20)
{ 503,  1, 2, 1 },//11.71 - 15.61 MHz, (8,14,20)
{ 627,  1, 3, 1 },//9.62 - 12.83 MHz, (8,18,20)
{ 750,  2, 3, 1 },//8.17 - 10.89 MHz, (12,18,20)
{ 874,  2, 4, 2 },//7.09 - 9.46 MHz, (12,22,24)
{ 998,  3, 4, 2 },//6.27 - 8.36 MHz, (16,22,24)
{ 1121,  3, 5, 3 },//5.62 - 7.49 MHz, (16,26,28)
{ 1301,  5, 5, 5 },//4.65 - 6.2 MHz, (24,26,36)
{ 1548,  5, 7, 6 },//3.97 - 5.29 MHz, (24,34,40)
{ 1852,  6, 9, 9 },//3.25 - 4.33 MHz, (28,42,52)
{ 2224,  8, 10, 11 },//2.75 - 3.67 MHz, (36,46,60)
{ 2652,  10, 12, 14 },//2.29 - 3.05 MHz, (44,54,72)
{ 3197,  12, 15, 18 },//1.89 - 2.52 MHz, (52,66,88)
{ 3873,  15, 18, 23 },//1.56 - 2.08 MHz, (64,78,108)
{ 4675,  18, 22, 28 },//1.3 - 1.73 MHz, (76,94,128)
{ 5594,  22, 26, 34 },//1.09 - 1.45 MHz, (92,110,152)
{ 6687,  27, 31, 42 },//0.91 - 1.21 MHz, (112,130,184)
{ 8011,  32, 38, 51 },//0.75 - 1.01 MHz, (132,158,220)
{ 9734,  38, 47, 63 },//0.62 - 0.83 MHz, (156,194,268)
{ 11742,  45, 58, 77 },//0.52 - 0.69 MHz, (184,238,324)
{ 13984,  53, 70, 93 },//0.43 - 0.58 MHz, (216,286,388)
{ 16905,  61, 88, 115 },//0.36 - 0.48 MHz, (248,358,476)
{ 20239,  72, 108, 140 },//0.3 - 0.4 MHz, (292,438,576)
{ 24409,  84, 130, 165 },//0.25 - 0.33 MHz, (340,526,676)
{ 29028,  100, 160, 200 },//0.21 - 0.28 MHz, (404,646,816)
{ 36571,  0, 0 ,0  },//<0.21 MHz - Min. Frequency
};

//! Selects Rx and Tx routine to be used according to SYNC length in \ref cable_status structure.
//! Sets up the soft wait delays
//!
//! @return
//!   \ref BDM_RC_OK             => Success \n
//!   \ref BDM_RC_NO_TX_ROUTINE  => No suitable Tx routine found \n
//!   \ref BDM_RC_NO_RX_ROUTINE  => No suitable Rx routine found \n
//!
U8 bdm_RxTxSelect(void) {
   const TxConfiguration  *txConfigPtr;
   const RxConfiguration  *rxConfigPtr;
   U8 sub;

   bdm_clearConnection();
   
#if 1
   for (sub=0; sub<(sizeof(txConfiguration)/sizeof(txConfiguration[0])); sub++) {

	   if (cable_status.sync_length >= txConfiguration[sub].syncThreshold) { // SYNC is >=
		 if ( sub >= (sizeof(txPtrs)/sizeof(txPtrs[0])) ) {
			 bdm_tx_ptr = bdm_txGeneric; // Select this routine
		 }
		 else {
			 bdm_tx_ptr = txPtrs[sub]; // Select this routine
		 }
		 txConfigPtr   = txConfiguration+sub;
         txTiming1     = txConfigPtr->time1;  // Save timing parameters
         txTiming2     = txConfigPtr->time2;
         txTiming3     = txConfigPtr->time3;
	  }
   }
   for (sub=0; sub<(sizeof(rxConfiguration)/sizeof(rxConfiguration[0])); sub++) {

	   if (cable_status.sync_length >= rxConfiguration[sub].syncThreshold) { // SYNC is >=
		 if ( sub >= (sizeof(rxPtrs)/sizeof(rxPtrs[0])) ) {
			 bdm_rx_ptr = bdm_rxGeneric; // Select this routine
		 }
		 else {
			 bdm_rx_ptr = rxPtrs[sub]; // Select this routine
		 }
		 rxConfigPtr   = rxConfiguration+sub;
         rxTiming1     = rxConfigPtr->time1;  // Save timing parameters
         rxTiming2     = rxConfigPtr->time2;
         rxTiming3     = rxConfigPtr->time3;
	  }
   }
#else
   for (  txConfigPtr  = txConfiguration+sizeof(txConfiguration)/sizeof(txConfiguration[0]);
        --txConfigPtr >= txConfiguration; ) { // Search the table

      if (cable_status.sync_length >= txConfigPtr->syncThreshold) { // SYNC is >=
         bdm_tx_ptr    = txConfigPtr->txFunc; // Select this routine
         txTiming1     = txConfigPtr->time1;  // Save timing parameters
         txTiming2     = txConfigPtr->time2;
         txTiming3     = txConfigPtr->time3;
         break;                               // Quit search
      }
   }
   if (bdm_tx_ptr==bdm_txEmpty) { // Return if no function found
      return(BDM_RC_NO_TX_ROUTINE);
   }
   for (  rxConfigPtr  = rxConfiguration+sizeof(rxConfiguration)/sizeof(rxConfiguration[0]);
        --rxConfigPtr >= rxConfiguration; ) { // Search the table

      if (cable_status.sync_length >= rxConfigPtr->syncThreshold) { // SYNC is >=
    	  
    	 if ( (rxConfigPtr-rxConfiguration) >= (sizeof(rxPtrs)/sizeof(rxPtrs[0])) ) {
             bdm_rx_ptr = bdm_rxGeneric; // Select this routine
    	 }
    	 else {
             bdm_rx_ptr = rxPtrs[rxConfigPtr-rxConfiguration]; // Select this routine
    	 }
         rxTiming1     = rxConfigPtr->time1;  // Save timing parameters
         rxTiming2     = rxConfigPtr->time2;
         rxTiming3     = rxConfigPtr->time3;
         break;                               // Quit search
      }
   }
#endif
   if (bdm_rx_ptr==bdm_rxEmpty) { // Return if no function found
      return(BDM_RC_NO_RX_ROUTINE);
   }
   // Calculate number of iterations for manual delay (each iteration is 8 cycles)
   cable_status.wait64_cnt  = cable_status.sync_length/(U16)(((8*60*128UL)/(BUS_FREQ/1000000)/64));
   cable_status.wait150_cnt = cable_status.sync_length/(U16)(((8*60*128UL)/(BUS_FREQ/1000000)/150));

//   // Correct for overhead in calling function etc. (JSR+RTS+JSR) = (5+4+5) ~ 2 iterations
//   if (cable_status.wait64_cnt<=2) {
//      cable_status.wait64_cnt = 1; // minimum of 1 iteration
//   }
//   else {
//      cable_status.wait64_cnt -= 2;
//   }
//   if (cable_status.wait150_cnt<=2) {
//      cable_status.wait150_cnt = 1; // minimum of 1 iteration
//   }
//   else {
//      cable_status.wait150_cnt -= 2;
//   }
   return(BDM_RC_OK);
}

// PARTID read from HCS12 - used to confirm target connection speed and avoid needless probing
static U16 partid = 0xFA50;

//! Confirm communication at given Sync value.
//! Only works on HC12 (and maybe only 1 of 'em!)
//!
//! @return
//!   == \ref BDM_RC_OK  => Success \n
//!   != \ref BDM_RC_OK  => Various errors
//!
#pragma MESSAGE DISABLE C4001 // Disable warnings about Condition always true
U8 bdmHC12_confirmSpeed(U16 syncValue) {
U8 rc;

   cable_status.sync_length = syncValue;

   rc = bdm_RxTxSelect(); // Drivers available for this frequency?
   if (rc != BDM_RC_OK) {
      goto tidyUp;
   }
   rc = BDM_RC_BDM_EN_FAILED; // Assume probing failed
   {
      U16 probe;
      
      // Check if we can read a previous PARTID, if so assume still connected 
      // and avoid further target probing
      // This should be the usual case
      (void)BDM12_CMD_READW(HCS12_PARTID,&probe);
      if (probe == partid) {
        return BDM_RC_OK;
      }
   }
   do {
      U16 probe;
      // This method works for secured or unsecured devices
      // in special mode that have a common Flash type.
      // BUT - it may upset flash programming if done at wrong time
      
      // Set FDATA to 0xAA55 & read back
      (void)BDM12_CMD_WRITEW(0x10A,0xAA55);
      (void)BDM12_CMD_READW(0x10A,&probe);

      // Read back correctly?
      if (probe != 0xAA55)
         break;

      // Set location to 0x55AA & read back
      (void)BDM12_CMD_WRITEW(0x10A,0x55AA);
      (void)BDM12_CMD_READW(0x10A,&probe);

      // Read back correctly?
      if (probe != 0x55AA)
         break;

      // Update partID
      (void)BDM12_CMD_READW(HCS12_PARTID, &partid);
      
      return BDM_RC_OK; // Success!

   } while (0);

   do {
      U8 probe;
      U8 originalValue;
      // This method works for unsecured devices
      // in special or non-special modes
      // BUT - it may upset CCR in some (unlikely?) cases

      // Get current BDMCCR
      (void)BDM12_CMD_BDREADB(HC12_BDMCCR,&originalValue);

      // Set location to 0xAA & read back
      (void)BDM12_CMD_BDWRITEB(HC12_BDMCCR,0xAA);
      (void)BDM12_CMD_BDREADB(HC12_BDMCCR,&probe);

      // Read back correctly?
      if (probe != 0xAA)
         break;

      // set location to 0x55 & read back
      (void)BDM12_CMD_BDWRITEB(HC12_BDMCCR,0x55);
      (void)BDM12_CMD_BDREADB(HC12_BDMCCR,&probe);

      // Read back correctly?
      if (probe != 0x55)
         break;

      // Restore BDMCCR
      (void)BDM12_CMD_BDWRITEB(HC12_BDMCCR,originalValue);

      // Update partID
      (void)BDM12_CMD_READW(HCS12_PARTID, &partid);
      
      return BDM_RC_OK; // Success!
      
   } while (0);
   
#if 0
   do {
      
   // Get current BDMSTS
   BDM12_CMD_BDREADB(HC12_BDMSTS,&originalValue);

   // Try to clear BDMSTS.ENBDM
   BDM12_CMD_BDWRITEB(HC12_BDMSTS,originalValue&~HC12_BDMSTS_ENBDM);
   BDM12_CMD_BDREADB(HC12_BDMSTS,&probe);

   if ((probe & HC12_BDMSTS_ENBDM) != 0) // Not clear now? - Try next speed
      goto tidyUp;

   // Try to set BDMSTS.ENBDM
   BDM12_CMD_BDWRITEB(HC12_BDMSTS,originalValue|HC12_BDMSTS_ENBDM);
   BDM12_CMD_BDREADB(HC12_BDMSTS,&probe);

   if ((probe & HC12_BDMSTS_ENBDM) == 0) // Not set now? - Try next speed
      goto tidyUp;

   return BDM_RC_OK; // Success!
      
   } while false;
#endif

tidyUp:
   cable_status.sync_length  = 1;
   cable_status.ackn         = WAIT;    // Clear indication of ACKN feature
   return rc;
}
#pragma MESSAGE DEFAULT C4001 // Disable warnings about Condition always true

//! Attempt to determine target speed by trial and error
//!
//! Basic process used to check for communication is:
//!   -  Attempt to modify the BDM Status register [BDMSTS] or BDM CCR Save Register [BDMCCR]
//!
//! The above is attempted for a range of 'nice' frequencies and then every Tx driver frequency. \n
//! To improve performance the last two successful frequencies are remembered.  This covers the \n
//! common case of alternating between two frequencies [reset & clock configured] with a minimum \n
//! number of probes.
//!
static U8 bdmHC12_alt_speed_detect(void) {
static const U16 typicalSpeeds[] = { // Table of 'nice' BDM speeds to try
   SYNC_MULTIPLE( 4000000UL),  //  4 MHz
   SYNC_MULTIPLE( 8000000UL),  //  8 MHz
   SYNC_MULTIPLE(16000000UL),  // 16 MHz
   SYNC_MULTIPLE(32000000UL),  // 32 MHz
   SYNC_MULTIPLE(24000000UL),  // 24 MHz
   SYNC_MULTIPLE(48000000UL),  // 48 MHz
   SYNC_MULTIPLE(20000000UL),  // 20 MHz
   SYNC_MULTIPLE( 2000000UL),  //  2 MHz
   SYNC_MULTIPLE(10000000UL),  // 10 MHz
   SYNC_MULTIPLE( 1000000UL),  //  1 MHz
   SYNC_MULTIPLE(  500000UL),  // 500kHz
   0
   };
#pragma DATA_SEG __SHORT_SEG Z_PAGE
static U16 lastGuess1 = SYNC_MULTIPLE(8000000UL);  // Used to remember last 2 guesses
static U16 lastGuess2 = SYNC_MULTIPLE(16000000UL); // Common situation to change between 2 speeds (reset,running)
#pragma DATA_SEG DEFAULT
const TxConfiguration  *txConfigPtr;
int sub;
U16 currentGuess;
U8  rc;

   // Try last used speed #1
   if (bdmHC12_confirmSpeed(lastGuess1) == BDM_RC_OK) {
      cable_status.speed = SPEED_GUESSED;  // Speed found by trial and error
      return BDM_RC_OK;
   }
   // Try last used speed #2
   currentGuess = lastGuess2;
   rc = bdmHC12_confirmSpeed(lastGuess2);
   if (rc != BDM_RC_OK) {
      // This may take a while
      setBDMBusy();
   }
   // Try some likely numbers!
   for (sub=0; typicalSpeeds[sub]>0; sub++) {
      if (rc == BDM_RC_OK) {
         break;
      }
      currentGuess = typicalSpeeds[sub];
      rc           = bdmHC12_confirmSpeed(currentGuess);
      }

   // Try each Tx driver BDM frequency
   for (  txConfigPtr  = txConfiguration+(sizeof(txConfiguration)/sizeof(txConfiguration[0])-1);
        --txConfigPtr >= txConfiguration; ) { // Search the table
      if (rc == BDM_RC_OK) {
         break;
      }
      currentGuess = (txConfigPtr->syncThreshold+(txConfigPtr+1)->syncThreshold)/2;
      rc           = bdmHC12_confirmSpeed(currentGuess);
      }

   if (rc == BDM_RC_OK) {
      // Update speed cache (LRU)
      lastGuess2       = lastGuess1;
      lastGuess1       = currentGuess;
      cable_status.speed = SPEED_GUESSED;  // Speed found by trial and error
      return BDM_RC_OK;
   }
   return rc;
}

#if (DEBUG&DEBUG_COMMANDS) // Debug commands enabled
U8 bdm_testTx(U8 speedIndex) {
	const TxConfiguration  *txConfigPtr;

	// Validate index
	if (speedIndex > (sizeof(txConfiguration)/sizeof(txConfiguration[0]))) {
		return BDM_RC_ILLEGAL_PARAMS;
	}
	txConfigPtr = &txConfiguration[speedIndex]; // selected routine

	if ( speedIndex >= (sizeof(txPtrs)/sizeof(txPtrs[0])) ) {
		bdm_tx_ptr = bdm_txGeneric; // Select this routine
	}
	else {
		bdm_tx_ptr = txPtrs[speedIndex]; // Select this routine
	}
	txTiming1     = txConfigPtr->time1;  // Save timing parameters
	txTiming2     = txConfigPtr->time2;
	txTiming3     = txConfigPtr->time3;

	bdm_txPrepare();
	bdmTx(0xF0);
	WAIT_MS(1);
	bdmTx(0x0F);
	WAIT_MS(1);
	bdmTx(0xAA);
	WAIT_MS(1);
	bdmTx(0x55);
	WAIT_MS(1);
	bdmHCS_interfaceIdle();
	return BDM_RC_OK;
}
#endif



#if (DEBUG&DEBUG_COMMANDS) // Debug commands enabled

//! Used to check the phase relationship between port reads and writes.
//!
//!  The higher speed BDM communication routines have very critical timing.  This routine
//!  allowed the timing delays between instructions accessing the ports and port pin changes to be measured.
//!  More precisely, the difference in delays between port reads and writes was measured.
//!  This may vary a bit with a particular chip but it's better than ignoring it.
//!
//! @warning This function will hang the BDM
//!
void bdm_checkTiming(void) {
#if ((TARGET_HARDWARE==H_USBDM) && (HW_CAPABILITY&CAP_RST_IO)) // Requires RESET driver!!
//! ToDO - Broken in USBDM_CF!!!!

   DATA_PORT_DDR = BDM_OUT_MASK|BDM_EN_MASK;     // BKGD pin is driven, Reset pin is driven
   RESET_OUT_DDR = 1;

   asm {
      LDHX   @DATA_PORT                                // Point X at DATA_PORT

   Loop1:
      LDA   #BDM_OUT_MASK|BDM_EN_WR_MASK               // [2      ] Mask for BDM high
      BRN   *+0                                        // [3      ] Waste some time
   Loop:
      BRN   *+0                                        // [3      ] Waste some time
      BRN   *+0                                        // [3      ]
      BRN   *+0                                        // [3      ]
      BRN   *+0                                        // [3      ]

      ASM_RESET_LOW
      CLR   ,X                                         // [4  rfwp] Drive BKGD low
                                                       // ---------
      STA   ,X                                         // [2    wp] Drive BKGD high
                                                       // --------- 2 cycles low
      BSET  BDM_EN_BIT,DATA_PORT                   // [4  rfwp] 3-state BKGD pin
                                                       // [1     p]
      BRN   *+0                                        // [3      ]
      LDA   ,X                                         // [2    rf] Sample BDM_IN
                                                       // --------- 12 cycles from BKGD low
                                                       // [1     p]
      AND   #BDM_IN_MASK                               // [2      ] Input high?
      ASM_RESET_HIGH
      BEQ   Loop1                                      // [3      ] No - set RESET low

      LDA   #BDM_OUT_MASK|BDM_DIR_Rx_WR                // [2      ] Mask for BDM high, RESET High
      BRA   Loop                                       // [3      ]
      }
#endif// (HW_CAPABILITY&CAP_RST_IO)
}
#endif // (DEBUG&DEBUG_COMMANDS) // Debug commands enabled



//====================================================================================================
//====================================================================================================
//====================================================================================================
//====================================================================================================

#pragma MESSAGE DISABLE C20001 // Disable warnings about stackpointer
#pragma MESSAGE DISABLE C5703 // Disable warnings about unused parameter

#pragma NO_RETURN
#pragma NO_ENTRY
void bdmTx16(U16 data) {
   asm {
      ; HX = 16-bit data
      pshx 			// lsb
      pshh 			// msb
      pula				// msb
      bsr    bdmTx8
      pula				// lsb
   bdmTx8:
      ldhx   bdm_tx_ptr
      jsr    ,x
       rts
   }
}

#pragma NO_RETURN
#pragma NO_ENTRY
void bdmRx16(U16 *data) {
   asm {
      ; HX = data ptr
      bsr   bdmRx8			// 1st byte
   bdmRx8:					// 2nd byte
      pshx
      pshh
      ldhx   bdm_rx_ptr
      jsr    ,x
      pulh
      pulx
      sta    ,x
      aix    #1 
      rts
   }
}

#pragma NO_RETURN
#pragma NO_ENTRY
void bdmRx32(U32 *data) {
   asm {
      ; HX = data ptr
      bsr   bdmRx16
      bsr   bdmRx16
      rts
   }
}

#pragma MESSAGE DEFAULT C20001 // Restore warnings about stackpointer
#pragma MESSAGE DEFAULT C5703  // Restore warnings about unused parameter

//============================================================
// The following commands DO NOT expect an ACK & do not delay
// Interrupts are left disabled!
//

//! Write command byte, truncated sequence
//!
//! @param cmd command byte to write
//!
//! @note Interrupts are left disabled
//!
void BDM_CMD_0_0_T(U8 cmd) {
   bdm_txPrepare();
   bdmTx(cmd);
//   enableInterrupts();
}

//! Write command byte + parameter, truncated sequence
//!
//! @param cmd         command byte to write
//! @param parameter   byte parameter to write
//!
//! @note Interrupts are left disabled
//!
void BDM_CMD_1B_0_T(U8 cmd, U8 parameter) {
   BDM_CMD_0_0_T(cmd);
   bdmTx(parameter);
//   enableInterrupts();
}
   
//!  Special for Software Reset HCS08, truncated sequence
//!
//! @param cmd         command byte to write
//! @param parameter1  word parameter to write
//! @param parameter2  byte parameter to write
//!
//! @note Interrupts are left disabled
//!
void BDM_CMD_1W1B_0_T(U8 cmd, U16 parameter1, U8 parameter2) {
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx16(parameter1);
   bdmTx(parameter2);
//   enableInterrupts();
}

//============================================================
// The following commands DO NOT expect an ACK & do not delay
//

//! Write cmd without ACK (HCS08/RS08/CFV1)
//!
//! @param cmd command byte to write
//!
//! @note No ACK is expected
//!
void BDM_CMD_0_0_NOACK(U8 cmd) {
   bdm_txPrepare();
   bdmTx(cmd);
   BDM_3STATE();
   enableInterrupts();
}
   
//! Write cmd & read byte without ACK (HCS08)
//!
//! @param cmd command byte to write
//! @param result word read
//!
//! @note No ACK is expected
//!
void BDM_CMD_0_1B_NOACK(U8 cmd, U8 *result) {
   bdm_txPrepare();
   bdmTx(cmd);
   *result = bdm_rx();
   enableInterrupts();
}

//! Write cmd & byte without ACK
//!
//! @param cmd       command byte to write
//! @param parameter byte to write
//!
//! @note No ACK is expected
//!
void BDM_CMD_1B_0_NOACK(U8 cmd, U8 parameter) {
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx(parameter);
   BDM_3STATE();
   enableInterrupts();
}

//! Write cmd & read word without ACK (HCS08)
//!
//! @param cmd    command byte to write
//! @param result word read
//!
//! @note No ACK is expected
//!
void BDM_CMD_0_1W_NOACK(U8 cmd, U16 *result) {
   bdm_txPrepare();
   bdmTx(cmd);
   bdmRx16(result);
   enableInterrupts();
}

//! Write cmd & word without ACK
//!
//! @param cmd       command byte to write
//! @param parameter word to write
//!
//! @note No ACK is expected
//!
void BDM_CMD_1W_0_NOACK(U8 cmd, U16 parameter) {
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx16(parameter);
   BDM_3STATE();
   enableInterrupts();
}

//! Write cmd, word & read word without ACK
//!
//! @param cmd        command byte to write
//! @param parameter  word to write
//! @param result     word ptr for read (status+data byte)
//!
//! @note no ACK is expected
//!
void BDM_CMD_1W_1W_NOACK(U8 cmd, U16 parameter, U16 *result) {
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx16(parameter);
   bdmRx16(result);
   enableInterrupts();
}

//! Write cmd, word, byte & read byte without ACK
//!
//! @param cmd        command byte to write
//! @param parameter  word to write
//! @param value      byte to write
//! @param status     byte ptr for read
//!
//! @note no ACK is expected 
//!
void BDM_CMD_1W1B_1B_NOACK(U8 cmd, U16 parameter, U8 value, U8 *status) {
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx16(parameter);
   bdmTx(value);
//   bdm_wait64();
   *status = bdm_rx();
   enableInterrupts();
}
                                                         
//====================================================================
// The following DO expect an ACK or wait at end of the command phase

//! Write cmd
//!
//! @param cmd command byte to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_0_0(U8 cmd) {
U8 rc;
    BDM_CMD_0_0_T(cmd);
//    bdm_txPrepare();
//    bdmTx(cmd);
    rc = doACKN_WAIT64();
    enableInterrupts();
    return rc;
}

//! Write cmd, read byte
//!
//! @param cmd        command byte to write
//! @param result     byte read
//!
//! @note ACK is expected
//!
U8 BDM_CMD_0_1B(U8 cmd, U8 *result) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   rc = doACKN_WAIT64();
   *result = bdm_rx();
   enableInterrupts();
   return rc;
}

//! Write cmd & read word
//!
//! @param cmd    command byte to write
//! @param result word read
//!
//! @note ACK is expected
//!
U8 BDM_CMD_0_1W(U8 cmd, U16 *result) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   rc = doACKN_WAIT64();
   bdmRx16(result);
   enableInterrupts();
   return rc;
}

//! Write cmd & read longword
//!
//! @param cmd    command byte to write
//! @param result longword read
//!
//! @note ACK is expected
//!
U8 BDM_CMD_0_1L(U8 cmd, U32 *result) {
U8 rc;
//   bdm_txPrepare();
//   bdmTx(cmd);
   BDM_CMD_0_0_T(cmd);
   rc = doACKN_WAIT64();
   bdmRx32(result);
   return rc;
}

//! Write cmd & byte
//!
//! @param cmd        command byte to write
//! @param parameter  byte to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1B_0(U8 cmd, U8 parameter) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx(parameter);
   rc = doACKN_WAIT64();
   enableInterrupts();
   return rc;
}

//! Write cmd & word
//!
//! @param cmd       command byte to write
//! @param parameter word to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1W_0(U8 cmd, U16 parameter) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx16(parameter);
   rc = doACKN_WAIT64();
   enableInterrupts();
   return rc;
}
   
//! Write cmd & longword
//!
//! @param cmd       command byte to write
//! @param parameter longword to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1L_0(U8 cmd, U32 parameter) {
U8 rc;
//   bdm_txPrepare();
//   bdmTx(cmd);
   BDM_CMD_0_0_T(cmd);
   bdmTx16(parameter>>16);
   bdmTx16((U16)parameter);
   rc = doACKN_WAIT64();
   enableInterrupts();
   return rc;
}
   
//! Write cmd, word & read byte (read word but return byte - HC/S12(x))
//!
//! @param cmd       command byte to write
//! @param parameter word to write
//! @param result    byte read
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1W_1WB(U8 cmd, U16 parameter, U8 *result) {
U8 rc;
//   bdm_txPrepare();
//   bdmTx(cmd);
   BDM_CMD_0_0_T(cmd);
   bdmTx16(parameter);
   rc = doACKN_WAIT150();
   if ((parameter)&0x0001) {
      (void)bdm_rx();
      *result = bdm_rx();
   } else {
      *result = bdm_rx();
      (void)bdm_rx();
   }
   enableInterrupts();
   return rc;
}

//! Write cmd & 2 words
//!
//! @param cmd       command byte to write
//! @param parameter1 word to write
//! @param parameter2 word to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_2W_0(U8 cmd, U16 parameter1, U16 parameter2) {
U8 rc;
//   bdm_txPrepare();
//   bdmTx(cmd);
   BDM_CMD_0_0_T(cmd);
   bdmTx16(parameter1);
   bdmTx16(parameter2);
   rc = doACKN_WAIT150();
   enableInterrupts();
   return rc;
}

//! Write cmd, word & read word
//!
//! @param cmd        command byte to write
//! @param parameter  word to write
//! @param result     word read
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1W_1W(U8 cmd, U16 parameter, U16 *result) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx16(parameter);
   rc = doACKN_WAIT150();
   bdmRx16(result);
   enableInterrupts();
   return rc;
}

//! Write cmd, word and a byte 
//! (sends 2 words, the byte in both high and low byte of the 16-bit value)
//!
//! @param cmd        command byte to write
//! @param parameter1 word to write
//! @param parameter2 bye to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_2WB_0(U8 cmd, U16 parameter1, U8 parameter2) {
U8 rc;
//   bdm_txPrepare();
//   bdmTx(cmd);
   BDM_CMD_0_0_T(cmd);
   bdmTx16(parameter1);
   bdmTx(parameter2);
   bdmTx(parameter2);
   rc = doACKN_WAIT150();
   enableInterrupts();
   return rc;
}

//! Write cmd, word & read byte
//!
//! @param cmd        command byte to write
//! @param parameter  word to write
//! @param result     byte read
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1W_1B(U8 cmd, U16 parameter, U8 *result) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx16(parameter);
   rc = doACKN_WAIT64();
   *result = bdm_rx();
   enableInterrupts();
   return rc;
}
                                                         
//! Write cmd, word & byte
//!
//! @param cmd         command byte to write
//! @param parameter1  word to write
//! @param parameter2  byte to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1W1B_0(U8 cmd, U16 parameter1, U8 parameter2) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx16(parameter1);
   bdmTx(parameter2);
   rc = doACKN_WAIT150();
   enableInterrupts();
   return rc;
}

//! Write cmd, 24-bit value & byte
//!
//! @param cmd    command byte to write
//! @param addr   24-bit value to write
//! @param value  byte to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1A1B_0(U8 cmd, U32 addr, U8 value) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx((U8)(addr>>16));
   bdmTx16((U16)addr);
   bdmTx(value);
   rc = doACKN_WAIT150();
   enableInterrupts();
   return rc;
}

//! Write cmd, 24-bit value & word
//!
//! @param cmd    command byte to write
//! @param addr   24-bit value to write
//! @param value  word to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1A1W_0(U8 cmd, U32 addr, U16 value) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx((U8)(addr>>16));
   bdmTx16((U16)addr);
   bdmTx16(value);
   rc = doACKN_WAIT150();
   enableInterrupts();
   return rc;
}

//! Write cmd, 24-bit value & longword
//!
//! @param cmd    command byte to write
//! @param addr   24-bit value to write
//! @param value  ptr to longword to write
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1A1L_0(U8 cmd, U32 addr, U32 *value) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx((U8)(addr>>16));
   bdmTx16((U16)addr);
   bdmTx16((U16)(*value>>16));
   bdmTx16((U16)(*value));
   rc = doACKN_WAIT150();
   enableInterrupts();
   return rc;
}

//! Write cmd, 24-bit value & read byte
//!
//! @param cmd     command byte to write
//! @param addr    24-bit value to write
//! @param result  ptr to longword to read
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1A_1B(U8 cmd, U32 addr, U8 *result) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx((U8)(addr>>16)&0xFF);
   bdmTx16((U16)addr);
   rc = doACKN_WAIT150();
   *result = bdm_rx();
   enableInterrupts();
   return rc;
}

////! Converts the Coldfire CFV1_XCSR_CSTAT status to an error code
////!
////! @param xcsr_byte byte read from CFV1 XCSR
////!
////! @return error code 
////!
//U8 convertColdfireStatusByte(U8 xcsr_byte) {
//	xcsr_byte &= CFV1_XCSR_CSTAT;
//	if ((xcsr_byte & CFV1_XCSR_CSTAT_INVALID)) {
//		return BDM_RC_CF_DATA_INVALID;
//	}
//	if ((xcsr_byte & CFV1_XCSR_CSTAT_ILLEGAL)) {
//		return BDM_RC_CF_ILLEGAL_COMMAND;
//	}
//	if ((xcsr_byte & CFV1_XCSR_CSTAT_OVERRUN)) {
//		return BDM_RC_TARGET_BUSY;
//	}
//	return BDM_RC_OK;
//}

////! Write cmd, 24-bit value, check status & read byte
////!
////! @param cmd     command byte to write
////! @param addr    24-bit value to write
////! @param status  ptr to status byte to read
////! @param result  ptr to byte to read
////!
////! @note ACK is expected
////!
////! @return error code
////!
//U8 BDM_CMD_1A_CS_1B(U8 cmd, U32 addr, U8 *result) {
//U8 status;
//   bdm_txPrepare();
//   bdmTx(cmd);
//   bdmTx((U8)(addr>>16)&0xFF);
//   bdmTx16((U16)addr);
//   status = bdm_rx();
//   *result = bdm_rx();
//   enableInterrupts();
//   return convertColdfireStatusByte(status);
//}

//! Write cmd, 24-bit value & read word
//!
//! @param cmd     command byte to write
//! @param addr    24-bit value to write
//! @param result  ptr to word to read
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1A_1W(U8 cmd, U32 addr, U16 *result) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx((U8)(addr>>16)&0xFF);
   bdmTx16((U16)addr);
   rc = doACKN_WAIT150();
   bdmRx16(result);
   enableInterrupts();
   return rc;
}

//! Write cmd, 24-bit value & read longword
//!
//! @param cmd        command byte to write
//! @param addr       24-bit value to write
//! @param result     ptr to longword to read
//!
//! @note ACK is expected
//!
U8 BDM_CMD_1A_1L(U8 cmd, U32 addr, U32 *result) {
U8 rc;
   bdm_txPrepare();
   bdmTx(cmd);
   bdmTx((U8)(addr>>16)&0xFF);
   bdmTx16((U16)addr);
   rc = doACKN_WAIT150();
   bdmRx32(result);
   enableInterrupts();
   return rc;
}

#endif (HW_CAPABILITY&CAP_BDM)
