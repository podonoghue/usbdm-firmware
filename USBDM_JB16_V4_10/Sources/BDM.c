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

ToDo / Issues
+=========================================================================
|  * The use of LVC45 driver ICs produces glitches and/or bus conflicts.  
|    These are unavoidable unless external resistors are used.             
+=========================================================================

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

#if (HW_CAPABILITY&CAP_BDM)
void enInterrupts(void) {
   asm("cli");
}

//=============================================================================================================================================
#define BDM_SYNC_REQus          960U //!< us - length of the longest possible SYNC REQUEST pulse (128 BDM cycles @ 400kHz = 320us plus some extra time)
#define SYNC_TIMEOUTus          460U //!< us - longest time for the target to completed a SYNC pulse (16+128+margin cycles @ 400kHz = 375us)
#define ACKN_TIMEOUTus          375U //!< us - longest time after which the target should produce ACKN pulse (150 cycles @ 400kHz = 375us)
#define SOFT_RESETus          10000U //!< us - longest time needed for soft reset of the BDM interface (512 BDM cycles @ 400kHz = 1280us)
#define RESET_LENGTHms          100U //!< ms - time of RESET assertion
#define RESET_RELEASE_WAITms    270U //!< ms - max time to wait for the RESET pin to come high after release

/* Function prototypes */
       U8   bdm_syncMeasure(void);
       void bdmHCS_interfaceIdle(void);
static U8   bdmHC12_alt_speed_detect(void);

//========================================================
//
#pragma DATA_SEG __SHORT_SEG Z_PAGE
// MUST be placed into the direct segment (assumed in ASM code).
       U8 bitCount;  //!< Used as a general purpose variable in the bdm_Tx{} & bdm_Rx{}etc.
static U8 rxTiming1; //!< bdm_Rx timing constant #1
static U8 rxTiming2; //!< bdm_Rx timing constant #2
static U8 rxTiming3; //!< bdm_Rx timing constant #3
static U8 txTiming1; //!< bdm_Tx timing constant #1
static U8 txTiming2; //!< bdm_Tx timing constant #2
static U8 txTiming3; //!< bdm_Tx timing constant #3

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
   if (rc != BDM_RC_OK)
      return rc;

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

//!  Tries to connect to target - doesn't try other strategies such as reset.
//!  This function does a basic connect sequence.  It doesn't configure the BDM
//!  registers on the target or enable ACKN
//!
//! @return
//!    == \ref BDM_RC_OK                  => success                             \n
//!    == \ref BDM_RC_VDD_NOT_PRESENT     => no target power present             \n
//!    == \ref BDM_RC_RESET_TIMEOUT_RISE  => RESET signal timeout - remained low \n
//!    == \ref BDM_RC_BKGD_TIMEOUT        => BKGD signal timeout - remained low  \n
//!    != \ref BDM_RC_OK                  => other failures
//!
U8 bdm_physicalConnect(void){
U8 rc;

//   cable_status.reset = NO_RESET_ACTIVITY; // Clear the reset flag
   cable_status.speed   = SPEED_NO_INFO;   // No connection
   bdm_rx_ptr           = bdm_rxEmpty;     // Clear the Tx/Rx pointers
   bdm_tx_ptr           = bdm_txEmpty;     //    i.e. no com. routines found

   bdmHCS_interfaceIdle(); // Make sure interface is idle

   // Target has power?
   rc = bdm_checkTargetVdd();
   if (rc != BDM_RC_OK)
      return rc;

#if (HW_CAPABILITY&CAP_RST_IO)
   // Wait with timeout until both RESET and BKGD  are high
   if (bdm_option.useResetSignal) {
      WAIT_WITH_TIMEOUT_MS(RESET_RELEASE_WAITms,(RESET_IN!=0)&&(BDM_IN!=0));
      if (RESET_IN==0)
         return(BDM_RC_RESET_TIMEOUT_RISE);  // RESET timeout
   }
#else
   WAIT_WITH_TIMEOUT_MS(RESET_RELEASE_WAITms,(BDM_IN!=0));
#endif

   if (BDM_IN==0)
      return(BDM_RC_BKGD_TIMEOUT);  // BKGD timeout

   rc = bdm_syncMeasure();
   if (rc != BDM_RC_OK) // try again
      rc = bdm_syncMeasure();
   if ((rc != BDM_RC_OK) &&                // Trying to measure SYNC was not successful
       (bdm_option.guessSpeed) &&          // Try alternative method if enabled
       (cable_status.target_type == T_HC12)) { // and HC12 target
      rc = bdmHC12_alt_speed_detect();     // Try alternative method (guessing!)
   }
   if (rc != BDM_RC_OK)
      return(rc);

   // If at least one of the two methods succeeded, we can select
   //  the right Rx and Tx routines
   rc = bdm_RxTxSelect();
   if (rc != BDM_RC_OK) {
      cable_status.speed = SPEED_NO_INFO;  // Indicate that we do not have a connection
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
      if (rc != BDM_RC_OK)
   	     return rc;
   }
   bdm_acknInit();  // Try the ACKN feature

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

   DISABLE_RESET_SENSE_INT(); // Mask RESET IC interrupts

   bdmHCS_interfaceIdle();  // Make sure BDM interface is idle

   mode &= RESET_MODE_MASK;

   // Wait with timeout until both RESET and BKGD are high
//   WAIT_WITH_TIMEOUT_MS(RESET_WAIT, (RESET_IN!=0)&&(BDM_IN!=0));
   WAIT_WITH_TIMEOUT_S(2, (RESET_IN!=0)&&(BDM_IN!=0));

   if (RESET_IN==0) return(BDM_RC_RESET_TIMEOUT_RISE);
   if (BDM_IN==0)   return(BDM_RC_BKGD_TIMEOUT);

   bdm_txPrepare(); // BKGD & RESET 3-state control on DIR_PORT

   if (mode==RESET_SPECIAL) {
      BDM_OUT = 0;        // Drive BKGD low
      BDM_ENABLE_TX();

      WAIT_MS(RESET_SETTLEms); // Wait for signals to settle
   }

   RESET_LOW();

   WAIT_MS(RESET_LENGTHms);  // Wait for reset pulse duration

   RESET_3STATE();

   // Wait with timeout until RESET is high
   WAIT_WITH_TIMEOUT_MS(RESET_RELEASE_WAITms, (RESET_IN!=0));

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

   if (RESET_IN==0)      // RESET failed to rise
      return(BDM_RC_RESET_TIMEOUT_RISE);

   CLEAR_RESET_SENSE_FLAG(); // Clear RESET IC Event
   ENABLE_RESET_SENSE_INT(); // Enable RESET IC interrupts

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
      BDM_OUT  = 0;      // drive BKGD low (out of reset)
      BDM_ENABLE_TX();
   }
   //enableInterrupts();
   
#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
#endif

   WAIT_MS(RESET_SETTLEms);   // Wait for target to start reset (and possibly assert reset)

#if (HW_CAPABILITY&CAP_RST_IO)
   if (bdm_option.useResetSignal) {
      // Wait with timeout until RESET is high (may be held low by processor)
      WAIT_WITH_TIMEOUT_MS(RESET_RELEASE_WAITms, (RESET_IN!=0));
      // Assume RESET risen - check later after cleanup
   }
#endif // (HW_CAPABILITY&CAP_RST_IO)

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   if (mode == 0) {   // Special mode - release BKGD
      WAIT_US(BKGD_WAITus);      // Wait for BKGD assertion time after reset rise
      BDM_3STATE_TX();
      WAIT_MS(RESET_SETTLEms);     // Time to wait for signals to settle
   }

   bdm_txFinish();

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

   // BKGD pin=L, wait...
   DATA_PORT      = BDM_DIR_Rx_WR;
   DATA_PORT_DDR  = BDM_DIR_Rx_MASK|BDM_OUT_MASK;
   RESET_LOW();
   WAIT_MS(RESET_SETTLEms);

   // Release reset
   RESET_3STATE();

   // Wait for Vdd to rise within 50% of 3V and RESET to return high
   // RESET rise may be delayed by target POR
   WAIT_WITH_TIMEOUT_MS( 250 /* ms */, (bdm_targetVddMeasure()>75)&&
                                       (!bdm_option.useResetSignal)||(RESET_IN!=0));

   // Let signals settle & CPU to finish reset (with BKGD held low)
   WAIT_US(BKGD_WAITus);

   if (bdm_targetVddMeasure()<=70) // Vdd didn't turn on!
      rc = BDM_RC_VDD_NOT_PRESENT;

   if (bdm_option.useResetSignal && (RESET_IN==0)) // RESET didn't rise
      rc = BDM_RC_RESET_TIMEOUT_RISE;

   bdmHCS_interfaceIdle();  // Make sure BDM interface is idle (BKGD now high)

   // Let signals settle
   WAIT_US(RESET_SETTLEms);

   if (BDM_IN==0) // BKGD didn't rise!
      rc = BDM_RC_BKGD_TIMEOUT;

   cable_status.reset = RESET_DETECTED;    // Record the fact that reset was asserted

#endif // (HW_CAPABILITY&CAP_VDDSENSE)
   return(rc);
}

//!  Directly set the interface levels
//!
//! @param level see \ref InterfaceLevelMasks_t
//!
//! @return
//!    == \ref BDM_RC_OK     => Success \n
//!    != \ref BDM_RC_OK     => various errors
//
U8  bdm_setInterfaceLevel(U8 level) {

   BDM_DIR_DDR &= BDM_DIR; // Disable BKGD control from BDM_DIR

   switch (level&SI_BKGD) {
      case SI_BKGD_LOW :  // BKGD pin=L
         DATA_PORT      = BDM_DIR_Rx_WR;                // Enable BKGD buffer,  BKGD = 0
         DATA_PORT_DDR  = BDM_DIR_Rx_MASK|BDM_OUT_MASK; // Enable port pins
         break;
      case SI_BKGD_HIGH : // BKGD pin=H
         DATA_PORT      = BDM_DIR_Rx_WR  |BDM_OUT_MASK; // Enable BKGD buffer,  BKGD = 1
         DATA_PORT_DDR  = BDM_DIR_Rx_MASK|BDM_OUT_MASK; // Enable port pins
         break;
      default :           // BKGD pin=3-state
         DATA_PORT      = BDM_DIR_Rx_RD;                // Disable BKGD buffer, BKGD = X
         DATA_PORT_DDR  = BDM_DIR_Rx_MASK;              // Enable port pins
         break;
   }

#if (HW_CAPABILITY & CAP_RST_IO)
   switch (level&SI_RESET) {
      case SI_RESET_LOW : // RESET pin=L
         RESET_LOW();
         break;
      default :
         RESET_3STATE();
         break;
   }
#endif

#if (HW_CAPABILITY & CAP_RST_IO)
   return (RESET_IN?SI_RESET_3STATE:SI_RESET_LOW)|(BDM_IN?SI_BKGD_3STATE:SI_BKGD_LOW);
#else
   return (BDM_IN?SI_BKGD:0);
#endif
}

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
   WAIT_WITH_TIMEOUT_US(BDM_SYNC_REQus, (BDM_IN!=0));

   if (BDM_IN==0)
      return(BDM_RC_BKGD_TIMEOUT); // Timeout !

#if (DRIVER == LVC45)
   // This sequence briefly drives BKGD high (670 ns) before driving low
   // Can't be avoided if using LVC45 buffer (conflict on BDM_OUT pin otherwise)
   // BKGD pin=L, RESET pin=3S (RESET 3-state & pulled high)
   DATA_PORT     = 0           |BDM_DIR_Rx_WR;
   DATA_PORT_DDR = BDM_OUT_MASK|BDM_DIR_Rx_MASK;
#else
   // This version directly drives BKGD low - no glitches!
   // BKGD pin=L, RESET pin=3S (RESET 3-state & pulled high)
   DATA_PORT     = 0           |BDM_DIR_Rx_RD;
   DATA_PORT_DDR = BDM_OUT_MASK|BDM_DIR_Rx_MASK;
   DATA_PORT     = 0           |BDM_DIR_Rx_WR;
#endif

   WAIT_US(BDM_SYNC_REQus);     // Wait SYNC request time

#if (DEBUG&SYNC_DEBUG)
   DEBUG_PIN   = 0;
   DEBUG_PIN   = 1;
#endif

   // Set up Input capture & timeout timers
   IC_BDM_TIMING_TMOD    = (U16)(ACKN_MICROSECOND(SYNC_TIMEOUTus)); // Set timeout
   IC_BDM_TIMING_TSC0    = T1SC0_ELS0B_MASK;              // Capture falling edges
   IC_BDM_TIMING_TSC     = ACKN_T1SC_VALUE;               // Reset the timer, clear TOF and start counting at fastest rate
   IC_BDM_TIMING_TSC_TOF = 0;                             // Clear TOF 
   IC_BDM_TIMING_TSC0_CH0F=0;                             // Clear capture flag

   asm {
      LDX   #BDM_OUT_MASK|BDM_DIR_Rx_WR            // Mask to Drive BKGD high
      LDA   #BDM_OUT_MASK|BDM_DIR_Rx_RD            // Mask to 3-state BKGD
      STX   DATA_PORT                              // [3   pwp]  Drive BKGD high (for 500 ns)
      STA   DATA_PORT                              // [3   pwp]  3-state BKGD
      // It took 500 ns cycles from bringing BKGD high to 3-state (3 cycles @ 6 MHz)
      // Target drives BKGD line after 16 BDM clock cycles
      // Fast enough up to approx 500ns/(16 BDM cycles) = 31 MHz BDM Frequency (62 MHz Target Crystal freq.)
   }

   while ((IC_BDM_TIMING_TSC0_CH0F==0)&&(WAIT_TIMER_EXPIRED==0)) { // wait for capture or timeout  
   }
   time               = IC_BDM_TIMING_TCH0;        // Capture start of the SYNC
   IC_BDM_TIMING_TSC0 = T1SC0_ELS0A_MASK;          // Capture rising edge, clear capture flag

   // it takes 22 cycles to re-enable capture (worst case) which is good enough up to 128*6/22 = 34 MHz on JB16 (64 crystal)
   while ((IC_BDM_TIMING_TSC0_CH0F==0)&&(WAIT_TIMER_EXPIRED==0)) { // Wait for capture or timeout
   }
   time = IC_BDM_TIMING_TCH0-time;                 // calculate length of the SYNC pulse

#if (DEBUG&SYNC_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   if (WAIT_TIMER_EXPIRED) {
      return(BDM_RC_SYNC_TIMEOUT);         // Timeout !
   }

#if (ACKN_TIMER_FREQ==3000000UL)
   cable_status.sync_length=(time<<2)+(time<<4);  // multiply by 20 to get the time in 60MHz ticks
#elif (ACKN_TIMER_FREQ==6000000UL)
   cable_status.sync_length=(time<<1)+(time<<3);  // multiply by 10 to get the time in 60MHz ticks
#elif (ACKN_TIMER_FREQ==24000000UL)
   cable_status.sync_length=(time<<1)+(time>>1);  // multiply by 2.5 to get the time in 60MHz ticks
#elif (ACKN_TIMER_FREQ==30000000UL)
   cable_status.sync_length=(time<<1);            // multiply by 2 to get the time in 60MHz ticks
#elif (ACKN_TIMER_FREQ==15000000UL)
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
   IC_BDM_TIMING_TMOD = (U16)(ACKN_MICROSECOND(ACKN_TIMEOUTus));  // The timer will set the TOF flag as soon as the timeout time is reached
   IC_BDM_TIMING_TSC  = ACKN_T1SC_VALUE;                  // Reset the timer, clear TOF and start counting at fastest rate
   IC_BDM_TIMING_TSC0 = T1SC0_ELS0A_MASK;	          // Capture rising edges

#if (DEBUG&ACK_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
   DEBUG_PIN     = 0;
#endif

   cable_status.ackn = ACKN;              // Switch ACKN on

   // Send the ACK enable command to the target
   if (cable_status.target_type==T_CFV1)
      rc = BDMCF_CMD_ACK_ENABLE();
   else
      rc = BDM_CMD_ACK_ENABLE();

   // If ACKN fails turn off ACKN (RS08 or early HCS12 target)
   if (rc == BDM_RC_ACK_TIMEOUT)
      cable_status.ackn = WAIT;  // Switch the ackn feature off
}

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
   bdm_txFinish();
   if (cable_status.ackn==ACKN) {
      // Set up Input capture & timeout timers
      IC_BDM_TIMING_TSC = T1SC_TRST_MASK;                    // Restart the timer
      WAIT_TIMER_EXPIRED = 0;                                // Clear TOF (must be RMW cycle!)

      // Wait for pin capture or timeout
      while ((IC_BDM_TIMING_TSC0_CH0F==0)&&(WAIT_TIMER_EXPIRED==0)){   
      }
      IC_BDM_TIMING_TSC0 = T1SC0_ELS0A_MASK;                 // Capture rising edge, clear capture flag
      if (WAIT_TIMER_EXPIRED) {     // Timeout
         return BDM_RC_ACK_TIMEOUT;  //   Return timeout error
      }
   }
   else asm {
      // Wait for 64 target clock cycles
         ldhx  cable_status.wait64_cnt   // Number of loop iterations to wait
      loop:
         aix   #-1      ; [2]
         cphx  #0       ; [3]
         bne   loop     ; [3] 8 cycles / iteration
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
   bdm_txFinish();
   if (cable_status.ackn==ACKN) {
      // Set up Input capture & timeout timers
      IC_BDM_TIMING_TSC = T1SC_TRST_MASK;                    // Restart the timer
      WAIT_TIMER_EXPIRED = 0;                                // Clear TOF (must be RMW cycle!)
      // Wait for pin capture or timeout
      while ((IC_BDM_TIMING_TSC0_CH0F==0)&&(WAIT_TIMER_EXPIRED==0)){   
      }
      IC_BDM_TIMING_TSC0 = T1SC0_ELS0A_MASK;                 // Capture rising edge, clear capture flag
      if (WAIT_TIMER_EXPIRED) {     // Timeout
         return BDM_RC_ACK_TIMEOUT;  //   Return timeout error
      }
   }
   else asm {
      // Wait for 150 target clock cycles
         ldhx  cable_status.wait150_cnt    // Number of loop iterations to wait
      loop:
         aix   #-1      ; [2]
         cphx  #0       ; [3]
         bne   loop     ; [3] 8 cycles / iteration
   }
   return BDM_RC_OK;
}

//!  Halts the processor - places in background mode
//!
U8 bdm_halt(void) {

   if (cable_status.target_type==T_CFV1)
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
   else
      return BDM_CMD_GO();
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
   else
      return BDM_CMD_TRACE1();
}

//!  Turns off the BDM interface
//!
//!  Depending upon settings, may leave target power on.
//!
void bdmHCS_off( void ) {
#if ((HW_CAPABILITY & CAP_FLASH) != 0)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif
   if (!bdm_option.leaveTargetPowered)
      VDD_OFF();
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

   RESET_3STATE();
   BDM_3STATE_TX();

   DATA_PORT      = BDM_DIR_Rx_RD;      //
   DATA_PORT_DDR  = BDM_DIR_Rx_MASK;    // BDM_DIR_Rx enabled on DATA_PORT
   DIR_PORT_DDR  &= ~BDM_DIR_MASK;      // Transfer BKGD 3-state control to BDM_DIR_Rx on DATA_PORT

//   - BKGD & RESET pins on BDM cable are 3-state and may be read through BDM_IN & RESET_IN
//   - BDM_OUT is 3-state
//   - BDM_DIR_Rx is enabled and BDM_DIR is 3-state so BKGD direction is controlled by BDM_DIR_Rx
//   - BDM_DIR_Rx is BDM_DIR_Rx_RD (and hence BKGD pin is 3-state)
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
   POCR         |= POCR_PAP_MASK|POCR_PCP_MASK|POCR_PTE20P_MASK;
   POCR         |= POCR_PTDLDD_MASK;  // Set PTD.2-5 to 10mA direct LED drive - no resistor required

   
   bdmHCS_interfaceIdle();
}

//! Prepares for transmission of BDM data
//!
//! Interrupts need to be disabled for the duration of all BDM commands. \n
//! It is up to the caller to see to this.
//!
void bdm_txPrepare(void) {
   //disableInterrupts();

   // Hand over BKGD direction control from BDM_DIR_Rx (on BDM_DATA_PORT) to BDM_DIR (on BDM_DIR_PORT)
   // Note: direction of the BKGD driver must be controlled during transition to avoid glitches on BDM
                                                // Keep BKGD 3-state until transfer
   DATA_PORT     = BDM_OUT_MASK|BDM_DIR_Rx_RD;  // Make sure BKGD is currently 3-state &
                                                //    will be driven high once the driver is enabled
   BDM_3STATE_TX();                             // Make sure BKGD is 3-state when control is transferred to DIR_PORT

   DIR_PORT_DDR  |= BDM_DIR_MASK;               // Transfer 3-state control to DIR_PORT
   DATA_PORT_DDR = BDM_OUT_MASK;                // BDM_OUT active others 3-state

   // BKGD direction controlled by BDM_DIR in DIR_PORT
   // BDM_DIR is set to disable the BKGD driver
   //    Macros used to enable/disable BKGD drive
   //      BDM_ENABLE_ASM_TX/BDM_ENABLE_TX()            // Enable BKGD
   //      BDM_3STATE_ASM_TX/BDM_3STATE_TX()            // 3-state BKGD
   // BKGD is driven via BDM_OUT in DATA_PORT - hard coded into the assembly code
   // Other bits of DATA_PORT are configured as inputs and have no effect when written
   // DATA_PORT = 0xxxxxxxx => BKGD driven low  (if enabled by BDM_DIR)
   // DATA_PORT = 1xxxxxxxx => BKGD driven high (if enabled by BDM_DIR)
   // RESET state is unchanged
}

//! Finishes transmission of BDM data and sets up for reception.
//!
void bdm_txFinish(void) {

   // Hand over BKGD direction control from BDM_DIR (on BDM_DIR_PORT) to BDM_DIR_Rx (on BDM_DATA_PORT)
   // Note: direction of the BKGD driver must be controlled during transition to avoid glitches on BDM
   BDM_3STATE_TX();                              // Make sure BKGD is currently 3-state
   DATA_PORT     = BDM_OUT_MASK|BDM_DIR_Rx_RD;   // Make sure BKGD will remain 3-state after handover

   DATA_PORT_DDR  = BDM_OUT_MASK|BDM_DIR_Rx_MASK; // Transfer BDM_DIR control from DIR_PORT to DATA_PORT
   DIR_PORT_DDR  &= ~BDM_DIR_MASK;

   // BKGD is now driven via BDM_OUT (DATA_PORT.7) with 3-state control on BDM_DIR_Rx (DATA_PORT.6).
   // Other bits of DATA_PORT are configured as inputs and have no effect when written.
   // The following examples assume active low enables of the buffer (74LC125 or similar driver IC)
   // DATA_PORT = 0xxx0xxxx => BKGD driven low
   // DATA_PORT = 1xxx1xxxx => BKGD 3-state (input)
   // DATA_PORT = 1xxx0xxxx => BKGD driven high (not used by Rx routines)
   // DATA_PORT = 0xxx1xxxx => BKGD 3-state (not used by Rx routines?)
}

#pragma MESSAGE DISABLE C1404 // Disable warnings about no return statement

//==============================================================
// Tx Routines bdm_tx..
//==============================================================
// Transmit 8 bits of data, MSB first
// These routines assume the following:
//    BKGD direction controlled via BDM_DIR in DIR_PORT
//      [use BDM_3STATE_TX/BDM_3STATE_ASM_TX or BDM_ENABLE_TX/BDM_ENABLE_ASM_TX macros]
//    BKGD output value may be controlled by DATA_PORT.7
//    The rest of DATA_PORT must be disabled call bdm_txPrepare
// Leaves BDM_OUT 3-state via BDM_DIR call bdm_txFinish to clean up
//
#if (BDM_OUT_BIT != 7)
#error "The bdm_txN routines requires BDM_OUT=PTA.7"
#endif

//=========================================================================
#pragma MESSAGE DISABLE C5703 // Disable warnings about unused parameter
//! Dummy BDM Tx routine
//!
void bdm_txEmpty(U8 data) {
   // If BDM command is executed with this routine set command failed...
//   commandBuffer[0] = BDM_RC_NO_CONNECTION;
}

#if TARGET_HARDWARE==H_USBSPYDER
//=========================================================================
// (2,2,12)
// This meets the timing but at a slow data rate
void bdm_txX(U8 data) {
   asm {
      STA  bitCount                // Save data
      
      LDX  #0             // Value to drive BDM_OUT low
      LDA  #BDM_OUT_MASK  // Value to drive BDM_OUT low
      
      SEC
      ROR  bitCount       // Get first data bit (and add sentinel)
      
   Loop:
      BCS  IsOne          // [3]  '1' -> skip
      
   isZero:
      PSHX                // Save '0' on stack
      BRA  common
      
   isOne:
      PSHA                // Save '1' on stack

   common:   
      ASR  bitCount       // [4]   Get next data bit
      BNE  Loop           // [3]   Finished? - yes, skip

      LDHX  @DATA_PORT             // Point HX at DATA_PORT
      BDM_ENABLE_ASM_TX            // Enable BDM
      
      LDA  #BDM_OUT_MASK           // Mask to drive BDM_OUT high
      PULA
      
   IsZero:
      CLR  ,X                      // [2]   Drive BDM low
                                   // --- 12
      CLR  ,X                      // [2]   Drive data (0) to BDM
                                   // --- 2
      STA  ,X                      // [2]   Drive BDM high
                                   // --- 2
      ASL  bitCount                // [4]   Get next data bit
      BEQ  Done                    // [3]   Finished? - yes, skip
      BCC  IsZero                  // [3]  '0' -> skip

   IsOne:
      CLR  ,X                      // [2]   Drive BDM low
                                   // --- 12   
      STA  ,X                      // [2]   Drive data (1) to BDM
                                   // --- 2
      STA  ,X                      // [2]   Drive BDM high
                                   // --- 2
      ASL  bitCount                // [4]   Get next data bit
      BNE  Loop                    // [3]   Finished? - no, loop
   
   Done:
      BDM_3STATE_ASM_TX            // [4]   3-state BDM         

      }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
   /* it takes 8 cycles from end of the last bit till enable of the ACKN capture */
   /* that is short enough for BDM freq of: (32+16)*3/8 = 18 MHz */
   /* 32+16 comes from minimum delay between command and ACKN (32 BDM cycles) and 16 cycles of the ACKN pulse (capturing its rising edge) */
}


#endif



//=========================================================================
// (2,2,12)
// This meets the timing but at a slow data rate
void bdm_tx0(U8 data) {
   asm {
      LDHX  @DATA_PORT             // Point HX at DATA_PORT
      BDM_ENABLE_ASM_TX            // Enable BDM
      STA  bitCount                // Save data
      LDA  #BDM_OUT_MASK           // Mask to drive BDM_OUT high
      SEC
      ROL  bitCount                // Get first data bit (and add sentinel)
      
   Loop:
      BCS  IsOne                   // [3]  '1' -> skip
      
   IsZero:
      CLR  ,X                      // [2]   Drive BDM low
                                   // --- 12
      CLR  ,X                      // [2]   Drive data (0) to BDM
                                   // --- 2
      STA  ,X                      // [2]   Drive BDM high
                                   // --- 2
      ASL  bitCount                // [4]   Get next data bit
      BEQ  Done                    // [3]   Finished? - yes, skip
      BCC  IsZero                  // [3]  '0' -> skip

   IsOne:
      CLR  ,X                      // [2]   Drive BDM low
                                   // --- 12   
      STA  ,X                      // [2]   Drive data (1) to BDM
                                   // --- 2
      STA  ,X                      // [2]   Drive BDM high
                                   // --- 2
      ASL  bitCount                // [4]   Get next data bit
      BNE  Loop                    // [3]   Finished? - no, loop
   
   Done:
      BDM_3STATE_ASM_TX            // [4]   3-state BDM         

      }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
   /* it takes 8 cycles from end of the last bit till enable of the ACKN capture */
   /* that is short enough for BDM freq of: (32+16)*3/8 = 18 MHz */
   /* 32+16 comes from minimum delay between command and ACKN (32 BDM cycles) and 16 cycles of the ACKN pulse (capturing its rising edge) */
}


//=========================================================================
// (2,3,4)
void bdm_tx1(U8 data) {
   asm {
      LDHX  @DATA_PORT     // Point HX at DATA_PORT
      BDM_ENABLE_ASM_TX    // Enable BKGD (high)
      
      /* bit 7 (MSB) */
      SEC                  // [1]   Used by ROR
      CLR   ,X             // [2]   Drive BDM low
                           // --- 4
      STA   ,X             // [2]   Drive data to BDM
                           // --- 2
      ROR   ,X             // [3]   Drive BDM high
                           // --- 3
      LSLA                 // [1]   Next bit
                           // ===========
      /* bit 6 */
      SEC                  // [1]   Used by ROR
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ROR   ,X             // [3]   Drive BDM high
      LSLA                 // [1]   Next bit

      /* bit 5 */
      SEC                  // [1]   Used by ROR
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ROR   ,X             // [3]   Drive BDM high
      LSLA                 // [1]   Next bit

      /* bit 4 */
      SEC                  // [1]   Used by ROR
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ROR   ,X             // [3]   Drive BDM high
      LSLA                 // [1]   Next bit

      /* bit 3 */
      SEC                  // [1]   Used by ROR
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ROR   ,X             // [3]   Drive BDM high
      LSLA                 // [1]   Next bit

      /* bit 2 */
      SEC                  // [1]   Used by ROR
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ROR   ,X             // [3]   Drive BDM high
      LSLA                 // [1]   Next bit

      /* bit 1 */
      SEC                  // [1]   Used by ROR
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ROR   ,X             // [3]   Drive BDM high
      LSLA                 // [1]   Next bit

      /* bit 0 */
      SEC                  // [1]   Used by ROR
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ROR   ,X             // [3]   Drive BDM high

      BDM_3STATE_ASM_TX    // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// (2,4,3)
void bdm_tx2(U8 data) {
   asm {
      LDHX  @DATA_PORT     // Point HX at DATA_PORT
      BDM_ENABLE_ASM_TX    // Enable BDM (high)
      
      /* bit 7 (MSB) */
      CLR   ,X             // [2]   Drive BDM low
                           // --- 3
      STA   ,X             // [2]   Drive data to BDM
                           // --- 2
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
                           // --- 4
      LSLA                 // [1]   Next bit
                           // ===========
      /* bit 6 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      LSLA                 // [1]   Next bit
      /* bit 5 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      LSLA                 // [1]   Next bit
      /* bit 4 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      LSLA                 // [1]   Next bit
      /* bit 3 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      LSLA                 // [1]   Next bit
      /* bit 2 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      LSLA                 // [1]   Next bit
      /* bit 1 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      LSLA                 // [1]   Next bit
      /* bit 0 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BDM_3STATE_ASM_TX    // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// (2,5,4)
void bdm_tx3(U8 data) {
   asm {
      LDHX  @DATA_PORT     // Point HX at DATA_PORT
      BDM_ENABLE_ASM_TX    // Enable BDM (high)
      
      /* bit 7 (MSB) */
      CLR   ,X             // [2]   Drive BDM low
                           // --- 4
      STA   ,X             // [2]   Drive data to BDM
                           // --- 2
      NOP                  // [1]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
                           // --- 5
      NOP                  // [1]
      LSLA                 // [1]   Next bit
                           // ===========
      /* bit 6 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      NOP                  // [1]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 5 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      NOP                  // [1]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 4 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      NOP                  // [1]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 3 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      NOP                  // [1]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 2 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      NOP                  // [1]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 1 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      NOP                  // [1]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 0 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      NOP                  // [1]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BDM_3STATE_ASM_TX    // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// (2,6,4)
void bdm_tx4(U8 data) {
   asm {
      BDM_ENABLE_ASM_TX    // Enable BDM
      CLRX                 // HX points to PTA = DATA_PORT
      CLRH
      /* bit 7 (MSB) */
      CLR   ,X             // [2]   Drive BDM low
                           // --- 4
      STA   ,X             // [2]   Drive data to BDM
                           // --- 2
      BIT   ,X             // [2]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
                           // --- 6
      NOP                  // [1]
      LSLA                 // [1]   Next bit
                           // ===========
      /* bit 6 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BIT   ,X             // [2]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 5 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BIT   ,X             // [2]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 4 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BIT   ,X             // [2]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 3 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BIT   ,X             // [2]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 2 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BIT   ,X             // [2]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 1 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BIT   ,X             // [2]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      NOP                  // [1]
      LSLA                 // [1]   Next bit
      /* bit 0 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BIT   ,X             // [2]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BDM_3STATE_ASM_TX    // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// (2,7,5)
void bdm_tx5(U8 data) {
   asm {
      BDM_ENABLE_ASM_TX    // Enable BDM
      CLRX                 // HX points to PTA = DATA_PORT
      CLRH
      /* bit 7 (MSB) */
      CLR   ,X             // [2]   Drive BDM low
                           // --- 5
      STA   ,X             // [2]   Drive data to BDM
                           // --- 2
      BRN   0              // [3]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
                           // --- 7
      BIT   ,X             // [2]
      LSLA                 // [1]   Next bit
                           // ===========
      /* bit 6 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BRN   0              // [3]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BIT   ,X             // [2]
      LSLA                 // [1]   Next bit
      /* bit 5 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BRN   0              // [3]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BIT   ,X             // [2]
      LSLA                 // [1]   Next bit
      /* bit 4 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BRN   0              // [3]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BIT   ,X             // [2]
      LSLA                 // [1]   Next bit
      /* bit 3 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BRN   0              // [3]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BIT   ,X             // [2]
      LSLA                 // [1]   Next bit
      /* bit 2 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BRN   0              // [3]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BIT   ,X             // [2]
      LSLA                 // [1]   Next bit
      /* bit 1 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BRN   0              // [3]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BIT   ,X             // [2]
      LSLA                 // [1]   Next bit
      /* bit 0 */
      CLR   ,X             // [2]   Drive BDM low
      STA   ,X             // [2]   Drive data to BDM
      BRN   0              // [3]
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
      BDM_3STATE_ASM_TX    // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// (2,8,5)
void bdm_tx6(U8 data) {
   asm {
      BDM_ENABLE_ASM_TX    // Enable BDM
      CLRX                 // HX points to PTA = DATA_PORT
      CLRH
      MOV   #8,bitCount    // # of bits to send
      BRA   IntoLoop       // Loop is rotated
      
   Loop:
      ROR   ,X             // [3]   Drive BDM high
                           // --- 8
      NOP                  // [1]
      LSLA                 // [1]   Next bit
   IntoLoop:
      SEC                  // [1]   This is used by ROR instruction!
      CLR   ,X             // [2]   Drive BDM low
                           // --- 5
      STA   ,X             // [2]   Drive data to BDM
                           // --- 2
      DBNZ  bitCount,Loop  // [5]   Count iteration
                           // ===========
      ROR   ,X             //       Drive BDM high
      BDM_3STATE_ASM_TX    // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// (2,9,5)
void bdm_tx7(U8 data) {
   asm {
      BDM_ENABLE_ASM_TX    // Enable BDM
      CLRX                 // HX points to PTA = DATA_PORT
      CLRH
      MOV   #8,bitCount    // # of bits to send
      BRA   IntoLoop       // Loop is rotated
      
   Loop:
      ORA   #BDM_OUT_MASK  // [2]
      STA   ,X             // [2]   Drive BDM high
                           // --- 9
      BIT   ,X             // [2]
      LSLA                 // [1]   Next bit
   IntoLoop:
      CLR   ,X             // [2]   Drive BDM low
                           // --- 5
      STA   ,X             // [2]   Drive data to BDM
                           // --- 2
      DBNZ  bitCount,Loop  // [5]   Count iteration
                           // ===========
      ORA   #BDM_OUT_MASK  //
      STA   ,X             //       Drive BDM high
      BDM_3STATE_ASM_TX    // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// (3,10,6)
void bdm_tx8(U8 data) {
   asm {
      BDM_ENABLE_ASM_TX                // Enable BDM
      LDX   #8                         // # of bits to send   
  Loop:      
      BSET  BDM_OUT_BIT,DATA_PORT      // [3]   Drive BDM high
                                       // --- 10
                                       // [1]
      NOP                              // [1]
      NOP                              // [1]
      BCLR  BDM_OUT_BIT,DATA_PORT      // [3]   Drive BDM low
                                       // --- 6
                                       // [1]
      STA   DATA_PORT                  // [2]   Drive data to BDM
                                       // --- 3
                                       // [1]
      NOP                              // [1]
      NOP                              // [1]
      LSLA                             // [1]   Next bit
                                       // ===========
      DBNZX Loop                       // [3]
      
      BSET  BDM_OUT_BIT,DATA_PORT      // Drive BDM high
      BDM_3STATE_ASM_TX                // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// (3,12,7)
void bdm_tx9(U8 data) {
   asm {
      BDM_ENABLE_ASM_TX                // Enable BDM
      LDX   #8                         // # of bits to send   

   Loop:      
      BCLR  BDM_OUT_BIT,DATA_PORT      // [3]   Drive BDM low
                                       // --- 7
                                       // [1]
      STA   DATA_PORT                  // [2]   Drive data to BDM
                                       // --- 3
                                       // [1]
      LSLA                             // [1]   Next bit
      NOP                              // [1]
      BRN   0                          // [3]
      BRN   0                          // [3]
      BSET  BDM_OUT_BIT,DATA_PORT      // [3]   Drive BDM high
                                       // --- 12
                                       // [1]
      DBNZX Loop                       // [3]
                                       // ===========
      BDM_3STATE_ASM_TX                // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// (4,13,8)
void bdm_tx10(U8 data) {
   asm {
      BDM_ENABLE_ASM_TX                // Enable BDM
      LDX   #8                         // # of bits to send   

  Loop:      
      NOP                              // [1]
      BCLR  BDM_OUT_BIT,DATA_PORT      // [3]   Drive BDM low
                                       // --- 8
                                       // [1]
      NOP                              // [1]
      STA   DATA_PORT                  // [2]   Drive data to BDM
                                       // --- 4
                                       // [1]
      LSLA                             // [1]   Next bit
      NOP                              // [1]
      NOP                              // [1]
      BRN   0                          // [3]
      BRN   0                          // [3]
      BSET  BDM_OUT_BIT,DATA_PORT      // [3]   Drive BDM high
                                       // --- 13
                                       // [1]
      DBNZX Loop                       // [3]
                                       // ===========
      BDM_3STATE_ASM_TX                // [4]   3-state BDM         
   }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}

//=========================================================================
// >=9,>=11,>=15
void bdm_txGeneric(U8 data) {
   asm {
      BDM_ENABLE_ASM_TX             // Enable BDM (high)
      MOV  #8,bitCount              // # of bits to send

   Loop:
      LDX   txTiming3               // [3      ]
      DBNZX *+0                     // [3n     ]
      MOV   #0,DATA_PORT            // [4  ppwp]   Drive BDM low
                                    // --- 12+3n   (>=15)
      LDX   txTiming1               // [3      ]
      DBNZX *+0                     // [3n     ]
      STA   DATA_PORT               // [3   pwp]   Drive data to BDM
                                    // ---  6+3n   (>=9)
      LDX   txTiming2               // [3      ]
      DBNZX *+0                     // [3n     ]
      LSLA                          // [1      ]   Next bit
      MOV   #BDM_OUT_MASK,DATA_PORT // [4  ppwp]   Drive BDM high
                                    // ---  8+3n   (>=11)
      DBNZ  bitCount,Loop           // [5      ]
                                    // ===========
      BDM_3STATE_ASM_TX             // [4]   3-state BDM         
      }
   IC_BDM_TIMING_TSC0_CH0F = 0;    /* clear ACKN flag */
}
#pragma MESSAGE DEFAULT C5703 // Restore warnings about unused parameter


//=================================================================================
// DATA_PORT values for Rx Routines - assumes BDM_DIR_Rx controls BKGD enable/direction
#define RxBDM_LOW         (BDM_DIR_Rx_WR|0)  // BDM_OUT=1 and enabled
#define RxBDM_3_STATE     (BDM_DIR_Rx_RD|BDM_OUT_MASK)  // BDM_OUT=1 as complement of value is used in code!


//==============================================================
//  Rx Routines bdm_rx..
//==============================================================
//  Receive 8 bit of data, MSB first
//  These routines assume the following call bdm_txFinish:
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

//=======================================================================
// bdm_rx1() (2,2,6)
U8 bdm_rx1(void) {
#pragma NO_RETURN
   asm {
      LDHX  @DATA_PORT        // Point HX at DATA_PORT
 
      LDA   #RxBDM_3_STATE       // Writing A to DATA_PORT 3-states BDM
      STA   ,X                   // 3-state BKGD pin

      /* bit 7 (MSB) */
      COM   ,X                // [3   prw]   Drive BDM low
                              // --- 6
      STA   ,X                // [2    pw]   BDM 3-state
                              // --- 2
      LDX   ,X                // [2    pr]   Sample BDM_IN
                              // --- 2
      PSHX                    // [2      ]   Save bit on stack
      CLRX                    // [1      ]   Clear X again
      /* bit 6 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 5 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 4 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 3 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 2 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 1 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 0 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      LDX   ,X                // [2]   Sample BDM_IN
      // now get the bit values (last value is in X, previous 7 on stack)
      JMP   rxStackDecode
   }
}

//======================================================================
// bdm_rx2() (2,3,6)
U8 bdm_rx2(void) {
#pragma NO_RETURN
   asm {
      LDHX  @DATA_PORT        // Point HX at DATA_PORT
      
      LDA   #RxBDM_3_STATE    // writing A to PTA 3-states BDM
      STA   ,X                // BDM 3-state
      /* bit 7 (MSB) */
      COM   ,X                // [3]   Drive BDM low
                              // --- 6
      STA   ,X                // [2]   BDM 3-state
                              // --- 2
      NOP                     // [1]       
      LDX   ,X                // [2]   Sample BDM_IN
                              // --- 3
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 6 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      NOP                     // [1]       
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 5 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      NOP                     // [1]       
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 4 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      NOP                     // [1]       
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 3 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      NOP                     // [1]       
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 2 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      NOP                     // [1]       
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 1 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      NOP                     // [1]       
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 0 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      NOP                     // [1]       
      LDX   ,X                // [2]   Sample BDM_IN
      /* now get the bit values (last value is in X, previous 7 on stack) */
      JMP   rxStackDecode
   }
}

//======================================================================
// bdm_rx3() (2,4,6)
U8 bdm_rx3(void) {
#pragma NO_RETURN
   asm {
      LDHX  @DATA_PORT        // Point HX at DATA_PORT
      LDA   #RxBDM_3_STATE    // writing A to PTA 3-states BDM
      STA   ,X                // BDM 3-state
      /* bit 7 (MSB) */
      COM   ,X                // [3]   Drive BDM low
                              // --- 6
      STA   ,X                // [2]   BDM 3-state
                              // --- 2
      BIT   ,X                // [2]
      LDX   ,X                // [2]   Sample BDM_IN
                              // --- 4
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 6 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BIT   ,X                // [2]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 5 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BIT   ,X                // [2]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 4 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BIT   ,X                // [2]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 3 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BIT   ,X                // [2]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 2 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BIT   ,X                // [2]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 1 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BIT   ,X                // [2]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 0 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BIT   ,X                // [2]
      LDX   ,X                // [2]   Sample BDM_IN
      /* now get the bit values (last value is in X, previous 7 on stack) */
      JMP   rxStackDecode
   }
}

//======================================================================
// bdm_rx4() (2,5,6)
U8 bdm_rx4(void) {
#pragma NO_RETURN
   asm {
      LDHX  @DATA_PORT        // Point HX at DATA_PORT
      LDA   #RxBDM_3_STATE    // writing A to PTA 3-states BDM
                              // writing ~A to PTA drives BDM low
      STA   ,X                // BDM 3-state
      /* bit 7 (MSB) */
      COM   ,X                // [3]   Drive BDM low
                              // --- 6
      STA   ,X                // [2]   BDM 3-state
                              // --- 2
      BRN   0                 // [3]
      LDX   ,X                // [2]   Sample BDM_IN
                              // --- 5
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 6 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BRN   0                 // [3]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 5 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BRN   0                 // [3]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 4 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BRN   0                 // [3]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 3 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BRN   0                 // [3]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 2 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BRN   0                 // [3]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 1 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BRN   0                 // [3]
      LDX   ,X                // [2]   Sample BDM_IN
      PSHX                    // [2]   Save bit on stack
      CLRX                    // [1]   Clear X again
      /* bit 0 */                
      COM   ,X                // [3]   Drive BDM low
      STA   ,X                // [2]   BDM 3-state
      BRN   0                 // [3]
      LDX   ,X                // [2]   Sample BDM_IN
      /* now get the bit values (last value is in X, previous 7 on stack) */
      JMP   rxStackDecode
   }
}

//======================================================================
// bdm_rx5() (3,5,7)
U8 bdm_rx5(void) {
   asm {
      LDHX  @DATA_PORT                 // Point HX at DATA_PORT
      MOV   #RxBDM_3_STATE,DATA_PORT   // BDM 3-state
      
      /* bit 7 (MSB) */
      COM   ,X                         // [3]   Drive BDM low
                                       // --- 7
      COM   ,X                         // [3]   BDM 3-state
                                       // --- 3
      BRN   0                          // [3]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2]   Sample BDM_IN
                                       // --- 5
                                       // [3]
      ROLA                             // [1]   Save bit in A
      /* bit 6 */                
      COM   ,X                         // [3]   Drive BDM low
      COM   ,X                         // [3]   BDM 3-state
      BRN   0                          // [3]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2+3] Sample BDM_IN
      ROLA                             // [1]   Save bit in A
      /* bit 5 */                
      COM   ,X                         // [3]   Drive BDM low
      COM   ,X                         // [3]   BDM 3-state
      BRN   0                          // [3]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2+3] Sample BDM_IN
      ROLA                             // [1]   Save bit in A
      /* bit 4 */                
      COM   ,X                         // [3]   Drive BDM low
      COM   ,X                         // [3]   BDM 3-state
      BRN   0                          // [3]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2+3] Sample BDM_IN
      ROLA                             // [1]   Save bit in A
      /* bit 3 */                
      COM   ,X                         // [3]   Drive BDM low
      COM   ,X                         // [3]   BDM 3-state
      BRN   0                          // [3]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2+3] Sample BDM_IN
      ROLA                             // [1]   Save bit in A
      /* bit 2 */                
      COM   ,X                         // [3]   Drive BDM low
      COM   ,X                         // [3]   BDM 3-state
      BRN   0                          // [3]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2+3] Sample BDM_IN
      ROLA                             // [1]   Save bit in A
      /* bit 1 */                
      COM   ,X                         // [3]   Drive BDM low
      COM   ,X                         // [3]   BDM 3-state
      BRN   0                          // [3]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2+3] Sample BDM_IN
      ROLA                             // [1]   Save bit in A
      /* bit 0 */                
      COM   ,X                         // [3]   Drive BDM low
      COM   ,X                         // [3]   BDM 3-state
      BRN   0                          // [3]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2+3] Sample BDM_IN
      ROLA                             // [1]   Save bit in A
   }
}

//======================================================================
// bdm_rx6() (3,7,7)
U8 bdm_rx6(void) {
   asm {
      LDHX  @DATA_PORT                 // Point HX at DATA_PORT
      MOV   #RxBDM_3_STATE,DATA_PORT   // BDM 3-state

      MOV   #8,bitCount                // # of iterations
      BRA   IntoLoop                   // Loop is rotated

      /* bits 7 down to 0 */
   Loop:
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2]   Sample BDM_IN
                                       // --- 7
                                       // [3]
      ROLA                             // [1]   Save bit in A
   IntoLoop:
      COM   ,X                         // [3]   Drive BDM low
                                       // --- 7
      COM   ,X                         // [3]   BDM 3-state
                                       // --- 3
      DBNZ  bitCount,Loop              // [5]   Count iteration

      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2]   Sample BDM_IN
                                       // --- 7
      ROLA                             //       Save bit in A
      }
}

//======================================================================
// bdm_rx7() (3,9,10)
U8 bdm_rx7(void) {
   asm {
      LDHX  @DATA_PORT                 // Point HX at DATA_PORT
      MOV   #RxBDM_3_STATE,DATA_PORT   // BDM 3-state

      MOV   #8,bitCount                // # of iterations
      BRA   IntoLoop                   // Loop is rotated

      /* bits 7 down to 0 */
   Loop:
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2]   Sample BDM_IN
                                       // --- 9
                                       // [3]
      BRN   0                          // [3]
      ROLA                             // [1]   Save bit in A
   IntoLoop:
      COM   ,X                         // [3]   Drive BDM low
                                       // --- 10
      COM   ,X                         // [3]   BDM 3-state
                                       // --- 3
      BIT   ,X                         // [2]
      DBNZ  bitCount,Loop              // [5]   Count iteration

      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2]   Sample last BDM_IN
                                       // --- 9
      ROLA                             //       Save bit in A
      }
}

//======================================================================
// bdm_rx8() (4,11,12)
U8 bdm_rx8(void) {
   asm {
      LDHX  @DATA_PORT                 // Point HX at DATA_PORT
      MOV   #RxBDM_3_STATE,DATA_PORT   // BDM 3-state

      MOV   #8,bitCount                // # of iterations

      /* bits 7 down to 0 */
   Loop:
      COM   ,X                         // [3]   Drive BDM low
                                       // --- 12
      NOP                              // [1]
      COM   ,X                         // [3]   BDM 3-state
                                       // --- 4
      NOP                              // [1]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2]   Sample BDM_IN
                                       // --- 11
                                       // [3]
      ROLA                             // [1]   Save bit in A
      DBNZ  bitCount,Loop              // [5]   Count iteration
      }
}

//======================================================================
// bdm_rx9() (4,14,14)
U8 bdm_rx9(void) {
   asm {
      LDHX  @DATA_PORT                 // Point HX at DATA_PORT
      MOV   #RxBDM_3_STATE,DATA_PORT   // BDM 3-state

      MOV   #8,bitCount                // # of iterations

      /* bits 7 down to 0 */
   Loop:
      NOP                              // [1]
      COM   ,X                         // [3]   Drive BDM low
                                       // --- 14
      NOP                              // [1]
      COM   ,X                         // [3]   BDM 3-state
                                       // --- 4
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2]   Sample BDM_IN
                                       // --- 14
                                       // [3]
      NOP                              // [1]
      ROLA                             // [1]   Save bit in A
      DBNZ  bitCount,Loop              // [5]   Count iteration
   }
}

//======================================================================
// bdm_rx10() (5,16,16)
U8 bdm_rx10(void) {
   asm {
      LDHX  @DATA_PORT                 // Point HX at DATA_PORT
      MOV   #RxBDM_3_STATE,DATA_PORT   // BDM 3-state

      MOV   #8,bitCount                // # of iterations

      /* bits 7 down to 0 */
   Loop:
      COM   ,X                         // [3]   Drive BDM low
                                       // --- 16
      BIT   ,X                         // [2]
      COM   ,X                         // [3]   BDM 3-state
                                       // --- 5
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      BRSET BDM_IN_BIT,DATA_PORT,*+3   // [2]   Sample BDM_IN
                                       // --- 16
                                       // [3]
      BIT   ,X                         // [2]
      BIT   ,X                         // [2]
      ROLA                             // [1]   Save bit in A
      DBNZ  bitCount,Loop              // [5]   Count iteration
   }
}

//=======================================================================
// >7,>9,>19
U8 bdm_rxGeneric(void) {
   asm {
      LDA   #0x01                    // Value used as sentinel
      
   Loop:
      LDX   rxTiming1                // [3      ]
      MOV   #RxBDM_LOW,DATA_PORT     // [4  ppwp]   Drive BKGD low
                                     // ---  16+3n
      DBNZX *+0                      // [3n     ]
      MOV   #RxBDM_3_STATE,DATA_PORT // [4  ppwp]   3-state BKGD pin
                                     // ---  4+3n
      LDX   rxTiming2                // [3      ]  
      DBNZX *+0                      // [3n     ]
      BRSET BDM_IN_BIT,DATA_PORT,*+3 // [3   prp]   Sample BDM_IN
                                     // ---  6+3n
                                     // [2+   dp] 
      LDX   rxTiming3                // [3      ]
      DBNZX *+0                      // [3n     ]
      ROLA                           // [1      ]   Save sample
      BCC   Loop                     // [3      ]
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
   void  (*txFunc)(U8 data);  //!< Ptr to selected function
   U8    time1,time2,time3;   //!< Timing Parameters for function use
} TxConfiguration;

//! Information for each Tx configuration
//!
const TxConfiguration txConfiguration[] =
{
{ 0, bdm_txEmpty, 0, 0, 0 },//>42 MHz - Max Fequency
{ 366, bdm_tx0, 0, 0, 0 },//33 - 42 MHz, (2,2,12)
{ 461, bdm_tx1, 1, 0, 0 },//26.4 - 33.6 MHz, (2,3,4)
{ 565, bdm_tx2, 2, 0, 0 },//22 - 28 MHz, (2,4,3)
{ 668, bdm_tx3, 3, 0, 0 },//18.86 - 24 MHz, (2,5,4)
{ 772, bdm_tx4, 4, 0, 0 },//16.5 - 21 MHz, (2,6,4)
{ 875, bdm_tx5, 5, 0, 0 },//14.66 - 18.66 MHz, (2,7,5)
{ 979, bdm_tx6, 6, 0, 0 },//13.2 - 16.8 MHz, (2,8,5)
{ 1082, bdm_tx7, 7, 0, 0 },//12 - 15.28 MHz, (2,9,5)
{ 1234, bdm_tx8, 8, 0, 0 },//10.16 - 12.92 MHz, (3,10,6)
{ 1440, bdm_tx9, 9, 0, 0 },//8.8 - 11.2 MHz, (3,12,7)
{ 1647, bdm_tx10, 10, 0, 0 },//7.76 - 9.88 MHz, (4,13,8)
{ 1902, bdm_txGeneric, 1, 1, 1 },//6.6 - 8.4 MHz, (9,11,15)
{ 2213, bdm_txGeneric, 1, 2, 1 },//5.74 - 7.3 MHz, (9,14,15)
{ 2522, bdm_txGeneric, 1, 3, 1 },//5.08 - 6.46 MHz, (9,17,15)
{ 2830, bdm_txGeneric, 1, 4, 1 },//4.56 - 5.8 MHz, (9,20,15)
{ 3136, bdm_txGeneric, 1, 5, 1 },//4.12 - 5.26 MHz, (9,23,15)
{ 3454, bdm_txGeneric, 1, 6, 2 },//3.78 - 4.8 MHz, (9,26,18)
{ 3758, bdm_txGeneric, 1, 7, 2 },//3.48 - 4.42 MHz, (9,29,18)
{ 4213, bdm_txGeneric, 1, 9, 3 },//3 - 3.82 MHz, (9,35,21)
{ 4687, bdm_txGeneric, 2, 9, 4 },//2.8 - 3.58 MHz, (12,35,24)
{ 5164, bdm_txGeneric, 2, 11, 5 },//2.5 - 3.16 MHz, (12,41,27)
{ 5765, bdm_txGeneric, 3, 12, 5 },//2.24 - 2.84 MHz, (15,44,27)
{ 6517, bdm_txGeneric, 4, 14, 7 },//1.94 - 2.48 MHz, (18,50,33)
{ 7469, bdm_txGeneric, 5, 16, 8 },//1.72 - 2.18 MHz, (21,56,36)
{ 8366, bdm_txGeneric, 7, 17, 10 },//1.54 - 1.96 MHz, (27,59,42)
{ 9438, bdm_txGeneric, 10, 18, 11 },//1.34 - 1.72 MHz, (36,62,45)
{ 10907, bdm_txGeneric, 13, 20, 14 },//1.16 - 1.48 MHz, (45,68,54)
{ 12605, bdm_txGeneric, 17, 22, 16 },//1 - 1.28 MHz, (57,74,60)
{ 14645, bdm_txGeneric, 22, 24, 20 },//0.86 - 1.1 MHz, (72,80,72)
{ 16905, bdm_txGeneric, 28, 26, 23 },//0.76 - 0.96 MHz, (90,86,81)
{ 19224, bdm_txGeneric, 34, 28, 27 },//0.66 - 0.84 MHz, (108,92,93)
{ 21979, bdm_txGeneric, 40, 32, 31 },//0.58 - 0.74 MHz, (126,104,105)
{ 26483, bdm_txEmpty, 0, 0 ,0  },//<0.58 MHz - Min. Frequency
};

//! Structure describing Rx configuration
typedef struct {
   U16   syncThreshold;       //!< Threshold to use this function
   U8    (*txFunc)(void);     //!< Ptr to selected function
   U8    time1,time2,time3;   //!< Timing Parameters for function use
} RxConfiguration;

//! Information for each Rx configuration
//!
const RxConfiguration rxConfiguration[] =
{
{ 0, bdm_rxEmpty, 0, 0, 0 },//>41.2 MHz - Max Fequency
{ 384, bdm_rx1, 0, 0, 0 },//38.06 - 41.2 MHz, (2,2,6) - out of spec
{ 404, bdm_rx1, 1, 0, 0 },//28.54 - 38.06 MHz, (2,2,6)
{ 524, bdm_rx2, 2, 0, 0 },//22.58 - 30.1 MHz, (2,3,6)
{ 648, bdm_rx3, 3, 0, 0 },//18.68 - 24.9 MHz, (2,4,6)
{ 771, bdm_rx4, 4, 0, 0 },//15.92 - 21.22 MHz, (2,5,6)
{ 895, bdm_rx5, 5, 0, 0 },//13.88 - 18.5 MHz, (3,5,7)
{ 1075, bdm_rx6, 6, 0, 0 },//11.3 - 14.72 MHz, (3,7,7)
{ 1307, bdm_rx7, 7, 0, 0 },//9.16 - 12.22 MHz, (3,9,10)
{ 1626, bdm_rx8, 8, 0, 0 },//7.3 - 9.74 MHz, (4,11,12)
{ 1998, bdm_rx9, 9, 0, 0 },//6.08 - 8.1 MHz, (4,14,14)
{ 2368, bdm_rx10, 10, 0, 0 },//5.2 - 6.92 MHz, (5,16,18)
{ 2792, bdm_rxGeneric, 1, 4, 1 },//4.36 - 5.82 MHz, (7,18,19)
{ 3232, bdm_rxGeneric, 1, 5, 2 },//3.88 - 5.18 MHz, (7,21,22)
{ 3605, bdm_rxGeneric, 1, 6, 3 },//3.5 - 4.68 MHz, (7,24,25)
{ 3978, bdm_rxGeneric, 1, 7, 3 },//3.42 - 4.26 MHz, (7,27,25)
{ 4195, bdm_rxGeneric, 2, 7, 5 },//2.94 - 3.92 MHz, (10,27,31)
{ 4887, bdm_rxGeneric, 3, 8, 6 },//2.52 - 3.36 MHz, (13,30,34)
{ 5624, bdm_rxGeneric, 4, 9, 8 },//2.22 - 2.96 MHz, (16,33,40)
{ 6369, bdm_rxGeneric, 5, 10, 9 },//1.98 - 2.62 MHz, (19,36,43)
{ 7106, bdm_rxGeneric, 6, 11, 11 },//1.78 - 2.36 MHz, (22,39,49)
{ 8021, bdm_rxGeneric, 8, 12, 13 },//1.54 - 2.06 MHz, (28,42,55)
{ 9175, bdm_rxGeneric, 9, 14, 15 },//1.38 - 1.82 MHz, (31,48,61)
{ 10210, bdm_rxGeneric, 10, 16, 18 },//1.24 - 1.64 MHz, (34,54,70)
{ 11338, bdm_rxGeneric, 11, 18, 20 },//1.12 - 1.48 MHz, (37,60,76)
{ 12446, bdm_rxGeneric, 12, 20, 22 },//1.02 - 1.36 MHz, (40,66,82)
{ 13769, bdm_rxGeneric, 14, 22, 26 },//0.92 - 1.22 MHz, (46,72,94)
{ 15269, bdm_rxGeneric, 16, 24, 30 },//0.84 - 1.1 MHz, (52,78,106)
{ 16594, bdm_rxGeneric, 18, 26, 32 },//0.76 - 1.02 MHz, (58,84,112)
{ 18572, bdm_rxGeneric, 21, 29, 36 },//0.68 - 0.9 MHz, (67,93,124)
{ 20825, bdm_rxGeneric, 24, 33, 42 },//0.6 - 0.8 MHz, (76,105,142)
{ 23369, bdm_rxGeneric, 28, 36, 48 },//0.54 - 0.72 MHz, (88,114,160)
{ 28444, bdm_rxEmpty, 0, 0 ,0  },//<0.54 MHz - Min. Frequency
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
   const TxConfiguration  * far txConfigPtr;
   const RxConfiguration  * far rxConfigPtr;

   bdm_rx_ptr = bdm_rxEmpty; // clear the Tx/Rx pointers
   bdm_tx_ptr = bdm_txEmpty; // i.e. no routines found

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
   if (bdm_tx_ptr==bdm_txEmpty) // Return if no function found
      return(BDM_RC_NO_TX_ROUTINE);

   for (  rxConfigPtr  = rxConfiguration+sizeof(rxConfiguration)/sizeof(rxConfiguration[0]);
        --rxConfigPtr >= rxConfiguration; ) { // Search the table

      if (cable_status.sync_length >= rxConfigPtr->syncThreshold) { // SYNC is >=
         bdm_rx_ptr    = rxConfigPtr->txFunc; // Select this routine
         rxTiming1     = rxConfigPtr->time1;  // Save timing parameters
         rxTiming2     = rxConfigPtr->time2;
         rxTiming3     = rxConfigPtr->time3;
         break;                               // Quit search
      }
   }
   if (bdm_rx_ptr==bdm_rxEmpty) // Return if no function found
      return(BDM_RC_NO_RX_ROUTINE);

   // Calculate number of iterations for manual delay (each iteration is 8 cycles)
   cable_status.wait64_cnt  = cable_status.sync_length/(U16)(((8*60*128UL)/(BUS_FREQ/1000000)/64));
   cable_status.wait150_cnt = cable_status.sync_length/(U16)(((8*60*128UL)/(BUS_FREQ/1000000)/150));
   // Correct for overhead in calling function etc. (JSR+RTS+JSR) = (5+4+5) ~ 2 iterations
   if (cable_status.wait64_cnt<=2)
      cable_status.wait64_cnt = 1; // minimum of 1 iteration
   else
      cable_status.wait64_cnt -= 2;
   if (cable_status.wait150_cnt<=2)
      cable_status.wait150_cnt = 1; // minimum of 1 iteration
   else
      cable_status.wait150_cnt -= 2;
   return(0);
}

// PARTID read from HCS12 - used to confirm target connection speed and avoid needless probing
static U16 partid = 0x00;

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
   if (rc != BDM_RC_OK)
      goto tidyUp;

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
      // in special mode.
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
   cable_status.speed        = SPEED_NO_INFO; // Connection cannot be established at this speed
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
const TxConfiguration  * far txConfigPtr;
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
      if (rc == BDM_RC_OK)
         break;
      currentGuess = typicalSpeeds[sub];
      rc           = bdmHC12_confirmSpeed(currentGuess);
      }

   // Try each Tx driver BDM frequency
   for (  txConfigPtr  = txConfiguration+(sizeof(txConfiguration)/sizeof(txConfiguration[0])-1);
        --txConfigPtr >= txConfiguration; ) { // Search the table
      if (rc == BDM_RC_OK)
         break;
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

   cable_status.speed = SPEED_NO_INFO;  // Speed not found
   return rc;
}

#if (DEBUG&DEBUG_COMMANDS) // Debug commands enabled
U8 bdm_testTx(U8 speedIndex) {
const TxConfiguration  * far txConfigPtr;

    // Validate index
   if (speedIndex > (sizeof(txConfiguration)/sizeof(txConfiguration[0])))
      return BDM_RC_ILLEGAL_PARAMS;
         
   txConfigPtr = &txConfiguration[speedIndex]; // selected routine

   bdm_tx_ptr    = txConfigPtr->txFunc; // Select this routine
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
#if (HW_CAPABILITY&CAP_RST_IO) // Requires RESET driver!!

   DIR_PORT_DDR  = 0;     // BKGD direction controlled by DATA_PORT
   DATA_PORT_DDR = BDM_OUT_MASK|BDM_DIR_Rx_MASK;     // BKGD pin is driven, Reset pin is driven
   RESET_OUT_DDR = 1;

#if (TARGET_HARDWARE==H_USBDM) // USBDM version
   asm {
      LDHX   @DATA_PORT                                // Point X at DATA_PORT

   Loop1:
      LDA   #BDM_OUT_MASK|BDM_DIR_Rx_WR                // [2      ] Mask for BDM high, RESET low
      NOP                                              // [1      ]
   Loop:
      BRN   *+0                                        // [3      ] Waste some time
      BRN   *+0                                        // [3      ]
      BRN   *+0                                        // [3      ]
      BRN   *+0                                        // [3      ]
      
      CLR   ,X                                         // [2    Pw] Drive BKGD,RESET low
                                                       // ---------
      STA   ,X                                         // [2    Pw] Drive BKGD high
                                                       // --------- 2 cycles low
      BSET  BDM_DIR_Rx_BIT,DATA_PORT                   // [4  prwp] 3-state BKGD pin
      NOP                                              // [1      ]
      LDA   ,X                                         // [2    pr] Sample BDM_IN
                                                       // --------- 9 cycles from BKGD low
      AND   #BDM_IN_MASK                               // [1      ] Input high?
      BEQ   Loop1                                      // [3/1    ] No - set RESET low

      LDA   #BDM_OUT_MASK|BDM_DIR_Rx_WR|RESET_OUT_MASK // [  2    ] Mask for BDM high, RESET High
      BRA   Loop                                       // [  3    ] 
      }   
#elif ((TARGET_HARDWARE==H_WTBDM) && 0) // Witztronic's version broken
   asm {
      LDHX   @DATA_PORT                                // Point X at DATA_PORT

   Loop1:
      LDA   #BDM_OUT_MASK|BDM_DIR_Rx_WR                // [2      ] Mask for BDM high, RESET low
      NOP                                              // [1      ]
   Loop:
      BRN   *+0                                        // [3      ] Waste some time
      BRN   *+0                                        // [3      ]
      BRN   *+0                                        // [3      ]
      BRN   *+0                                        // [3      ]

      CLR   ,X                                         // [2    Pw] Drive BKGD,RESET low
                                                       // ---------
      STA   ,X                                         // [2    Pw] Drive BKGD high
                                                       // --------- 2 cycles low
      BCLR  BDM_DIR_Rx_BIT,DATA_PORT                   // [4  prwp] 3-state BKGD pin
      NOP                                              // [1      ]
      LDA   ,X                                         // [2    pr] Sample BDM_IN
                                                       // --------- 9 cycles from BKGD low
      AND   #BDM_IN_MASK                               // [1      ] Input high?
      BEQ   Loop1                                      // [3/1    ] No - set RESET low

      LDA   #BDM_OUT_MASK|BDM_DIR_Rx_WR|RESET_OUT_MASK // [  2    ] Mask for BDM high, RESET High
      BRA   Loop                                       // [  3    ] 
      }   
#endif  // (TARGET_HARDWARE==xxx)
#endif  // (CAPABILITY&CAP_RESET)
}

#endif // (DEBUG&DEBUG_COMMANDS) // Debug commands enabled

#endif (HW_CAPABILITY&CAP_BDM)
