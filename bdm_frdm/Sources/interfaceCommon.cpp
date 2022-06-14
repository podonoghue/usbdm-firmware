/*! \file  interfaceCommon.cpp
    \brief USBDM - Common interface routines.

   USBDM
   Copyright (C) 2016  Peter O'Donoghue

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
+================================================================================================
| 22 Nov 2011 | Kinetis version using C++ libraries                                    - pgo
+================================================================================================
\endverbatim
 */

#include <string.h>
#include <targetVddInterface.h>
#include "configure.h"
#include "interfaceCommon.h"
#include "interface.h"
#include "swd.h"
#if (HW_CAPABILITY&CAP_BDM)
#include "bdm.h"
#endif
#include "resetInterface.h"
#include "cmdProcessing.h"
#include "commands.h"
#include "console.h"

/** How long to wait for Target Vdd rise */
static constexpr uint32_t VDD_RISE_TIMEms = 100;

/** How long to wait for Target Vdd fall */
static constexpr uint32_t VDD_FALL_TIMEms = 500;

#if HW_CAPABILITY & CAP_RST_IN
/** How long to wait for Reset rise after Vdd on etc */
static constexpr uint32_t RESET_RISE_TIMEus = 100;
#endif

/** How long to wait for BKGD rise after Reset etc */
static constexpr uint32_t BKGD_WAITus = 100;

/** How long to wait after RESET rise before continuing */
static constexpr uint32_t RESET_RECOVERYms = 10;

//=========================================================================
// Target power control
//
//=========================================================================

/*
 * Checks Target Vdd
 *
 * @return BDM_RC_OK              Target Vdd present or no target Vdd sensing
 * @return BDM_RC_VDD_NOT_PRESENT Target Vdd missing
 */
USBDM_ErrorCode checkTargetVdd(void) {
#if (HW_CAPABILITY&(CAP_VDDSENSE|CAP_VDDCONTROL))
   switch(TargetVddInterface::checkVddState()) {
      case VddState_Error:
      case VddState_None:
         return BDM_RC_VDD_NOT_PRESENT;
      case VddState_External:
      case VddState_Internal:
      default:
         return BDM_RC_OK;
         break;
   }
#else
   // No target Vdd sensing - assume external Vdd is present
   return BDM_RC_OK;
#endif // CAP_VDDSENSE
}

#if (HW_CAPABILITY&CAP_VDDCONTROL)
/**
 *  Sets Target Vdd
 *  Checks for Vdd present.
 *
 * @return BDM_RC_OK                => Target Vdd confirmed on target
 * @return BDM_RC_VDD_NOT_PRESENT   => Target Vdd not present
 */
USBDM_ErrorCode setTargetVdd(TargetVddSelect_t targetVdd) {

   if (targetVdd == BDM_TARGET_VDD_ENABLE) {
      // Enable at previously set level
      targetVdd = bdm_option.targetVdd;
   }
   USBDM_ErrorCode rc = BDM_RC_OK;
   switch (targetVdd) {
      case BDM_TARGET_VDD_DISABLE:
      case BDM_TARGET_VDD_OFF :
         TargetVddInterface::vddOff();
         // Wait for Vdd to fall
         if (!USBDM::waitMS(VDD_FALL_TIMEms, TargetVddInterface::isVddLow)) {
            rc = BDM_RC_VDD_NOT_REMOVED;
         }
         break;
      case BDM_TARGET_VDD_3V3 :
         TargetVddInterface::vdd3V3On();
         // Wait for Vdd to rise
         if (!USBDM::waitMS(VDD_RISE_TIMEms, TargetVddInterface::isVddOK_3V3)) {
            // In case of Vdd overload
            TargetVddInterface::vddOff();
            rc = BDM_RC_VDD_NOT_PRESENT;
         }
         break;
      case BDM_TARGET_VDD_5V  :
         TargetVddInterface::vdd5VOn();
         // Wait for Vdd to rise
         if (!USBDM::waitMS(VDD_RISE_TIMEms, TargetVddInterface::isVddOK_5V)) {
            // In case of Vdd overload
            TargetVddInterface::vddOff();
            rc = BDM_RC_VDD_NOT_PRESENT;
         }
         break;
      default:
         // Should be impossible
         return BDM_RC_VDD_WRONG_MODE;
   }
   return (rc);
}

#endif // CAP_VDDCONTROL

/**
 *  Turns on Target Vdd (if internally controlled).
 *
 * @return BDM_RC_OK                => Target Vdd confirmed on target \n
 * @return BDM_RC_VDD_NOT_PRESENT   => Target Vdd not present
 */
USBDM_ErrorCode enableTargetVdd() {
   return setTargetVdd(bdm_option.targetVdd);
}

#if (HW_CAPABILITY&CAP_BDM)
/**
 *   Cycle power ON to target (T_HC12, T_HCS08, T_RS08, T_CFV1 only)
 *
 *  @param mode
 *     - \ref RESET_SPECIAL => Power on in special mode,
 *     - \ref RESET_NORMAL  => Power on in normal mode
 *
 *   If RESET_SPECIAL mode then BKGD is held low when power
 *   is re-applied to start target in BKGD active mode.
 *
 *   @return
 *    \ref BDM_RC_OK                   => Target Vdd confirmed on target \n
 *    \ref BDM_RC_VDD_WRONG_MODE       => Target Vdd not controlled by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_PRESENT      => Target Vdd failed to rise     \n
 *    \ref BDM_RC_RESET_TIMEOUT_RISE   => RESET signal failed to rise      \n
 *    \ref BDM_RC_BKGD_TIMEOUT         => BKGD signal failed to rise
 */
USBDM_ErrorCode cycleTargetVddOnBdm(TargetMode_t mode) {

   USBDM_ErrorCode rc = BDM_RC_OK;

   // Used to indicate doing power on sequence with BKGD low
   bool bkgdHeldLow = false;

   mode = (TargetMode_t)(mode&RESET_MODE_MASK);

   do {
      Bdm::initialise();
      if (mode == RESET_SPECIAL) {
         bkgdHeldLow = true;
         Bdm::setPinState(PIN_BKGD_LOW);
      }

      // Power on
      rc = enableTargetVdd();
      if (rc != BDM_RC_OK) {
         // No target Vdd
         continue;
      }

#if (HW_CAPABILITY&CAP_RST_IN)
      // RESET rise may be delayed by target POR
      if (bdm_option.useResetSignal) {
         USBDM::waitUS(RESET_RISE_TIMEus, ResetInterface::isHigh);
      }
#endif // (HW_CAPABILITY&CAP_RST_IN)

      // Let signals settle & CPU to finish reset (with BKGD possibly held low)
      USBDM::waitUS(BKGD_WAITus);

#if (HW_CAPABILITY&CAP_RST_IN)
      if (bdm_option.useResetSignal && ResetInterface::isLow()) {
         // RESET didn't rise
         rc = BDM_RC_RESET_TIMEOUT_RISE;
         continue;
      }
#endif // (HW_CAPABILITY&CAP_RST_IN)

      if (bkgdHeldLow) {
         // Release BKGD
         Bdm::setPinState(PinLevelMasks_t::PIN_BKGD_3STATE);
      }

      // Let processor start up
      USBDM::waitMS(RESET_RECOVERYms);

   } while (false);

   if (bkgdHeldLow) {
      Bdm::initialise();
   }

   return(rc);
}
#endif

#if (HW_CAPABILITY&CAP_SWD_HW)

/**
 *   Cycle power ON to target (ARM_SWD)
 *
 *   @return
 *    \ref BDM_RC_OK                   => Target Vdd confirmed on target \n
 *    \ref BDM_RC_VDD_WRONG_MODE       => Target Vdd not controlled by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_PRESENT      => Target Vdd failed to rise     \n
 *    \ref BDM_RC_RESET_TIMEOUT_RISE   => RESET signal failed to rise      \n
 *    \ref BDM_RC_BKGD_TIMEOUT         => BKGD signal failed to rise
 */
USBDM_ErrorCode cycleTargetVddOnSwd() {

   USBDM_ErrorCode rc = BDM_RC_OK;

   do {
      Swd::initialiseInterface();

      // Power on
      rc = enableTargetVdd();
      if (rc != BDM_RC_OK) {
         // No target Vdd
         continue;
      }

#if (HW_CAPABILITY&CAP_RST_IN)
      // RESET rise may be delayed by target POR
      if (bdm_option.useResetSignal) {
         USBDM::waitUS(RESET_RISE_TIMEus, ResetInterface::isHigh);
      }
#endif

      // Let signals settle & CPU to finish reset (with BKGD held low)
      USBDM::waitUS(BKGD_WAITus);

#if (HW_CAPABILITY&CAP_RST_IN)
      if (bdm_option.useResetSignal && ResetInterface::isLow()) {
         // RESET didn't rise
         rc = BDM_RC_RESET_TIMEOUT_RISE;
         continue;
      }
#endif

      // Let processor start up
      USBDM::waitMS(RESET_RECOVERYms);

   } while(false);

   return(rc);
}
#endif

#if (HW_CAPABILITY&CAP_CFVx_HW)
/**
 *   Cycle power ON to target (Coldfire V2/3)
 *
 *  @param mode
 *     - \ref RESET_SPECIAL => Power on in special mode,
 *     - \ref RESET_NORMAL  => Power on in normal mode
 *
 *   If RESET_SPECIAL mode then BKPT is held low when power
 *   is re-applied to start target in BKGD active mode.
 *
 *   @return
 *    \ref BDM_RC_OK                   => Target Vdd confirmed on target \n
 *    \ref BDM_RC_VDD_WRONG_MODE       => Target Vdd not controlled by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_PRESENT      => Target Vdd failed to rise     \n
 *    \ref BDM_RC_RESET_TIMEOUT_RISE   => RESET signal failed to rise      \n
 *    \ref BDM_RC_BKGD_TIMEOUT         => BKGD signal failed to rise
 */
USBDM_ErrorCode cycleTargetVddOnColdfireVx(TargetMode_t mode) {

   USBDM_ErrorCode rc = BDM_RC_OK;

   mode = (TargetMode_t)(mode&RESET_MODE_MASK);

   do {
      bdmcf_interfaceIdle();  // Make sure BDM interface is idle
      if (mode == RESET_SPECIAL)
         BKPT_LOW();

      // Power on
      enableTargetVdd();
      if (rc != BDM_RC_OK) {
         // No target Vdd
         continue;
      }

#if (HW_CAPABILITY&CAP_RST_IN)
   // RESET rise may be delayed by target POR
   if (bdm_option.useResetSignal) {
      USBDM::waitUS(RESET_RISE_TIMEus, ResetInterface::isHigh);
   }
#endif // (HW_CAPABILITY&CAP_RST_IN)

   // Let signals settle & CPU to finish reset (with BKGD held low)
   USBDM::waitUS(BKGD_WAITus);

#if (HW_CAPABILITY&CAP_RST_IN)
   if (bdm_option.useResetSignal && ResetInterface::isLow()) {
      // RESET didn't rise
      rc = BDM_RC_RESET_TIMEOUT_RISE;
      continue;
   }
#endif // (HW_CAPABILITY&CAP_RST_IN)

   bdmcf_interfaceIdle();  // Release BKPT etc

   // Let processor start up
   USBDM::waitMS(RESET_RECOVERYms);

   } while(false);

   return(rc);
}
#endif

#if (HW_CAPABILITY&CAP_JTAG_HW)
/**
 *   Cycle power ON to target (JTAG targets)
 *
 *   @return
 *    \ref BDM_RC_OK                	=> Target Vdd confirmed on target \n
 *    \ref BDM_RC_VDD_WRONG_MODE    	=> Target Vdd not controlled by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_PRESENT   	=> Target Vdd failed to rise 		\n
 *    \ref BDM_RC_RESET_TIMEOUT_RISE  	=> RESET signal failed to rise 		\n
 *    \ref BDM_RC_BKGD_TIMEOUT      	=> BKGD signal failed to rise
 */
USBDM_ErrorCode cycleTargetVddOnJtag() {

   USBDM_ErrorCode rc = BDM_RC_OK;

   mode = (TargetMode_t)(mode&RESET_MODE_MASK);

   do {
      jtag_interfaceIdle();  // Make sure BDM interface is idle

      // Power on
      enableTargetVdd();
      if (rc != BDM_RC_OK) {
         // No target Vdd
         continue;
      }

#if (HW_CAPABILITY&CAP_RST_IN)
      // RESET rise may be delayed by target POR
      if (bdm_option.useResetSignal) {
         USBDM::waitUS(RESET_RISE_TIMEus, ResetInterface::isHigh);
      }
#endif

      // Let signals settle & CPU to finish reset (with BKGD held low)
      USBDM::waitUS(BKGD_WAITus);

#if (HW_CAPABILITY&CAP_RST_IN)
      if (bdm_option.useResetSignal && ResetInterface::isLow()) {
         // RESET didn't rise
         rc = BDM_RC_RESET_TIMEOUT_RISE;
         continue;
#endif
      }

   } while (false);

   // Let processor start up
   USBDM::waitMS(RESET_RECOVERYms);

   return(rc);
}
#endif

/**
 *   Cycle power ON to target
 *
 *  @param mode
 *     - \ref RESET_SPECIAL => Power on in special mode,
 *     - \ref RESET_NORMAL  => Power on in normal mode
 *
 *   If RESET_SPECIAL, and HCS08 or CFV1 targets, BKGD/BKPT is held low when power
 *   is re-applied to start target in BKGD active mode.
 *
 *   @return
 *    \ref BDM_RC_OK                    => Target Vdd confirmed on target \n
 *    \ref BDM_RC_VDD_WRONG_MODE        => Target Vdd not controlled by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_PRESENT       => Target Vdd failed to rise   \n
 *    \ref BDM_RC_FEATURE_NOT_SUPPORTED => Target Vdd control not supported \n
 *    \ref BDM_RC_RESET_TIMEOUT_RISE    => RESET signal failed to rise \n
 *    \ref BDM_RC_BKGD_TIMEOUT          => BKGD signal failed to rise
 */
USBDM_ErrorCode cycleTargetVddOn(TargetMode_t mode) {
   (void) mode;

#if !(HW_CAPABILITY&CAP_VDDCONTROL)
   // No Vdd control
   return BDM_RC_FEATURE_NOT_SUPPORTED;
#endif

   switch(cable_status.target_type) {

#if (HW_CAPABILITY&CAP_BDM)
      case T_HC12:
      case T_HCS08:
      case T_RS08:
      case T_CFV1:
         return cycleTargetVddOnBdm(mode);
#endif

#if (HW_CAPABILITY&CAP_SWD_HW)
      case T_ARM_SWD:
         return cycleTargetVddOnSwd();
#endif

#if (HW_CAPABILITY&CAP_CFVx_HW)
      case T_CFVx:
         return cycleTargetVddOnColdfireVx(mode);
#endif

#if (HW_CAPABILITY&CAP_JTAG_HW)
      case T_JTAG:
      case T_MC56F80xx:
      case T_ARM_JTAG:
         return cycleTargetVddOnJtag();
#endif

      default:
         return BDM_RC_UNKNOWN_TARGET;
   }
}

/**
 *   Cycle power OFF to target
 *
 *   @return
 *    \ref BDM_RC_OK                => No error  \n
 *    \ref BDM_RC_VDD_WRONG_MODE    => Target Vdd not provided by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_REMOVED   => Target Vdd failed to fall \n
 */
USBDM_ErrorCode cycleTargetVddOff(void) {
   USBDM_ErrorCode rc = BDM_RC_OK;

#if (HW_CAPABILITY&CAP_VDDCONTROL)

   if(TargetVddInterface::checkVddState() != VddState_Internal) {
      return BDM_RC_VDD_WRONG_MODE;
   }

   /// Make sure interface is idle
   switch(cable_status.target_type) {
#if (HW_CAPABILITY&CAP_CFVx_HW)
      case T_CFVx:
         bdmcf_interfaceIdle();  // Make sure BDM interface is idle
         break;
#endif
#if (HW_CAPABILITY&CAP_BDM)
      case T_HC12:
      case T_HCS08:
      case T_RS08:
      case T_CFV1:
         Bdm::initialise();
         break;
#endif
#if (HW_CAPABILITY&CAP_JTAG_HW)
      case T_JTAG:
      case T_MC56F80xx:
      case T_ARM_JTAG:
         jtag_interfaceIdle();  // Make sure BDM interface is idle
         break;
#endif
      case T_ARM_SWD:
         Swd::initialiseInterface();
         break;

      default:
         break;
   }
   // Power off & wait for Vdd to fall
   TargetVddInterface::vddOff();
   if (!USBDM::waitMS(1000, TargetVddInterface::isVddLow)) {
      // Vdd didn't turn off!
      rc = BDM_RC_VDD_NOT_REMOVED;
   }

   // Wait a while with power off
   USBDM::waitUS(RESET_RECOVERYms);

   // Clear Vdd monitoring interrupt
   TargetVddInterface::clearVddChangeFlag();

#endif // CAP_VDDCONTROL

   return(rc);
}

/**
 *  Cycle power to target
 *
 *  @param mode
 *     - \ref RESET_SPECIAL => Power on in special mode,
 *     - \ref RESET_NORMAL  => Power on in normal mode
 *
 *   BKGD/BKPT is held low when power is re-applied to start
 *   target with BDM active if RESET_SPECIAL
 *
 *   @return
 *    \ref BDM_RC_OK                 => No error \n
 *    \ref BDM_RC_VDD_WRONG_MODE     => Target Vdd not provided by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_REMOVED    => Target Vdd failed to fall \n
 *    \ref BDM_RC_VDD_NOT_PRESENT    => Target Vdd failed to rise \n
 *    \ref BDM_RC_RESET_TIMEOUT_RISE => RESET signal failed to rise \n
 */
USBDM_ErrorCode cycleTargetVdd(TargetMode_t mode) {
   USBDM_ErrorCode rc;

   // TODO This may take a while
   //setBDMBusy();

   rc = cycleTargetVddOff();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   USBDM::waitMS(1000);
   rc = cycleTargetVddOn(mode);
   return rc;
}

/**
 *   Measures Target Vdd
 *
 *   @return 8-bit value representing the Target Vdd, N ~ (N/255) * 5V \n
 *   @note This routine is a dummy if no measurement hardware is available.
 */
uint16_t targetVddMeasure(void) {
   return TargetVddInterface::readRawVoltage();
}

//=========================================================================
// Common BDM routines
//
//=========================================================================

/**   Sets the BDM interface to a suspended state
 *
 *   - All signals idle \n
 *   - All voltages off.
 */
void suspend(void){
#if (HW_CAPABILITY&CAP_FLASH)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif
#if (HW_CAPABILITY&CAP_CFVx_HW)
   bdmCF_suspend();
#endif
#if (HW_CAPABILITY&CAP_BDM)
   Bdm::disableInterface();
#endif
#if (HW_CAPABILITY&CAP_SWD_HW)
   Swd::disableInterface();
#endif
   cycleTargetVddOff();
}

/**
 *
 *  Turns off the BDM interface
 *
 *   Depending upon settings, may leave target power on.
 */
void interfaceOff( void ) {
#if (HW_CAPABILITY&CAP_SWD_HW)
   Swd::disableInterface();
#endif
#if (HW_CAPABILITY&CAP_BDM)
   Bdm::disableInterface();
#endif
   if (!bdm_option.leaveTargetPowered) {
      cycleTargetVddOff();
   }
}

/**
 * Clear Cable status
 */
USBDM_ErrorCode clearStatus(void) {
   memset(&cable_status, 0, sizeof(cable_status));
   cable_status.target_type = T_OFF;
   return BDM_RC_OK;
}

/**
 * Initialises BDM module for the given target type
 *
 * @param target = Target processor
 *
 * @return E_NO_ERROR on success
 */
USBDM_ErrorCode setTarget(TargetType_t target) {
   USBDM_ErrorCode rc = BDM_RC_OK;

   if (target == T_OFF) {
      interfaceOff();
   }
   clearStatus();

   // Initially assume mode is valid
   cable_status.target_type = target;

   switch (target) {
#if TARGET_CAPABILITY & CAP_S12Z
      case T_S12Z  :
#endif
#if (TARGET_CAPABILITY & (CAP_HCS12|CAP_S12Z))
      case T_HC12:
         bdm_option.useResetSignal = 1; // Must use RESET signal on HC12
         Bdm::initialise();
         break;
#endif
#if (TARGET_CAPABILITY & CAP_RS08)
      case T_RS08:
#endif
#if (TARGET_CAPABILITY & CAP_HCS08)
      case T_HCS08:
#endif
#if (TARGET_CAPABILITY & CAP_CFV1)
      case T_CFV1:
#endif
#if (TARGET_CAPABILITY & (CAP_RS08|CAP_HCS08|CAP_CFV1))
         Bdm::initialise();
         break;
#endif
#if (TARGET_CAPABILITY&CAP_CFVx)
      case T_CFVx:
         bdm_option.useResetSignal = 1; // Must use RESET signal on CFVx
         bdmcf_init();                  // Initialise the BDM interface
         bdmcf_resync();                // Synchronise with the target (ignore error?)
         break;
#endif
#if (TARGET_CAPABILITY&CAP_JTAG)
      case T_JTAG:
#endif
#if (TARGET_CAPABILITY&CAP_DSC)
      case T_MC56F80xx:
#endif
#if (TARGET_CAPABILITY&(CAP_JTAG|CAP_DSC))
         bdm_option.useResetSignal = 1; // Must use RESET signal on JTAG etc
         jtag_init();                   // Initialise JTAG
         break;
#endif
#if (TARGET_CAPABILITY&CAP_ARM_JTAG)
      case T_ARM_JTAG:
         jtag_init();      // Initialise JTAG
         break;
#endif
#if (TARGET_CAPABILITY&CAP_ARM_SWD)
      case T_ARM_SWD:
         Swd::initialiseInterface(); // Initialise JTAG
         break;
#endif
      case T_OFF:
         break;

      default:
         // Turn off the interface
         interfaceOff();
         return BDM_RC_UNKNOWN_TARGET;
   }
   return rc;
}
