/*! \file
    \brief USBDM - RS08 low level BDM communication.

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
   +===================================================================================================
   |    Oct 2011 | Modified VPP_EN Control to allow use of timer (for TOWER boards)         V3.8  - pgo
   |    Apr 2010 | All significant RS08 code is now in USBDM.dll (only Vpp control remains) V3.5  - pgo
   |    Feb 2010 | Greatly simplified for Version 3 USBDM - most code now in USBDM.dll            - pgo
   +===================================================================================================
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

#if ((HW_CAPABILITY & CAP_FLASH) != 0)

void vppOff(void) {
#ifdef VPP_USES_TIMER
   // Set up timer channel to (immediately) turn off VPP_EN
   VPP_TPMxCnVALUE = TPMCNT+10;
   VPP_TPMxCnSC    = VPP_TPMxCnSC_OC_CLR_MASK;
#else
   VPP_OFF();
#endif
}

void vppOn(void) {
#ifdef VPP_USES_TIMER
   // Set up timer channel to (immediately) turn on VPP_EN
   VPP_TPMxCnVALUE = TPMCNT+10;
   VPP_TPMxCnSC    = VPP_TPMxCnSC_OC_SET_MASK;
#else
   VPP_ON();
#endif
}

//========================================================
//! Control target VPP level
//!
//! @param level (FlashState_t) control value for VPP \n
//!
//! @return
//!     BDM_RC_OK   => Success \n
//!     else        => Error
//!
U8 bdmSetVpp(U8 level ) {

   U8 rc = BDM_RC_OK;
   
   // Safety check - don't do anything other than turn off Vpp for wrong target
   if ((cable_status.target_type != T_RS08) && (commandBuffer[2] != BDM_TARGET_VPP_OFF)) {
      rc    = BDM_RC_ILLEGAL_COMMAND;
      level = BDM_TARGET_VPP_OFF;
   }
   switch (level) {
      default :
		  rc = BDM_RC_ILLEGAL_PARAMS;
		  // Fall through
      case BDM_TARGET_VPP_OFF      :  // Flash programming voltage off
         vppOff();
         FLASH12V_OFF();
         cable_status.flashState = BDM_TARGET_VPP_OFF;
         break;

      case BDM_TARGET_VPP_STANDBY  :  // Flash programming voltage off (inverter on)
         vppOff();
         FLASH12V_ON();
         cable_status.flashState = BDM_TARGET_VPP_STANDBY;
         break;

      case BDM_TARGET_VPP_ON       :  // Flash programming voltage on
         // Must already be in standby
         if (cable_status.flashState != BDM_TARGET_VPP_STANDBY)
            return BDM_RC_ILLEGAL_PARAMS;
         
         // Must have Vdd BEFORE Vpp
         if ((cable_status.power != BDM_TARGET_VDD_EXT) &&
             (cable_status.power != BDM_TARGET_VDD_INT))
            return BDM_RC_VDD_NOT_PRESENT;
         vppOn();
         cable_status.flashState = BDM_TARGET_VPP_ON;
         break;
         
   }
   return rc;
}

#endif
