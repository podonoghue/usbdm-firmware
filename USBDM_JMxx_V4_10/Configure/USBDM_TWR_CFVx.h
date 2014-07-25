/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_TWR_KINETIS - USDBM for Coldfire boards using OSBDM e.g. M52259Demo
            
    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.

   \verbatim
   Change History
   +================================================================================================
   | 18 Jul 2014 | Removed   CAP_VDDSENSE                                      - pgo, ver 4.10.6.170
   +================================================================================================
   \endverbatim
*/

#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//==========================================================================================
// USB Serial Number
#define SERIAL_NO "USBDM-TWR-CFVx-0001"
#define ProductDescription "USBDM CFVx BDM for Tower"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_CDC|CAP_RST_IO|CAP_JTAG_HW|CAP_CFVx_HW|CAP_PST|CAP_CORE_REGS)
#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_CDC|CAP_RST|CAP_CFVx|CAP_JTAG|CAP_PST)

// Include common pin assignments
#include "USBDM_TWR.h"

#define VDD_HAS_DIVIDER     2 // Board has VDD input 2:1 voltage divider

#endif // _CONFIGURE_H_
