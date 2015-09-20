/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_TWR_HCS08 - USDBM for RS08/HCS08 TWR boards
            
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
#define SERIAL_NO          "USBDM-TWR-HCS08-0001"
#define ProductDescription "USBDM RS08,HCS08 BDM for Tower"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_CDC|CAP_BDM|CAP_FLASH|CAP_CORE_REGS)
#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_CDC|CAP_RS08|CAP_HCS08)

// Include common pin assignments
#include "USBDM_TWR.h"

#define  VDD_HAS_DIVIDER     2 // VDD input has 2:1 voltage divider

#endif // _CONFIGURE_H_
