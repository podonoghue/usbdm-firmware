/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_TWR_HCS12 - USDBM for HCS12 TWR boards
            
    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//==========================================================================================
// USB Serial Number
#define SERIAL_NO          "USBDM-TWR-HCS12-0001"
#define ProductDescription "USBDM HCS12 BDM for Tower"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_CDC|CAP_RST_IO|CAP_BDM|CAP_CORE_REGS)
#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_CDC|CAP_HCS12)

// Include common pin assignments
#include "USBDM_TWR.h"

#define  VDD_HAS_DIVIDER     2 // VDD input has 2:1 voltage divider

#endif // _CONFIGURE_H_
