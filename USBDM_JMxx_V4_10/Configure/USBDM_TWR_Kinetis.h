/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_TWR_KINETIS - USDBM for Kinetis TWR boards
            
    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//==========================================================================================
// USB Serial Number
#define SERIAL_NO "USBDM-TWR-KINETIS-0001"
#define ProductDescription "USBDM Kinetis BDM for Tower"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_CDC|CAP_RST_IO|CAP_JTAG_HW)
#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_CDC|CAP_RST|CAP_ARM_JTAG)

// Include common pin assignments
#include "USBDM_TWR.h"

#undef VDD_HAS_DIVIDER // Boards has no VDD input voltage divider

#endif // _CONFIGURE_H_
