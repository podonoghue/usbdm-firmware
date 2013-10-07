/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_TWR_KINETIS - USDBM for Coldfire V1 TWR boards
            
    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//==========================================================================================
// USB Serial Number
#define SERIAL_NO          "USBDM-TWR-CFV1-0001"               //! Default USB serial number 
#define ProductDescription "USBDM Coldfire-V1 BDM for Tower"   //! USB description

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_CDC|CAP_RST_IO|CAP_BDM|CAP_CORE_REGS)
#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_CDC|CAP_RST|CAP_CFV1)

// Include common pin assignments
#include "USBDM_TWR.h"

#undef VDD_HAS_DIVIDER // Boards has no VDD input voltage divider

#endif // _CONFIGURE_H_
