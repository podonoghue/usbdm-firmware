/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_CF - MC56F8006 Demoboard version
            
    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//==========================================================================================
// USB Serial Number
#define SERIAL_NO          "USBDM-JMxx-MC56F006Demo-0001"      //! Default USB serial number 
#define ProductDescription "USBDM DSC on MC56F8006 Demo board" //! USB description

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY     (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_CDC|CAP_RST_IO|CAP_JTAG_HW)
#define TARGET_CAPABILITY (CAP_VDDCONTROL|CAP_VDDSENSE|CAP_CDC|CAP_RST|CAP_DSC    )     //!

// Include common pin assignments
#include "USBDM_TWR.h"

#endif // _CONFIGURE_H_
