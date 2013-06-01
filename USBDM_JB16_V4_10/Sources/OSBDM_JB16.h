/*! @file
    @brief This file contains hardware specific information and configuration for the "Freescale OSBDM hardware".
    
    USBDM - Universal BDM, JB16 Version \n
    Configuration for Original Freescale OSBDM hardware, see 
    @htmlonly <a href="OSBDM.pdf">Schematic</a> @endhtmlonly \n
    Supports HC12, HCS08 and Coldfire V1 targets \n

    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file 
    under a new name and change Configure.h appropropriately.
*/
// OSBDM & WTBDM hardware are the same
#include "WTBDM_JB16.h"

#undef ICP_PIN
#undef ENABLE_ICP_PIN
#undef DISABLE_ICP_PIN
#undef TEST_ICP_PIN

#define ICP_PIN            (3<<0)  // ICP Test input D.3
#define ENABLE_ICP_PIN()   (DDRD &= ~ICP_PIN)
#define DISABLE_ICP_PIN()  
#define TEST_ICP_PIN()     (~PTD & ICP_PIN)
