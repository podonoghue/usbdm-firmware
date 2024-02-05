/*
 *  @file system-gcc.cpp (180.ARM_Peripherals/Startup_Code/system-gcc.cpp)
 *
 *  GCC specific code
 *
 *  Created on: 25/5/2017
 */
#include <stdlib.h>
#include "pin_mapping.h"

/* Prevents the exception handling name demangling code getting pulled in */
namespace __gnu_cxx {
    void __verbose_terminate_handler() {
        USBDM::setAndCheckErrorCode(USBDM::E_TERMINATED);
    }
}
extern "C" __attribute__((__weak__)) void __cxa_pure_virtual(void);
extern "C" __attribute__((__weak__)) void __cxa_pure_virtual(void) {
    exit(1);
}
