/*! \file
    \brief Configuration for debug options and main hardware selection

    @note This file \b \#includes a detailed configuration file for each hardware platform supported.
         If creating a configuration for new hardware do the following: \n\n
         -  Copy one of the existing configurations (e.g. USBDM_JM60.h) to a new file with a sensible name and edit that.
         -  Add the appropriate conditional lines to Configure.h (this file).   This should \e \#include your file when TARGET_HARDWARE has a unique value
         -  Add another target to the Codewarrior project (Project->create Target...)
         -  Choose the "Clone existing target" option and choose \e USBDM
         -  Modify the \b Compiler options for this target to define the symbol used above (change  \b -DTARGET_HARDWARE=H_USBDM appropriately)
         -  Modify the \b Linker options for this target so that the Application Filename is unique (change  \b USBDM_JB16 appropriately)
 */

//==========================================================================================
// Define the following to enable use of USBDM with MC51AC256 Colfire CPU
// Not extensively tested - may affect other coldfire chips adversely
// NOTE: This has been moved to Codewarrior Legacy DLLs
//#define MC51AC256_HACK (1)


#include "derivative.h"

//==========================================================================================
// CPUs supported (just clock frequency changes)
//
#define JB8      (1)      // Not supported
#define JB16     (2)
#define JMxx     (3)
#define UF32     (5)
#define JS16     (6)
#define MK20D5   (7)
#define MKL25Z4  (8)

#include "USBDM_MK20D5.h"

#ifndef CPU
#error "Please define CPU in Configure.h"
#define CPU  JM60
#endif

#if (CPU == JB16)
#define OSC_FREQ     (12000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/2)               // Bus freq. derived from oscillator
#error "JB16 is not supported by this version of the software"
#elif (CPU == JB8)
#define OSC_FREQ     (12000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/4)               // Bus freq. derived from oscillator
#error "JB8 is not supported by this version of the software"
#elif (CPU == JMxx) || (CPU == JS16)
#define OSC_FREQ     (48000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/2)               // Bus freq. derived from oscillator
//#error "JMxx is not supported by this version of the software"
#elif (CPU == UF32)
#define OSC_FREQ     (60000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/2)               // Bus freq. derived from oscillator
#error "UF32 is not supported by this version of the software"
#elif (CPU == MK20D5)
#define OSC_FREQ     (96000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/4)               // Bus freq. derived from oscillator
#elif (CPU == MKL25Z4)
#define OSC_FREQ     (96000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/4)               // Bus freq. derived from oscillator
#else
#error "Please correctly define CPU in Configure.h"
#define CPU JM60
#endif
