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
//======================================================================================
//
#define VISTA (1) // Define to remove bulk endpoints which are not supported under Vista
                  // Note: Removing this definition breaks the BDM under Vista even
                  //       if bulk endpoints are not actually used by the driver!

//==========================================================================================
// Define the following to enable use of USBDM with MC51AC256 Colfire CPU
// Not extensively tested - may affect other coldfire chips adversely
// NOTE: This has been moved to Codewarrior Legacy DLLs
//#define MC51AC256_HACK (1)

//=================================================================================
// Debugging options
//
#define DEBUG_COMMANDS (1<<0)                   //!< Implement debugging command interface (see \ref CMD_USBDM_DEBUG)
#define STACK_DEBUG    (DEBUG_COMMANDS|(1<<1))  //!< Implement measurement of stack size code (see \ref BDM_DBG_STACKSIZE)
#define ACK_DEBUG      (1<<2)                   //!< Debug pin toggles during ACK code
#define SYNC_DEBUG     (1<<3)                   //!< Debug pin toggles during SYNC code
#define RESET_DEBUG    (1<<4)                   //!< Debug pin toggles during Reset sequence
#define CYCLE_DEBUG    (1<<5)                   //!< Debug pin toggles during Vdd cycling
#define COMMAND_BUSY   (1<<6)                   //!< Debug pin high while command being executed
//! Use debug version of commandExec - appends an extra byte with connection status to each command response
//! Requires modified DLLs (for coldfire)
#define COMMAND_DEBUG  (1<<7)
#define DEBUG_MESSAGES (1<<8)                   //!< Serial port/memory debug messages

/*! \brief Enables various debugging code options.

    This is a bit mask made up of all the debugging options that are to be implemented in the code.
 */
#define DEBUG (DEBUG_COMMANDS|STACK_DEBUG|CYCLE_DEBUG)//|ACK_DEBUG|SYNC_DEBUG|RESET_DEBUG)

#if DEBUG&DEBUG_MESSAGES
extern void dputs(char *msg);
#define dprint(x) dputs(x)
//#define dprint(x) (void)puts(x)
//#define dprint(x) ;
#endif // DEBUG&DEBUG_MESSAGES

//==========================================================================================
//Control of  USB pup on D-
#define PUP_AUTO (0)  // Auto detect if external PUP is present 
#define PUP_ON   (1)  // Turn on internal PUP
#define PUP_OFF  (2)  // Turn off internal PUP (external PUP required)

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
// HW_CAPABILITY
//
#define CAP_RST_IO      (1<<0)   // RESET can be driven/sensed (req. for HC12)
#define CAP_FLASH       (1<<1)   // 12 V Flash programming supply available (req. RS08)
#define CAP_VDDCONTROL  (1<<2)   // Control over target Vdd
#define CAP_VDDSENSE    (1<<3)   // Sensing of target Vdd
#define CAP_CFVx_HW     (1<<4)   // Supports CFVx extensions beyond basic JTAG (TA etc)
#define CAP_BDM         (1<<5)   // Supports 1-wire BDM interface (BKGD I/O)
#define CAP_JTAG_HW     (1<<7)   // Supports JTAG interface (TCK/TDI/TDO/TMS/TRST?)
#define CAP_CDC         (1<<12)  // Supports CDC USB interface

//==========================================================================================
// Targets and visible capabilities supported - related to above but not exactly!
// e.g. CAP_HCS12 => CAP_BDM+CAP_RST_IO
//      CAP_RS08  => CAP_BDM+CAP_FLASH(+CAP_RST_IO)
//      CAP_HCS08 => CAP_BDM(+CAP_RST_IO)
//      CAP_CFVx  => CAP_JTAG_HW+CAP_CFVx_HW+CAP_RST_IO
//      CAP_DSC   => CAP_JTAG_HW+CAP_RST_IO + s/w routines
//      CAP_JTAG  => CAP_JTAG_HW+CAP_RST_IO
//      CAP_RST   => CAP_RST_IO
// TARGET_CAPABILITY
//
#define CAP_HCS12       (1<<0)      // Supports HCS12 targets
#define CAP_RS08        (1<<1)      // Supports RS08 targets
#define CAP_VDDCONTROL  (1<<2)      // Control over target Vdd
#define CAP_VDDSENSE    (1<<3)      // Sensing of target Vdd
#define CAP_CFVx        (1<<4)		// Supports CFVx
#define CAP_HCS08       (1<<5)		// Supports HCS08 targets - inverted when queried
#define CAP_CFV1        (1<<6)		// Supports CFV1 targets  - inverted when queried
#define CAP_JTAG        (1<<7)		// Supports JTAG targets
#define CAP_DSC         (1<<8)		// Supports DSC targets
#define CAP_ARM_JTAG    (1<<9)      // Supports ARM targets via JTAG
#define CAP_RST         (1<<10)     // Control & sensing of RESET
#define CAP_PST         (1<<11)     // Supports PST signal sensing
#define CAP_CDC         (1<<12)     // Supports CDC Serial over USB interface
#define CAP_ARM_SWD     (1<<13)     // Supports ARM targets via SWD

//===========================================================================================
// Three types of BDM interface chips are supported
// I believe use of the transceiver has a hardware bug unless resistors are used!
//
#define LVC125 (1)  // Hex buffer (LVC125, LVC2G125 etc) as used in original TBDML
#define LVC45  (2)  // Transceiver (LVC1T45, LVC2T45 etc) as used in OSBDM08
#define FETDRV (3)  // Single MOSFET used as bidirectional driver/level converter

//=====================================================================================
// The following lines choose a Hardware configuration
//=====================================================================================
#define H_USBDM                 1  //!< USBDM    - Universal TBDML/OSBDM JB16
#define H_TBDML                 2  //!< TBDML    - Minimal JB16 version (JB16DWE,JB16JDWE)
#define H_TBDMLSwin             3  //!< No longer used
#define H_OSBDM                 4  //!< OSBDM    - Basic OSBDM hardware
#define H_WTBDM                 5  //!< WTBDM08  - Wiztronics BDMS08/12
#define H_OSBDME                6  //!< OSBDM+E  - OSBDM+Flash supply
#define H_USBDM_JMxxCLD         7  //!< USBDM hardware using 9S08JM16/32/60CLD (44p package)
#define H_USBDM_JMxxCLC         8  //!< USBDM hardware using 9S08JM16CLC (32p package)
#define H_USBSPYDER             9  //!< USBSPYDER - SofTec USBSPYDER08 - not functional
#define H_USBDM_UF32PBE        10  //!< USBDM hardware using MC9S12UF32PBE (64p package)
#define H_USBDM_CF_JS16CWJ     11  //!< USBDM hardware CF/DSC only using MC9S08JS16CWJ (20p SOIC package)
#define H_USBDM_CF_JMxxCLD     12  //!< Combined USBDM/TBLCF using 9S08JM16/32/60CLD (44p package)
#define H_USBDM_JS16CWJ        13  //!< USBDM hardware using MC9S08JS16CWJ (20p SOIC package)
#define H_USBDM_MC56F8006DEMO  14  //!< MC56F8006DEMO Board (Axiom)
#define H_CUSTOM               15  //!< Reserved for USER created custom hardware
#define H_USBDM_CF_SER_JS16CWJ 16  //!< USBDM hardware CF/DSC only using MC9S08JS16CWJ (20p SOIC package) with serial interface
#define H_USBDM_SER_JS16CWJ    17  //!< USBDM hardware using MC9S08JS16CWJ (20p SOIC package) with Serial interface

#include "derivative.h"

//==========================================================================================
//! Software Version Information
//
#define VERSION_MAJOR 4     // 4.10.0 - Last published -- 4.9
#define VERSION_MINOR 10
#define VERSION_MICRO 2
#define VERSION_STR "4.10.0"
#define VERSION_SW  ((VERSION_MAJOR<<4)+VERSION_MINOR)
//! Selected hardware platform
#if TARGET_HARDWARE==H_USBDM
#include "USBDM_JB16.h"          // USBDM    - Universal TBDML/OSBDM  - see USBDM_JB16_SOIC.pdf
#elif TARGET_HARDWARE==H_TBDML
#include "TBDML_JB16.h"          // TBDML    - Minimal JB16 version   - see TBDML_JB16_SOIC28.pdf
#elif TARGET_HARDWARE==H_OSBDM
#include "OSBDM_JB16.h"          // OSBDM    - Basic OSBDM hardware   - see OSBDM.pdf schematic, 21/12/2005
#elif TARGET_HARDWARE==H_WTBDM
#include "WTBDM_JB16.h"          // WTBDM08  - Wiztronics BDMS08      - see WTBDMS08.pdf schematic 27/6/2007 
#elif TARGET_HARDWARE==H_OSBDME
#include "OSBDME_JB16.h"         // OSBDM+E  - OSBDM+Flash supply     - see OSBDM08+e.pdf schematic, 30/12/2006
#elif TARGET_HARDWARE==H_USBSPYDER
#include "USBSPYDER_JB16.h"      // USBSPYDER
#else
#error "Target Hardware not specified (see TARGET_HARDWARE)"
// To stop lots of further errors!
#define TARGET_HARDWARE H_USBDM
#include "USBDM_JB16.h"          // USBDM    - Universal TBDML/OSBDM  - see USBDM_JB16_SOIC.pdf
#endif

#ifndef SERIAL_NO
// USB Serial number as UTF-16-LE
#define SERIAL_NO "USBDM-0000"
//#define SERIAL_NO "Leovenaðes"
//#define SERIAL_NO "¥£€$¢¢??£?"
//#define SERIAL_NO "\xC2\xA9\xE2\x89\xA0" 
#endif

#ifndef ProductDescription
#define ProductDescription "USBDM BDM Interface"
#endif

//==========================================================================================
//! Hardware Version Information
//! An unique number is used for each hardware combination.
//! JMxx/JS16 	version have +0x80
//! UF32 		version has  +0xC0
//!
#define HW_JB        0x00
#define HW_JM        0x80
#define HW_UF        0xC0

#define VERSION_HW  (HW_JB+TARGET_HARDWARE)

//===========================================================================================
// Platforms
// -  It controls how the hardware identifies itself to the host - either as TBDML or OSBDM
//
#define DEBUG_USBDM (-1) // For debug as different USB Device
#define TBDML (1)        // Enumerate as TBDML
#define OSBDM (2)        // Enumerate as OSBDM
#define USBDM (3)        // Enumerate as OSBDM
#define TBLCF (4)        // Enumerate as TBLCF

#ifndef PLATFORM
#define PLATFORM USBDM   // Choose BDM emulation
#endif

#if (PLATFORM==TBDML)
#error "TBDML emulation is no longer supported"
#elif (PLATFORM==OSBDM)
#error "OSBDM emulation is no longer supported"
#elif (PLATFORM==USBDM)
//#define VendorID  (0x15A2)
//#define ProductID (0x0021)
#define VendorID  (0x16D0)
#define ProductID (0x0567)
#elif (PLATFORM==DEBUG_USBDM)
#define VendorID  (0x1111)
#define ProductID (0x2222)
#undef ProductDescription
#define ProductDescription "USBDM(JB16) Version " VERSION_STR " (Debug)      "
#elif (PLATFORM==TBLCF)
#error "TBLCF emulation is not supported"
#else
#error "Please correctly define PLATFORM in Configure.h"
#define PLATFORM USBDM // Just to prevent further error messages.
#endif

//==========================================================================================
// CPUs supported (just clock frequency changes)
//
#define JB8  (1)      // Not supported
#define JB16 (2)
#define JM32 (3)
#define JM60 (4)
#define UF32 (5)
#define JS16 (6)

#define CPU JB16

#ifndef CPU
#error "Please define CPU in Configure.h"
#define CPU JB16
#endif

#if (CPU == JB16)
#define OSC_FREQ     (12000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/2)               // Bus freq. derived from oscillator
//#error "JB16 is not supported by this version of the software"
#elif (CPU == JB8)
#define OSC_FREQ     (12000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/4)               // Bus freq. derived from oscillator
#error "JB8 is not supported by this version of the software"
#elif (CPU == JM32) || (CPU == JM60) || (CPU == JS16)
#define OSC_FREQ     (48000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/2)               // Bus freq. derived from oscillator
#error "JM60 is not supported by this version of the software"
#elif (CPU == UF32)
#define OSC_FREQ     (60000000UL)               // Oscillator frequency
#define BUS_FREQ     (OSC_FREQ/2)               // Bus freq. derived from oscillator
#error "UF32 is not supported by this version of the software"
#else
#error "Please correctly define CPU in Configure.h"
#define CPU JB16
#endif

#if !defined USBPUP     // Not told if external USB resistor is present
#define USBPUP PUP_AUTO // Try to guess!
#endif

