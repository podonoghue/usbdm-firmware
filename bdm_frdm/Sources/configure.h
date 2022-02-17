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

#ifndef SOURCES_CONFIGURE_H_
#define SOURCES_CONFIGURE_H_

//=================================================================================
// Debugging options
//
#define DEBUG_COMMANDS (1<<0)                   //!< Implement debugging command interface (see \ref CMD_USBDM_DEBUG)
#define STACK_DEBUG    (1<<1)                   //!< Implement measurement of stack size code (see \ref BDM_DBG_STACKSIZE)
#define ACK_DEBUG      (1<<2)                   //!< Debug pin toggles during ACK code
#define SYNC_DEBUG     (1<<3)                   //!< Debug pin toggles during SYNC code
#define RESET_DEBUG    (1<<4)                   //!< Debug pin toggles during Reset sequence
#define CYCLE_DEBUG    (1<<5)                   //!< Debug pin toggles during Vdd cycling
#define COMMAND_BUSY   (1<<6)                   //!< Debug pin high while command being executed
#define USB_PING_DEBUG (1<<7)                   //!< Debug pin toggles on USB ...
#define DEBUG_MESSAGES (1<<8)                   //!< Serial port/memory debug messages
#define SCI_DEBUG      (1<<9)                   //!< SCI Tx & Rx routines

/*! \brief Enables various debugging code options.
 *
 *  This is a bit mask made up of all the debugging options that are to be enabled in the code.
 */
#define DEBUG 0 //USB_PUTS_DEBUG // DEBUG_COMMANDS // (STACK_DEBUG)

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code in build
// HW_CAPABILITY
//
#define CAP_RST_OUT     (1<<0)   // RESET can be driven/sensed (required for HC12)
#define CAP_FLASH       (1<<1)   // 12 V Flash programming supply available (required RS08)
#define CAP_VDDCONTROL  (1<<2)   // Control over target Vdd
#define CAP_VDDSENSE    (1<<3)   // Sensing of target Vdd
#define CAP_CFVx_HW     (1<<4)   // Supports CFVx extensions beyond basic JTAG (TA etc)
#define CAP_BDM         (1<<5)   // Supports 1-wire BDM interface (BKGD I/O)
#define CAP_JTAG_HW     (1<<7)   // Supports JTAG interface (TCK/TDI/TDO/TMS/TRST?)
#define CAP_SWD_HW      (1<<8)   // Supports SWD interface (SWD/SWCLK)
#define CAP_RST_IN      (1<<9)   // RESET can be sensed
#define CAP_CDC         (1<<12)  // Supports CDC USB interface
#define CAP_CORE_REGS   (1<<31)  // Supports reading core regs

//==========================================================================================
// Targets and visible capabilities supported - related to above but not exactly!
// e.g. CAP_HCS12 => CAP_BDM+CAP_RST_OUT
//      CAP_RS08  => CAP_BDM+CAP_FLASH(+CAP_RST_OUT)
//      CAP_HCS08 => CAP_BDM(+CAP_RST_OUT)
//      CAP_CFVx  => CAP_JTAG_HW+CAP_CFVx_HW+CAP_RST_OUT
//      CAP_DSC   => CAP_JTAG_HW+CAP_RST_OUT + s/w routines
//      CAP_JTAG  => CAP_JTAG_HW+CAP_RST_OUT
//      CAP_RST   => CAP_RST_OUT
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
#define CAP_RST         (1<<10)     // Control of RESET
#define CAP_PST         (1<<11)     // Supports PST signal sensing
#define CAP_CDC         (1<<12)     // Supports CDC Serial over USB interface
#define CAP_ARM_SWD     (1<<13)     // Supports ARM targets via SWD
#define CAP_S12Z        (1<<14)     // Supports HCS12ZVM

//=====================================================================================
// The following lines choose a Hardware configuration
//=====================================================================================
#define H_USBDM_OPENSDA         25  //!< Freescale FRDM-xxxx board (MK20 chip)
#define H_USBDM_MKL25Z          26  //!< Experimental MKL25Z
#define H_USBDM_MK20D5          27  //!< Experimental MK20D5 or MK22D5
//Reserved                      28  //!< TWR HCS12 boards
#define H_USBDM_MK22F12         29  //!< Experimental MK22F512M12

#include "derivative.h"

#if defined MCU_MK22F51212
#define TARGET_HARDWARE H_USBDM_MK22F12
#elif defined  MCU_MK22D5
#define TARGET_HARDWARE H_USBDM_MK20D5
#else
#define TARGET_HARDWARE H_USBDM_OPENSDA
#endif

//==========================================================================================
//! Hardware Version Information
//! An unique number is used for each hardware combination.
//! JMxx/JS16  version have +0x80
//! UF32       version has  +0xC0
//! ARM        version has  +0x40
//!
#define HW_JB        0x00
#define HW_JM        0x80
#define HW_UF        0xC0
#define HW_ARM       0x40

//==========================================================================================
// USB Serial Number
#ifdef UNIQUE_ID
#define SERIAL_NO           "USBDM-OPENSDA-"
#else
#define SERIAL_NO           "USBDM-OPENSDA-0001"
#endif

#if defined(OPEN_SDA_V2_1)
#define PRODUCT_DESCRIPTION "USBDM ARM-SWD for OpenSDAv2.1"
#elif defined(OPEN_SDA_V2_0)
#define PRODUCT_DESCRIPTION "USBDM ARM-SWD for OpenSDAv2.0"
#elif defined(OPEN_SDA_V1)
#define PRODUCT_DESCRIPTION "USBDM ARM-SWD for OpenSDAv1"
#else
#define PRODUCT_DESCRIPTION "USBDM ARM-SWD for FRDM"
#endif

#define MANUFACTURER        "pgo"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#if defined SDA_POWER
#define HW_CAPABILITY       (CAP_RST_OUT|CAP_CDC|CAP_SWD_HW|CAP_CORE_REGS|CAP_VDDCONTROL)
#define TARGET_CAPABILITY   (CAP_RST    |CAP_CDC|CAP_ARM_SWD|CAP_VDDCONTROL)
#else
#define HW_CAPABILITY       (CAP_RST_OUT|CAP_CDC|CAP_SWD_HW|CAP_CORE_REGS)
#define TARGET_CAPABILITY   (CAP_RST    |CAP_CDC|CAP_ARM_SWD)
#endif

#define CPU  MK20D5

#define VERSION_HW  (HW_ARM+TARGET_HARDWARE)

//==========================================================================================
//! Software Version Information
//
#define VERSION_MAJOR 5
#define VERSION_MINOR 4
#define VERSION_MICRO 0
#define VERSION_SW  ((VERSION_MAJOR<<4)+VERSION_MINOR)

#define VENDOR_ID        (0x16D0)       // Vendor (actually MCS)
#if (HW_CAPABILITY&CAP_CDC)
#define PRODUCT_ID       (0x06A5)       // CDC versions
#else
#define PRODUCT_ID       (0x0567)       // Non-CDC versions
#endif

#define VERSION_ID       (VERSION_SW)   // Reported version (via USB)

#endif /* SOURCES_CONFIGURE_H_ */
