/*! \file
    \brief Command and Error codes for BDM communication over USB

\verbatim

   USBDM
   Copyright (C) 20010  Peter O'Donoghue

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    Change History
+===================================================================================================
| 18 Jul 2014 | Added HCS12ZVM support                                             - pgo V4.10.6.170
+===================================================================================================
    \endverbatim
*/
#ifndef _COMMANDS_H_
#define _COMMANDS_H_

static constexpr int  MAX_COMMAND_SIZE = 254;

//! BDM command values
//!
//! The following values are the 1st byte in each command.  \n
//! Other parameters are as shown below. \n
//! Each command returns a status value (see  USBDM_ErrorCode) as the first byte
//! followed by any results as indicated below.
//!
enum BDMCommands {
   // Common to all targets
   CMD_USBDM_GET_COMMAND_RESPONSE        = 0,   //!< Status of last/current command
   CMD_USBDM_SET_TARGET                  = 1,   //!< Set target,  @param [2] 8-bit target value @ref TargetType_t
   CMD_USBDM_SET_VDD                     = 2,   //!< Set target Vdd (immediate effect)
   CMD_USBDM_DEBUG                       = 3,   //!< Debugging commands (parameter determines actual command) @param [2]  Debug command see DebugSubCommands
   CMD_USBDM_GET_BDM_STATUS              = 4,   //!< Get BDM status\n @return [1] 16-bit status value reflecting BDM status
   CMD_USBDM_GET_CAPABILITIES            = 5,   //!< Get capabilities of BDM, see HardwareCapabilities_t
   CMD_USBDM_SET_OPTIONS                 = 6,   //!< Set BDM options, see BDM_Options_t
//   CMD_USBDM_GET_SETTINGS              = 7,   //!< Get BDM setting
   CMD_USBDM_CONTROL_PINS                = 8,   //!< Directly control BDM interface levels
   // Reserved 7..11
   CMD_USBDM_GET_VER                     = 12,  //!< Sent to ep0 \n Get firmware version in BCD \n
                                                //!< @return [1] 8-bit HW (major+minor) revision \n [2] 8-bit SW (major+minor) version number
   CMD_GET_VER                           = 12,  //!< Deprecated name - Previous version
   // Reserved 13
   CMD_USBDM_ICP_BOOT                    = 14,  //!< Sent to ep0 \n
                                                //!< Requests reboot to ICP mode. @param [2..5] must be "BOOT"
   CMD_SET_BOOT                          = 14,  //!< Deprecated - Previous version

   // Target specific versions
   CMD_USBDM_CONNECT                     = 15,  //!< Try to connect to the target
   CMD_USBDM_SET_SPEED                   = 16,  //!< Sets-up the BDM interface for a new bit rate & tries
                                                //!    to enable ackn feature, @param [2..3] 16-bit tick count
   CMD_USBDM_GET_SPEED                   = 17,  //!< Read speed of the target: @return [1..2] 16-bit tick coun

   CMD_CUSTOM_COMMAND                    = 18,  //!< Directly control BDM interface levels
   // Reserved 19

   CMD_USBDM_READ_STATUS_REG             = 20,  //!< Get BDM status
                                                //! @return [1] 8-bit status byte made up as follows: \n
                                                //!    - (HC08/12/RS08/CFV1) bit0   - ACKN, \n
                                                //!    - (All)               bit1   - target was reset (this bit is cleared after reading),  \n
                                                //!    - (CFVx only)         bit2   - current RSTO value \n
                                                //!    - (HC08/12/RS08/CFV1) bit4-3 - comm status: 00=NOT CONNECTED, 01=SYNC, 10=GUESS,  11=USER SUPPLIED \n
                                                //!    - (All)               bit7   - target has power

   CMD_USBDM_WRITE_CONTROL_REG           = 21,  //!< Write to target Control register

   CMD_USBDM_TARGET_RESET                = 22,  //!< Reset target @param [2] TargetMode_t
   CMD_USBDM_TARGET_STEP                 = 23,  //!< Perform single step
   CMD_USBDM_TARGET_GO                   = 24,  //!< Start code execution
   CMD_USBDM_TARGET_HALT                 = 25,  //!< Stop the CPU and bring it into background mode

   CMD_USBDM_WRITE_REG                   = 26,  //!< Write to target register
   CMD_USBDM_READ_REG                    = 27,  //!< Read target register

   CMD_USBDM_WRITE_CREG                  = 28,  //!< Write target Core register
   CMD_USBDM_READ_CREG                   = 29,  //!< Read from target Core register

   CMD_USBDM_WRITE_DREG                  = 30,  //!< Write target Debufg register
   CMD_USBDM_READ_DREG                   = 31,  //!< Read from target Debug register

   CMD_USBDM_WRITE_MEM                   = 32,  //!< Write to target memory
   CMD_USBDM_READ_MEM                    = 33,  //!< Read from target memory
   CMD_USBDM_READ_ALL_REGS               = 34,  //!< Read all target core registers

//   CMD_USBDM_TRIM_CLOCK                  = 34,  //!< Trim target clock - deleted in V3.2
//   CMD_USBDM_RS08_FLASH_ENABLE           = 35,  //!< Enable target flash programming (Vpp on)
//   CMD_USBDM_RS08_FLASH_STATUS           = 36,  //!< Status of target flash programming
//   CMD_USBDM_RS08_FLASH_DISABLE          = 37,  //!< Stop target flash programming (Vpp off)

   CMD_USBDM_JTAG_GOTORESET              = 38,  //!< Reset JTAG Tap controller
   CMD_USBDM_JTAG_GOTOSHIFT              = 39,  //!< Move JTAG TAP controller to SHIFT-IR/DR
   CMD_USBDM_JTAG_WRITE                  = 40,  //!< Write to JTAG chain
   CMD_USBDM_JTAG_READ                   = 41,  //!< Read from JTAG chain
   CMD_USBDM_SET_VPP                     = 42,  //!< Set VPP level
   CMD_USBDM_JTAG_READ_WRITE             = 43,  //!< Read & Write to JTAG chain (in-out buffer)
   CMD_USBDM_JTAG_EXECUTE_SEQUENCE       = 44,  //!< Execute sequence of JTAG commands
};

//! Error codes returned from BDM routines and BDM commands.
//!
enum USBDM_ErrorCode {
 BDM_RC_OK                                     = 0,     //!< No error
 BDM_RC_ILLEGAL_PARAMS                         = 1,     //!< Illegal parameters to command
 BDM_RC_FAIL                                   = 2,     //!< General Fail
 BDM_RC_BUSY                                   = 3,     //!< Busy with last command - try again - don't change
 BDM_RC_ILLEGAL_COMMAND                        = 4,     //!< Illegal (unknown) command (may be in wrong target mode)
 BDM_RC_NO_CONNECTION                          = 5,     //!< No connection to target
 BDM_RC_OVERRUN                                = 6,     //!< New command before previous command completed
 BDM_RC_CF_ILLEGAL_COMMAND                     = 7,     //!< Coldfire BDM interface did not recognize the command
 BDM_RC_DEVICE_OPEN_FAILED                     = 8,     //!< BDM Open Failed - Other LIBUSB error on open
 BDM_RC_USB_DEVICE_BUSY                        = 9,     //!< BDM Open Failed - LIBUSB_ERROR_ACCESS on open - Probably open in another app
 BDM_RC_USB_DEVICE_NOT_INSTALLED               = 10,    //!< BDM Open Failed - LIBUSB_ERROR_ACCESS on claim I/F - Probably driver not installed
 BDM_RC_USB_DEVICE_REMOVED                     = 11,    //!< BDM Open Failed - LIBUSB_ERROR_NO_DEVICE - enumerated device has been removed
 BDM_RC_USB_RETRY_OK                           = 12,    //!< USB Debug use only
 BDM_RC_UNEXPECTED_RESET                       = 13,    //!< Target reset was detected
 BDM_RC_CF_NOT_READY                           = 14,    //!< Coldfire 2,3,4 Not ready response
 BDM_RC_UNKNOWN_TARGET                         = 15,    //!< Target unknown or not supported by this BDM
 BDM_RC_NO_TX_ROUTINE                          = 16,    //!< No Tx routine available at measured BDM communication speed
 BDM_RC_NO_RX_ROUTINE                          = 17,    //!< No Rx routine available at measured BDM communication speed
 BDM_RC_BDM_EN_FAILED                          = 18,    //!< Failed to enable BDM mode in target (warning)
 BDM_RC_RESET_TIMEOUT_FALL                     = 19,    //!< RESET signal failed to fall
 BDM_RC_BKGD_TIMEOUT                           = 20,    //!< BKGD signal failed to rise/fall
 BDM_RC_SYNC_TIMEOUT                           = 21,    //!< No response to SYNC sequence
 BDM_RC_UNKNOWN_SPEED                          = 22,    //!< Communication speed is not known or cannot be determined
 BDM_RC_WRONG_PROGRAMMING_MODE                 = 23,    //!< Attempted Flash programming when in wrong mode (e.g. Vpp off)
 BDM_RC_FLASH_PROGRAMING_BUSY                  = 24,    //!< Busy with last Flash programming command
 BDM_RC_VDD_NOT_REMOVED                        = 25,    //!< Target Vdd failed to fall
 BDM_RC_VDD_NOT_PRESENT                        = 26,    //!< Target Vdd not present/failed to rise
 BDM_RC_VDD_WRONG_MODE                         = 27,    //!< Attempt to cycle target Vdd when not controlled by BDM interface
 BDM_RC_CF_BUS_ERROR                           = 28,    //!< Illegal bus cycle on target (Coldfire)
 BDM_RC_USB_ERROR                              = 29,    //!< Indicates USB transfer failed (returned by driver not BDM)
 BDM_RC_ACK_TIMEOUT                            = 30,    //!< Indicates an expected ACK was missing
 BDM_RC_FAILED_TRIM                            = 31,    //!< Trimming of target clock failed (out of clock range?).
 BDM_RC_FEATURE_NOT_SUPPORTED                  = 32,    //!< Feature not supported by this version of hardware/firmware
 BDM_RC_RESET_TIMEOUT_RISE                     = 33,    //!< RESET signal failed to rise

 // Used by USBDM DLL
 BDM_RC_WRONG_BDM_REVISION                     = 34,    //!< BDM Hardware is incompatible with driver/program
 BDM_RC_WRONG_DLL_REVISION                     = 35,    //!< Program is incompatible with DLL
 BDM_RC_NO_USBDM_DEVICE                        = 36,    //!< No USBDM device was located

 BDM_RC_JTAG_UNMATCHED_REPEAT                  = 37,    //!< Unmatched REPEAT-END_REPEAT
 BDM_RC_JTAG_UNMATCHED_RETURN                  = 38,    //!< Unmatched CALL-RETURN
 BDM_RC_JTAG_UNMATCHED_IF                      = 39,    //!< Unmatched IF-END_IF
 BDM_RC_JTAG_STACK_ERROR                       = 40,    //!< Underflow in call/return sequence, unmatched REPEAT etc.
 BDM_RC_JTAG_ILLEGAL_SEQUENCE                  = 41,    //!< Illegal JTAG sequence
 BDM_RC_TARGET_BUSY                            = 42,    //!< Target is busy (executing?)
 BDM_RC_JTAG_TOO_LARGE                         = 43,    //!< Subroutine is too large to cache
 BDM_RC_DEVICE_NOT_OPEN                        = 44,    //!< USBDM Device has not been opened
 BDM_RC_UNKNOWN_DEVICE                         = 45,    //!< Device is not in database
 BDM_RC_DEVICE_DATABASE_ERROR                  = 46,    //!< Device database not found or failed to open/parse

 BDM_RC_ARM_PWR_UP_FAIL                        = 47,    //!< ARM System power failed
 BDM_RC_ARM_ACCESS_ERROR                       = 48,    //!< ARM Access error

 BDM_JTAG_TOO_MANY_DEVICES                     = 49,    //!< JTAG chain is too long (or greater than 1!)

 BDM_RC_SECURED                                = 50,    //!< ARM Device is secured (& operation failed?)
 BDM_RC_ARM_PARITY_ERROR                       = 51,    //!< ARM PARITY error
 BDM_RC_ARM_FAULT_ERROR                        = 52,    //!< ARM FAULT response error
 BDM_RC_UNEXPECTED_RESPONSE                    = 53,    //!< Unexpected/inconsistent response from BDM
 BDM_RC_HCS_ACCESS_ERROR                       = 54,    //!< Memory access failed due to target in stop or wait state
 BDM_RC_CF_DATA_INVALID                        = 55,    //!< CF target returned data invalid response (whatever that means!)
 BDM_RC_CF_OVERRUN                             = 56,    //!< CF target returned overrun response
 BDM_RC_MASS_ERASE_DISABLED                    = 57,    //!< ARM Device has mass erase disabled
 BDM_RC_FLASH_NOT_READY                        = 58,    //!< ARM - Flash failed to become ready
};

//! Capabilities of the hardware
//!
enum HardwareCapabilities_t {
	   BDM_CAP_NONE         = (0),
	   BDM_CAP_ALL          = (0xFFFF),
	   BDM_CAP_HCS12        = (1<<0),   //!< Supports HCS12
	   BDM_CAP_RS08         = (1<<1),   //!< 12 V Flash programming supply available (RS08 support)
	   BDM_CAP_VDDCONTROL   = (1<<2),   //!< Control over target Vdd
	   BDM_CAP_VDDSENSE     = (1<<3),   //!< Sensing of target Vdd
	   BDM_CAP_CFVx         = (1<<4),   //!< Support for CFV 1,2 & 3
	   BDM_CAP_HCS08        = (1<<5),   //!< Supports HCS08 targets - inverted when queried
	   BDM_CAP_CFV1         = (1<<6),   //!< Supports CFV1 targets  - inverted when queried
	   BDM_CAP_JTAG         = (1<<7),   //!< Supports JTAG targets
	   BDM_CAP_DSC          = (1<<8),   //!< Supports DSC targets
	   BDM_CAP_ARM_JTAG     = (1<<9),   //!< Supports ARM targets via JTAG
	   BDM_CAP_RST          = (1<<10),  //!< Control & sensing of RESET
	   BDM_CAP_PST          = (1<<11),  //!< Supports PST signal sensing
	   BDM_CAP_CDC          = (1<<12),  //!< Supports CDC Serial over USB interface
	   BDM_CAP_ARM_SWD      = (1<<13),  //!< Supports ARM targets via SWD
	   BDM_CAP_HCS12Z       = (1<<14),  //!< Supports HCS12Z targets via SWD
};

//===================================================================================
//!  Target microcontroller types
//!
enum TargetType_t {
   T_HC12      = 0,       //!< HC12 or HCS12 target
   T_HCS12     = T_HC12,  //!< HC12 or HCS12 target
   T_HCS08     = 1,       //!< HCS08 target
   T_RS08      = 2,       //!< RS08 target
   T_CFV1      = 3,       //!< Coldfire Version 1 target
   T_CFVx      = 4,       //!< Coldfire Version 2,3,4 target
   T_JTAG      = 5,       //!< JTAG target - TAP is set to \b RUN-TEST/IDLE
   T_EZFLASH   = 6,       //!< EzPort Flash interface (SPI?)
   T_MC56F80xx = 7,       //!< JTAG target with MC56F80xx optimised subroutines
   T_ARM_JTAG  = 8,       //!< ARM target using JTAG
   T_ARM_SWD   = 9,       //!< ARM target using SWD
   T_ARM       = 10,      //!< ARM target using either SWD (preferred) or JTAG as supported
   T_S12Z      = 11,      //!< S12Z target
   T_LAST      = T_S12Z,
   T_ILLEGAL   = 0xFE,    //!< Used to indicate error in selecting target
   T_OFF       = 0xFF,    //!< Turn off interface (no target)
   T_NONE      = 0xFF,
};

//! Memory space indicator - includes element size
//!
enum MemorySpace_t {
   // One of the following
   MS_Byte     = 1,        //! Byte (8-bit) access
   MS_Word     = 2,        //! Word (16-bit) access
   MS_Long     = 4,        //! Long (32-bit) access
   // One of the following
   MS_None     = 0<<4,     //! Memory space unused/undifferentiated
   MS_Program  = 1<<4,     //! Program memory space (e.g. P: on DSC)
   MS_Data     = 2<<4,     //! Data memory space (e.g. X: on DSC)
   MS_Global   = 3<<4,     //! HCS12 Global addresses (Using BDMPPR register)

   MS_Fast     = 1<<7,     //! Fast memory access for HCS08/HCS12 (stopped target, regs. are modified

   // Masks for above
   MS_SIZE     = 0x7<<0,   //! Size
   MS_SPACE    = 0x7<<4,   //! Memory space

   // For convenience (DSC)
   MS_PWord    = MS_Word+MS_Program,
   MS_PLong    = MS_Long+MS_Program,
   MS_XByte    = MS_Byte+MS_Data,
   MS_XWord    = MS_Word+MS_Data,
   MS_XLong    = MS_Long+MS_Data,
};

//! Target supports ACKN or uses fixed delay {WAIT} instead
//!
enum AcknMode_t {
   WAIT  = 0,   //!< Use WAIT (delay) instead
   ACKN  = 1,   //!< Target supports ACKN feature and it is enabled
};

//! Target speed selection
//!
enum SpeedMode_t {
   SPEED_NO_INFO        = 0,   //!< Not connected
   SPEED_SYNC           = 1,   //!< Speed determined by SYNC
   SPEED_GUESSED        = 2,   //!< Speed determined by trial & error
   SPEED_USER_SUPPLIED  = 3    //!< User has specified the speed to use
};

//! Target RSTO state
//!
enum ResetState_t {
   RSTO_ACTIVE=0,     //!< RSTO* is currently active [low]
   RSTO_INACTIVE=1    //!< RSTO* is currently inactive [high]
};

//! Target reset status values
//!
enum ResetMode_t {
   NO_RESET_ACTIVITY    = 0,   //!< No reset activity since last polled
   RESET_INACTIVE       = NO_RESET_ACTIVITY,
   RESET_DETECTED       = 1    //!< Reset since last polled
};

//! Target Halt state
//!
enum TargetRunState_t {
   TARGET_RUNNING    = 0,   //!< CFVx target running (ALLPST == 0)
   TARGET_HALTED     = 1    //!< CFVx target halted (ALLPST == 1)
};

//! Target Voltage supply state
//!
enum TargetVddState_t {
   BDM_TARGET_VDD_NONE      = 0,   //!< Target Vdd not detected
   BDM_TARGET_VDD_EXT       = 1,   //!< Target Vdd external
   BDM_TARGET_VDD_INT       = 2,   //!< Target Vdd internal
   BDM_TARGET_VDD_ERR       = 3,   //!< Target Vdd error
};

//! Auto-reconnect options
//!
enum AutoConnect_t {
   AUTOCONNECT_NEVER   = 0,  //!< Only connect explicitly
   AUTOCONNECT_STATUS  = 1,  //!< Reconnect on USBDM_ReadStatusReg()
   AUTOCONNECT_ALWAYS  = 2,  //!< Reconnect before every command
};

//====================================================================================

//! Internal Target Voltage supply selection
//!
enum TargetVddSelect_t {
   BDM_TARGET_VDD_OFF       = 0,     //!< Target Vdd Off
   BDM_TARGET_VDD_3V3       = 1,     //!< Target Vdd internal 3.3V
   BDM_TARGET_VDD_5V        = 2,     //!< Target Vdd internal 5.0V
   BDM_TARGET_VDD_ENABLE    = 0x10,  //!< Target Vdd internal at last set level
   BDM_TARGET_VDD_DISABLE   = 0x11,  //!< Target Vdd Off but previously set level unchanged
};

//! Internal Programming Voltage supply selection
//!
enum TargetVppSelect_t {
   BDM_TARGET_VPP_OFF       = 0,   //!< Target Vpp Off
   BDM_TARGET_VPP_STANDBY   = 1,   //!< Target Vpp Standby (Inverter on, Vpp off)
   BDM_TARGET_VPP_ON        = 2,   //!< Target Vpp On
   BDM_TARGET_VPP_ERROR     = 3,   //!< Target Vpp ??
};

//! Target BDM Clock selection
//!
enum ClkSwValues_t {
   CS_DEFAULT           = 0xFF,  //!< Use default clock selection (don't modify target's reset default)
   CS_ALT_CLK           =  0,    //!< Force ALT clock (CLKSW = 0)
   CS_NORMAL_CLK        =  1,    //!< Force Normal clock (CLKSW = 1)
};

//!  Reset mode as used by CMD_USBDM_TARGET_RESET
//!
enum TargetMode_t { /* type of reset action required */
   RESET_MODE_MASK   = (3<<0), //!< Mask for reset mode (SPECIAL/NORMAL)
   RESET_SPECIAL     = (0<<0), //!< Special mode [BDM active, Target halted]
   RESET_NORMAL      = (1<<0), //!< Normal mode [usual reset, Target executes]

   RESET_METHOD_MASK = (7<<2), //!< Mask for reset type (Hardware/Software/Power)
   RESET_ALL         = (0<<2), //!< Use all reset strategies as appropriate
   RESET_HARDWARE    = (1<<2), //!< Use hardware RESET pin reset
   RESET_SOFTWARE    = (2<<2), //!< Use software (BDM commands) reset
   RESET_POWER       = (3<<2), //!< Cycle power
   RESET_DEFAULT     = (7<<2), //!< Use target specific default method
};

//=======================================================================
//
// regNo Parameter values for USBDM_ReadReg()
//
//=======================================================================

//! regNo Parameter for USBDM_ReadReg() with HCS12 target
//!
//! @note CCR is accessed through USBDM_ReadDReg()
enum HCS12_Registers_t {
   HCS12_RegPC    = 3,    //!< PC reg
   HCS12_RegD     = 4,    //!< D reg
   HCS12_RegX     = 5,    //!< X reg
   HCS12_RegY     = 6,    //!< Y reg
   HCS12_RegSP    = 7,    //!< SP reg
   HCS12_RegCCR   = 0x80, //!< CCR reg - redirected to USBDM_ReadDReg()
};

//! regNo Parameter for USBDM_ReadReg() with HCS12 target
//!
//! @note CCR is accessed through USBDM_ReadDReg()
enum S12Z_Registers_t {
   S12Z_RegD0   = 0x0, //!< D0 reg
   S12Z_RegD1   = 0x1, //!< D1 reg
   S12Z_RegD2   = 0x2, //!< D2 reg
   S12Z_RegD3   = 0x3, //!< D3 reg
   S12Z_RegD4   = 0x4, //!< D4 reg
   S12Z_RegD5   = 0x5, //!< D5 reg
   S12Z_RegD6   = 0x6, //!< D6 reg
   S12Z_RegD7   = 0x7, //!< D7 reg
   S12Z_RegX    = 0x8, //!< X reg
   S12Z_RegY    = 0x9, //!< Y reg
   S12Z_RegSP   = 0xA, //!< SP reg
   S12Z_RegPC   = 0xB, //!< PC reg
   S12Z_RegCCR  = 0xC, //!< CCR reg
};

//! regNo Parameter for USBDM_ReadReg() with HCS08 target
//!
enum HCS08_Registers_t {
   HCS08_RegPC  = 0xB,  //!< PC  reg
   HCS08_RegSP  = 0xF,  //!< SP  reg
   HCS08_RegHX  = 0xC,  //!< HX  reg
   HCS08_RegA   = 8,    //!< A   reg
   HCS08_RegCCR = 9,    //!< CCR reg
};

//! regNo Parameter for USBDM_ReadReg() with RS08 target
//!
enum RS08_Registers_t {
   RS08_RegCCR_PC  = 0xB, //!< Combined CCR/PC register
   RS08_RegSPC     = 0xF, //!< Shadow PC
   RS08_RegA       = 8,   //!< A reg
};

//! regNo Parameter for USBDM_ReadReg() with CFV1 target
//!
enum CFV1_Registers_t {
   CFV1_RegD0     = 0,  //!< D0
   CFV1_RegD1     = 1,  //!< D1
   CFV1_RegD2     = 2,  //!< D2
   CFV1_RegD3     = 3,  //!< D3
   CFV1_RegD4     = 4,  //!< D4
   CFV1_RegD5     = 5,  //!< D5
   CFV1_RegD6     = 6,  //!< D6
   CFV1_RegD7     = 7,  //!< D7
   CFV1_RegA0     = 8,  //!< A0
   CFV1_RegA1     = 9,  //!< A1
   CFV1_RegA2     = 10, //!< A2
   CFV1_RegA3     = 11, //!< A3
   CFV1_RegA4     = 12, //!< A4
   CFV1_RegA5     = 13, //!< A5
   CFV1_RegA6     = 14, //!< A6
   CFV1_RegA7     = 15, //!< A7
   CFV1_RegSP     = CFV1_RegA7,
   CFV1_PSTBASE   = 16, //!< Start of PST registers, access as CFV1_PSTBASE+n
   // The following are used internally by the BDM and only available
   // externally from firmware version 4.10.6
   // Takes advantage of the similarity b/w READ_Rn and READ_DREG
   CFV1_RegOTHER_A7  = 0xC0|0,  //!< Other A7 (not active in target)
   CFV1_RegVBR       = 0xC0|1,  //!< Vector Base register
   CFV1_RegCPUCR     = 0xC0|2,  //!< CPUCR
   CFV1_RegMACSR     = 0xC0|4,  //!< MACSR
   CFV1_RegMASK      = 0xC0|5,  //!< MASK
   CFV1_RegACC       = 0xC0|6,  //!< ACC
   CFV1_RegSR        = 0xC0|14, //!< Status register
   CFV1_RegPC        = 0xC0|15, //!< Program Counter
};

//! regNo Parameter for USBDM_ReadReg() with CFVx target
//!
enum CFVx_Registers_t {
   CFVx_RegD0  = 0,          //!< D0
   CFVx_RegD1  = 1,          //!< D1
   CFVx_RegD2  = 2,          //!< D2
   CFVx_RegD3  = 3,          //!< D3
   CFVx_RegD4  = 4,          //!< D4
   CFVx_RegD5  = 5,          //!< D5
   CFVx_RegD6  = 6,          //!< D6
   CFVx_RegD7  = 7,          //!< D7
   CFVx_RegA0  = 8,          //!< A0
   CFVx_RegA1  = 9,          //!< A1
   CFVx_RegA2  = 10,         //!< A2
   CFVx_RegA3  = 11,         //!< A3
   CFVx_RegA4  = 12,         //!< A4
   CFVx_RegA5  = 13,         //!< A5
   CFVx_RegA6  = 14,         //!< A6
   CFVx_RegA7  = 15,         //!< A7
   CFVx_RegSP  = CFVx_RegA7,
};

//! regNo Parameter for ARM_ReadReg() with ARM (Kinetis) target
//!
enum ARM_Registers_t {
   ARM_RegR0     = 0,    //!< R0
   ARM_RegR1     = 1,    //!< R1
   ARM_RegR2     = 2,    //!< R2
   ARM_RegR3     = 3,    //!< R3
   ARM_RegR4     = 4,    //!< R4
   ARM_RegR5     = 5,    //!< R5
   ARM_RegR6     = 6,    //!< R6
   ARM_RegR7     = 7,    //!< R7
   ARM_RegR8     = 8,    //!< R8
   ARM_RegR9     = 9,    //!< R9
   ARM_RegR10    = 10,   //!< R10
   ARM_RegR11    = 11,   //!< R11
   ARM_RegR12    = 12,   //!< R12
   ARM_RegSP     = 13,   //!< SP
   ARM_RegLR     = 14,   //!< LR
   ARM_RegPC     = 15,   //!< PC (Debug return address)
   ARM_RegxPSR   = 16,   //!< xPSR
   ARM_RegMSP    = 17,   //!< Main Stack pointer
   ARM_RegPSP    = 18,   //!< Process Stack pointer
   ARM_RegMISC   = 20,   //!< [31:24]=CONTROL,[23:16]=FAULTMASK,[15:8]=BASEPRI,[7:0]=PRIMASK.
   //
   ARM_RegFPSCR  = 0x21, //!< Floating point control register
   ARM_RegFPS0   = 0x40, //!< Floating point +0..+31
};

//! startRegIndex, endRegIndex Parameters for USBDM_ReadMultipleRegs() with ARM (Kinetis) target
//!
enum ARM_RegisterIndex_t {
   ARM_RegIndexFirstCore  = 0,                          //!< First code reg
   ARM_RegIndexLastCore   = ARM_RegIndexFirstCore+19,   //!< Last core reg (20 regs R0..R12,SP,LR,PC,XPSR,MSP,PSP,MISC)
   ARM_RegIndexFirstFloat = 20,                         //!< First float register
   ARM_RegIndexLastFloat  = ARM_RegIndexFirstFloat+32,  //!< Last float reg (33 regs FPSCR, FPS0..FPS32)
};

//! startRegIndex, endRegIndex Parameters for USBDM_ReadMultipleRegs() with Coldfire V1 target
//!
enum CFV1_RegisterIndex_t {
   CFV1_RegIndexFirstCore  = 0,                          //!< First code reg
   CFV1_RegIndexLastCore   = CFV1_RegIndexFirstCore+17,  //!< Last core reg (18 regs D0..D7,A0..A7,SR,PC)
};

//! startRegIndex, endRegIndex Parameters for USBDM_ReadMultipleRegs() with Coldfire Vx target
//!
enum CFVx_RegisterIndex_t {
   CFVx_RegIndexFirstCore  = 0,                          //!< First code reg
   CFVx_RegIndexLastCore   = CFV1_RegIndexFirstCore+17,  //!< Last core reg (18 regs D0..D7,A0..A7,SR,PC)
};

//! regNo Parameter for DSC_ReadReg() with DSC target
//! DSC Core registers
//!
enum DSC_Registers_t {
   // Core registers
   DSC_RegX0,
   DSC_FirstCoreRegister = DSC_RegX0,          // 0
   DSC_RegY0,
   DSC_RegY1,
   DSC_RegA0,
   DSC_RegA1,
   DSC_RegA2,
   DSC_RegB0,
   DSC_RegB1,
   DSC_RegB2,
   DSC_RegC0,
   DSC_RegC1,                                  // 10
   DSC_RegC2,
   DSC_RegD0,
   DSC_RegD1,
   DSC_RegD2,
   DSC_RegOMR,
   DSC_RegSR,
   DSC_RegLA,
   DSC_RegLA2, /* read only */
   DSC_RegLC,
   DSC_RegLC2, /* read only */                 //  20
   DSC_RegHWS0,
   DSC_RegHWS1,
   DSC_RegSP,
   DSC_RegN3,
   DSC_RegM01,
   DSC_RegN,
   DSC_RegR0,
   DSC_RegR1,
   DSC_RegR2,
   DSC_RegR3,                                  // 30
   DSC_RegR4,
   DSC_RegR5,
   DSC_RegsHM01,
   DSC_RegsHN,
   DSC_RegsHR0,
   DSC_RegsHR1,
   DSC_RegPC,
   DSC_LastCoreRegister = DSC_RegPC,           // 37
   // JTAG registers
   DSC_RegIDCODE,                              // JTAG Core IDCODE
   // ONCE registers
   DSC_RegOCR,                                 // ONCE Control register
   DSC_FirstONCERegister = DSC_RegOCR,         // 39
   DSC_RegOSCNTR,                              // ONCE Instruction Step Counter
   DSC_RegOSR,                                 // ONCE Status register
   DSC_RegOPDBR,                               // ONCE Program Data Bus Register
   DSC_RegOBASE,                               // ONCE Peripheral Base Address regitsre
   DSC_RegOTXRXSR,                             // ONCE Tx & Rx Status & Control register
   DSC_RegOTX,                                 // ONCE Transmit register (32-bit)
   DSC_RegOTX1,                                // ONCE Transmit register (16-bit)
   DSC_RegORX,                                 // ONCE Receive register (32-bit)
   DSC_RegORX1,                                // ONCE Receive register (16-bit)
   DSC_RegOTBCR,                               // ONCE Trace buffer control register
   DSC_RegOTBPR,                               // ONCE Trace Buffer Pointer register
   DSC_RegOTB,                                 // Trace Buffer Register Stages
   DSC_RegOB0CR,                               // Breakpoint Unit 0 Control register
   DSC_RegOB0AR1,                              // Breakpoint Unit 0 Address register 1
   DSC_RegOB0AR2,                              // Breakpoint Unit 0 Address register 2
   DSC_RegOB0MSK,                              // Breakpoint Unit 0 Mask register
   DSC_RegOB0CNTR,                             // Breakpoint Unit 0 Counter
   DSC_LastONCERegister = DSC_RegOB0CNTR,      // 58

   DSC_GdiStatus = 0x1001,                     // Used by stand-alone programmer - dummied
   DSC_UnknownReg = 0xFFFFFF,
};

//=======================================================================
//
// regNo Parameter values for USBDM_ReadCReg()
//
//=======================================================================

//! regNo Parameter for USBDM_ReadCReg() with CFV1 target
//!
enum CFV1_CRegisters_t {
   CFV1_CRegOTHER_A7  = 0,  //!< Other A7 (not active in target)
   CFV1_CRegVBR       = 1,  //!< Vector Base register
   CFV1_CRegCPUCR     = 2,  //!< CPUCR
   CFV1_CRegMACSR     = 4,  //!< MACSR
   CFV1_CRegMASK      = 5,  //!< MASK
   CFV1_CRegACC       = 6,  //!< ACC
   CFV1_CRegSR        = 14, //!< Status register
   CFV1_CRegPC        = 15, //!< Program Counter
};

//! regNo Parameter for USBDM_ReadCReg() with CFVx target
//! Note - These values vary with processor
enum CFVx_CRegisters_t {
//   CFVx_CRegD0        = 0x80, //!< D0-D7 not available on all targets
//   CFVx_CRegD1,
//   CFVx_CRegD2,
//   CFVx_CRegD3,
//   CFVx_CRegD4,
//   CFVx_CRegD5,
//   CFVx_CRegD6,
//   CFVx_CRegD7,
//   CFVx_CRegA0,               //!< A0-A7
//   CFVx_CRegA1,
//   CFVx_CRegA2,
//   CFVx_CRegA3,
//   CFVx_CRegA4,
//   CFVx_CRegA5,
//   CFVx_CRegA6,
   CFVx_CRegUSER_SP,
   CFVx_CRegOTHER_SP  = 0x800, //!< Other A7 (not active in target)
   CFVx_CRegVBR       = 0x801, //!< Vector Base register
   CFVx_CRegSR        = 0x80E, //!< Status Register
   CFVx_CRegPC        = 0x80F, //!< Program Counter
   CFV1_CRegFLASHBAR  = 0xC04, //!< Flash Base register
   CFV1_CRegRAMBAR    = 0xC05, //!< RAM Base register
   // May be others
};

//! regNo Parameter for USBDM_ReadCReg() with SWD-ARM target
//!
//! The regNo is actually a AP bus address as follows:
//!   -  A[31:24]  => DP-AP-SELECT[31:24] (AP # Select)
//!   -  A[23:8]   => unused (0)
//!   -  A[7:4]    => DP-AP-SELECT[7:4]   (Bank select within AP)
//!   -  A[3:2]    => APACC[3:2]          (Register select within AP bank)
//!   -  A[1:0]    => unused (0)
//!
enum ARM_CRegisters_t {
   // AP#0 - Common ARM AHB-AP
   ARM_CRegAHB_AP_CSW      = 0x00000000U,   //!< AHB-AP Control/Status Word register
   ARM_CRegAHB_AP_TAR      = 0x00000004U,   //!< AHB-AP Transfer Address register
   ARM_CRegAHB_AP_DRW      = 0x0000000CU,   //!< AHB-AP Data Read/Write register

   ARM_CRegAHB_AP_CFG      = 0x000000F4U,   //!< AHB-AP Config register
   ARM_CRegAHB_AP_Base     = 0x000000F8U,   //!< AHB-AP IDebug base address register
   ARM_CRegAHB_AP_Id       = 0x000000FCU,   //!< AHB-AP ID Register

   // AP#1 - Kinetis MDM-AP registers
   ARM_CRegMDM_AP_Status   = 0x01000000U,   //!< Status register
   ARM_CRegMDM_AP_Control  = 0x01000004U,   //!< Control register
   ARM_CRegMDM_AP_Ident    = 0x010000FCU,   //!< Identifier register (should read 0x001C_0000)
};

//=======================================================================
//
// regNo Parameter values for USBDM_ReadDReg()
//
//=======================================================================

//! regNo Parameter for USBDM_ReadDReg() with HCS12 target [BD Space]
//!
//! @note: There may be other registers
//!
enum HCS12_DRegisters_t {
   // 8-bit accesses using READ_BD_BYTE
   HCS12_DRegBDMSTS = (int)0xFF01, //!< - BDMSTS (debug status/control) register
   HCS12_DRegCCR    = (int)0xFF06, //!< - Saved Target CCR
   HCS12_DRegBDMINR = (int)0xFF07, //!< - BDM Internal Register Position Register
   // Others may be device dependent
};

//! regNo Parameter for USBDM_ReadDReg() with HCS08 target [BKPT reg]
//!
enum HCS08_DRegisters_t {
   HCS08_DRegBKPT = 0x0, //!< Breakpoint register
};

//! regNo Parameter for USBDM_ReadDReg() with RS08 target (BKPT)
//!
enum RS08_DRegisters_t {
   RS08_DRegBKPT = 0x0, //!< Breakpoint register
};

//! regNo Parameter for USBDM_ReadDReg() with CFV1 target
//!
//! @note: There may be other registers
enum CFV1_DRegisters_t {
   CFV1_DRegCSR        = 0x00,   //!< CSR
   CFV1_DRegXCSR       = 0x01,   //!< XCSR
   CFV1_DRegCSR2       = 0x02,   //!< CSR2
   CFV1_DRegCSR3       = 0x03,   //!< CSR3
   CFV1_DRegBAAR       = 0x05,   //!< BAAR
   CFV1_DRegAATR       = 0x06,   //!< AATR
   CFV1_DRegTDR        = 0x07,   //!< TDR
   CFV1_DRegPBR0       = 0x08,   //!< PBR0
   CFV1_DRegPBMR       = 0x09,   //!< PBMR - mask for PBR0
   CFV1_DRegABHR       = 0x0C,   //!< ABHR
   CFV1_DRegABLR       = 0x0D,   //!< ABLR
   CFV1_DRegDBR        = 0x0E,   //!< DBR
   CFV1_DRegBDMR       = 0x0F,   //!< DBMR - mask for DBR
   CFV1_DRegPBR1       = 0x18,   //!< PBR1
   CFV1_DRegPBR2       = 0x1A,   //!< PBR2
   CFV1_DRegPBR3       = 0x1B,   //!< PBR3

   CFV1_ByteRegs       = 0x1000, // Special access to MSB
   CFV1_DRegXCSRbyte   = CFV1_ByteRegs+CFV1_DRegXCSR, //!< XCSR.msb
   CFV1_DRegCSR2byte   = CFV1_ByteRegs+CFV1_DRegCSR2, //!< CSR2.msb
   CFV1_DRegCSR3byte   = CFV1_ByteRegs+CFV1_DRegCSR3, //!< CSR3.msb
};

//! regNo Parameter for USBDM_ReadDReg() with CFV1 target
//!
enum CFVx_DRegisters_t {
   CFVx_DRegCSR    = 0x00, //!< CSR
   CFVx_DRegBAAR   = 0x05, //!< BAAR
   CFVx_DRegAATR   = 0x06, //!< AATR
   CFVx_DRegTDR    = 0x07, //!< TDR
   CFVx_DRegPBR0   = 0x08, //!< PBR0
   CFVx_DRegPBMR   = 0x09, //!< PBMR - mask for PBR0
   CFVx_DRegABHR   = 0x0C, //!< ABHR
   CFVx_DRegABLR   = 0x0D, //!< ABLR
   CFVx_DRegDBR    = 0x0E, //!< DBR
   CFVx_DRegBDMR   = 0x0F, //!< DBMR - mask for DBR
   CFVx_DRegPBR1   = 0x18, //!< PBR1
   CFVx_DRegPBR2   = 0x1A, //!< PBR2
   CFVx_DRegPBR3   = 0x1B, //!< PBR3
};

//! regNo Parameter for USBDM_ReadDReg() with SWD-ARM target
//!
enum ARM_DRegisters_t {
   ARM_DRegIDCODE  = 0,    //!< IDCODE  reg - read, SWD-AP only
   ARM_DRegABORT   = 0,    //!< ABORT   reg - write only
   ARM_DRegSTATUS  = 1,    //!< STATUS  reg - read only
   ARM_DRegCONTROL = 1,    //!< CONTROL reg - write only
   ARM_DRegRESEND  = 2,    //!< RESEND  reg - read only
   ARM_DRegSELECT  = 2,    //!< SELECT  reg - write only
   ARM_DRegRDBUFF  = 3,    //!< RDBUFF  reg - read only

   ARM_DRegAPReg0  = 4,    //!< AP reg #0
   ARM_DRegAPReg1  = 5,    //!< AP reg #1
   ARM_DRegAPReg2  = 6,    //!< AP reg #2
   ARM_DRegAPReg3  = 7,    //!< AP reg #3
};

//=======================================================================
//
//=======================================================================


//! State of BDM Communication
//!
struct USBDMStatus_t {
   TargetType_t         target_type;       //!< Type of target (HCS12, HCS08 etc) @deprecated
   AcknMode_t           ackn_state;        //!< Supports ACKN ?
   SpeedMode_t          connection_state;  //!< Connection status & speed determination method
   ResetState_t         reset_state;       //!< Current target RST0 state
   ResetMode_t          reset_recent;      //!< Target reset recently?
   TargetRunState_t     halt_state;        //!< CFVx halted (from ALLPST)?
   TargetVddState_t     power_state;       //!< Target has power?
   TargetVppSelect_t    flash_state;       //!< State of Target Vpp
};

//=======================================================================
//
//  JTAG Interface
//
//=======================================================================

//! Options used with JTAG commands
//!
enum JTAG_ExitActions_t {
   JTAG_STAY_SHIFT       = 0,     //!< Remain in SHIFT-DR or SHIFT-IR
   JTAG_EXIT_IDLE        = 1,     //!< Exit SHIFT-XX to RUN-TEST/IDLE
   JTAG_EXIT_SHIFT_DR    = 2,     //!< Exit SHIFT-XX & enter SHIFT-DR w/o crossing RUN-TEST/IDLE
   JTAG_EXIT_SHIFT_IR    = 3,     //!< Exit SHIFT-XX & enter SHIFT-IR w/o crossing RUN-TEST/IDLE
   JTAG_EXIT_ACTION_MASK = 0x3,   //!< Mask for Exit actions

   JTAG_WRITE_0          = 0x00,  //!< Write 0's when reading - combined with above
   JTAG_WRITE_1          = 0x80,  //!< Write 1's when reading - combined with above
   JTAG_WRITE_MASK       = 0x80,  //!< Mask for Write actions

   JTAG_SHIFT_DR         = 0,     //!< Enter SHIFT-DR (from TEST-LOGIC-RESET or RUN-TEST/IDLE)
   JTAG_SHIFT_IR         = 1,     //!< Enter SHIFT-IR (from TEST-LOGIC-RESET or RUN-TEST/IDLE)
};

//! Error codes returned by JMxx BDM when in ICP mode
//!
enum ICP_ErrorCode_t {
   ICP_RC_OK          = 0,    //!< No error
   ICP_RC_ILLEGAL     = 1,    //!< Illegal command or parameters
   ICP_RC_FLASH_ERR   = 2,    //!< Flash failed to program etc
   ICP_RC_VERIFY_ERR  = 3,    //!< Verify failed
};

//! Target Status bit masks for CMD_USBDM_GET_BDM_STATUS\n
//! \verbatim
//!     9       8       7       6       5        4       3       2       1       0
//! +-------+-------+-------+-------+--------+-------+-------+-------+-------+-------+
//! |      VPP      |     Power     |  Halt  | Communication | Reset | ResDet| Ackn  |
//! +-------+-------+-------+-------+--------+-------+-------+-------+-------+-------+
//! \endverbatim
enum StatusBitMasks_t {
   S_ACKN            = (1<<0),  //!< - Target supports BDM ACK (HCS08/12/CFV1)

   S_RESET_DETECT    = (1<<1),  //!< - Target has been reset since status last polled

   S_RESET_STATE     = (1<<2),  //!< - Current state of target reset pin (RESET or RSTO) (active low!)

   S_NOT_CONNECTED   = (0<<3),  //!< - No connection with target
   S_SYNC_DONE       = (1<<3),  //!< - Target communication speed determined by BDM SYNC
   S_GUESS_DONE      = (2<<3),  //!< - Target communication speed guessed
   S_USER_DONE       = (3<<3),  //!< - Target communication speed specified by user
   S_COMM_MASK       = (3<<3),  //!< - Mask for communication state

   S_HALT            = (1<<5),  //!< - Indicates target is halted (CF V2, V3 & V4)

   S_POWER_NONE      = (0<<6),  //!< - Target power not present
   S_POWER_EXT       = (1<<6),  //!< - External target power present
   S_POWER_INT       = (2<<6),  //!< - Internal target power on
   S_POWER_ERR       = (3<<6),  //!< - Internal target power error - over-current or similar
   S_POWER_MASK      = (3<<6),  //!< - Mask for Power

   S_VPP_OFF         = (0<<8),  //!< - Vpp Off
   S_VPP_STANDBY     = (1<<8),  //!< - Vpp Standby (Inverter on)
   S_VPP_ON          = (2<<8),  //!< - Vpp On
   S_VPP_ERR         = (3<<8),  //!< - Vpp Error - not used
   S_VPP_MASK        = (3<<8),  //!< - Mask for Vpp
};


//! Control signal masks for CMD_USBDM_CONTROL_PIN
enum PinLevelMasks_t {
   PIN_BKGD_OFFS      = (0),
   PIN_BKGD_MASK      = (3<<PIN_BKGD_OFFS),  //!< Mask for BKGD values (PIN_BKGD_LOW, PIN_BKGD_HIGH & PIN_BKGD_3STATE)
   PIN_BKGD_NC        = (0<<PIN_BKGD_OFFS),  //!< No change
   PIN_BKGD_3STATE    = (1<<PIN_BKGD_OFFS),  //!< Set BKGD 3-state
   PIN_BKGD_LOW       = (2<<PIN_BKGD_OFFS),  //!< Set BKGD low
   PIN_BKGD_HIGH      = (3<<PIN_BKGD_OFFS),  //!< Set BKGD high

   PIN_RESET_OFFS     = (2),
   PIN_RESET_MASK     = (3<<PIN_RESET_OFFS), //!< Mask for RESET values (PIN_RESET_LOW & PIN_RESET_3STATE)
   PIN_RESET_NC       = (0<<PIN_RESET_OFFS), //!< No change
   PIN_RESET_3STATE   = (1<<PIN_RESET_OFFS), //!< Set Reset 3-state
   PIN_RESET_LOW      = (2<<PIN_RESET_OFFS), //!< Set Reset low
   PIN_RESET_HIGH     = (3<<PIN_RESET_OFFS), //!< Status only - Reset high

   PIN_TA_OFFS        = (4),
   PIN_TA_MASK        = (3<<PIN_TA_OFFS),    //!< Mask for TA signal
   PIN_TA_NC          = (0<<PIN_TA_OFFS),    //!< No change
   PIN_TA_3STATE      = (1<<PIN_TA_OFFS),    //!< Set TA 3-state
   PIN_TA_LOW         = (2<<PIN_TA_OFFS),    //!< Set TA low

   PIN_DE_OFFS        = (4),
   PIN_DE_MASK        = (3<<PIN_DE_OFFS),    //!< Mask for DE signal
   PIN_DE_NC          = (0<<PIN_DE_OFFS),    //!< No change
   PIN_DE_3STATE      = (1<<PIN_DE_OFFS),    //!< Set DE 3-state
   PIN_DE_LOW         = (2<<PIN_DE_OFFS),    //!< Set DE low

   PIN_TRST_OFFS      = (6),
   PIN_TRST_MASK      = (3<<PIN_TRST_OFFS),  //!< Mask for TRST signal (not implemented)
   PIN_TRST_NC        = (0<<PIN_TRST_OFFS),  //!< No change
   PIN_TRST_3STATE    = (1<<PIN_TRST_OFFS),  //!< Set TRST 3-state
   PIN_TRST_LOW       = (2<<PIN_TRST_OFFS),  //!< Set TRST low

   PIN_BKPT_OFFS      = (8),
   PIN_BKPT_MASK      = (3<<PIN_BKPT_OFFS),  //!< Mask for BKPT signal
   PIN_BKPT_NC        = (0<<PIN_BKPT_OFFS),  //!< No change
   PIN_BKPT_3STATE    = (1<<PIN_BKPT_OFFS),  //!< Set BKPT 3-state
   PIN_BKPT_LOW       = (2<<PIN_BKPT_OFFS),  //!< Set BKPT low

   PIN_SWD_OFFS       = (10),
   PIN_SWD_MASK       = (3<<PIN_SWD_OFFS),   //!< Mask for SWD values (PIN_SWD_LOW, PIN_SWD_HIGH & PIN_SWD_3STATE)
   PIN_SWD_NC         = (0<<PIN_SWD_OFFS),   //!< No change
   PIN_SWD_3STATE     = (1<<PIN_SWD_OFFS),   //!< Set SWD 3-state
   PIN_SWD_LOW        = (2<<PIN_SWD_OFFS),   //!< Set SWD low
   PIN_SWD_HIGH       = (3<<PIN_SWD_OFFS),   //!< Set SWD high

   PIN_SWCLK_OFFS     = (12),
   PIN_SWCLK_MASK     = (3<<PIN_SWCLK_OFFS),   //!< Mask for SWD values (PIN_SWCLK_LOW, PIN_SWCLK_HIGH & PIN_SWCLK_3STATE)
   PIN_SWCLK_NC       = (0<<PIN_SWCLK_OFFS),   //!< No change
   PIN_SWCLK_3STATE   = (1<<PIN_SWCLK_OFFS),   //!< Set SWD 3-state
   PIN_SWCLK_LOW      = (2<<PIN_SWCLK_OFFS),   //!< Set SWD low
   PIN_SWCLK_HIGH     = (3<<PIN_SWCLK_OFFS),   //!< Set SWD high

   PIN_NOCHANGE       = 0,    //!< No change to pins (used to get pin status)
   PIN_RELEASE        = -1,   //!< Release all pins (go to default for current target)
};

constexpr PinLevelMasks_t operator| (const PinLevelMasks_t &left, const PinLevelMasks_t &right) {
   return static_cast<PinLevelMasks_t>((unsigned)left|(unsigned)right);
}

//! Debugging sub commands (used with CMD_USBDM_DEBUG )
//! @note Not for general use! (Dangerous - don't try turning on VPP with the wrong chip!)
enum DebugSubCommands {
  BDM_DBG_ACKN             = 0,  //!< - Test ACKN
  BDM_DBG_SYNC             = 1,  //!< - Test SYNC
  BDM_DBG_TESTPORT         = 2,  //!< - Test BDM port timing
  BDM_DBG_USBDISCONNECT    = 3,  //!< - Test USB disconnect (don't use!)
  BDM_DBG_STACKSIZE        = 4,  //!< - Determine stack size
  BDM_DBG_VPP_OFF          = 5,  //!< - Remove Flash programming voltage from target
  BDM_DBG_VPP_ON           = 6,  //!< - Apply Flash programming voltage to target
  BDM_DBG_FLASH12V_OFF     = 7,  //!< - Turn 12V flash programming voltage source off
  BDM_DBG_FLASH12V_ON      = 8,  //!< - Turn 12V flash programming voltage source on
  BDM_DBG_VDD_OFF          = 9,  //!< - Turn Target Vdd supply off
  BDM_DBG_VDD3_ON          = 10, //!< - Set  Target Vdd supply to 3V3
  BDM_DBG_VDD5_ON          = 11, //!< - Set Target Vdd supply to 5V
  BDM_DBG_CYCLE_POWER      = 12, //!< - Cycle Target Vdd supply off and on
  BDM_DBG_MEASURE_VDD      = 13, //!< - Measure Target Vdd supply
  BDM_DBG_RS08TRIM         = 14, //!< - Calculate RS08 clock trim value
  BDM_DBG_TESTWAITS        = 15, //!< - Tests the software counting delays used for BDM communication. (locks up BDM!)
  BDM_DBG_TESTALTSPEED     = 16, //!< - Test bdmHC12_alt_speed_detect{}
  BDM_DBG_TESTBDMTX        = 17, //!< - Test various BDM tx routines with dummy data
  BDM_DBG_SWD              = 18, //!< - Test SWD
  BDM_DBG_ARM              = 19, //!< - Test ARM
  BDM_DBG_SWD_ERASE_LOOP   = 20, //!< - Power on polling to capture difficult chips
};

//! Commands for BDM when in ICP mode
//!
enum ICPCommandCodes {
   ICP_GET_RESULT    =  1,                   //!< Get result of last command
                                             //!< @return [0] 8-bit Error code, see  ICP_ErrorCode_t
   ICP_ERASE_PAGE    =  2,                   //!< Erase page (must be within a single Flash memory page)
                                             //!<   @param 16-bit Address within Flash page to erase
   ICP_PROGRAM_ROW   =  3,                   //!< Program row (must be within a single Flash memory row)
                                             //!<   @param [0..1] 16-bit Address within Flash page to program
                                             //!<   @param [2..3] 16-bit Number of bytes to program
                                             //!<   @param [4..N] data to program
   ICP_VERIFY_ROW    =  4,                   //!< Verify row
                                             //!<   @param [0..1] 16-bit Address within Flash page to verify
                                             //!<   @param [2..3] 16-bit Number of bytes to verify
                                             //!<   @param [4..N] data to verify
   ICP_REBOOT        =  5,                   //!< Reboot device - device immediately reboots so contact is lost!
   ICP_GET_VER       =  CMD_USBDM_GET_VER,   //!< Get version - must be common to both modes
                                             //!< @return [0] 16-bit Version number major.minor
                                             //!< @return Error code, see  ICP_ErrorCode_t
};
#endif
