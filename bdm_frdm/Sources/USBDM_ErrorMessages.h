/*! \file
    \brief USBDM error codes

    \verbatim
    Copyright (C) 2010  Peter O'Donoghue

    Based on material from OSBDM-JM60 Target Interface Software Package
    Copyright (C) 2009  Freescale

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
   +====================================================================
   |    May 2010 | Created
   +====================================================================
    \endverbatim
*/
#ifndef USBDMERRORMESSAGES_H_
#define USBDMERRORMESSAGES_H_

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
 BDM_RC_CF_NOT_READY                           = 14,    //!< Coldfire 2,3,4 Not ready response (running?)
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
 BDM_RC_SELECTED_BDM_NOT_FOUND                 = 55,    //!< Selected BDM not found (removed)
 BDM_RC_NOT_INITIALISED                        = 56,    //!< Interface not initialised before use e.g. failed to call USBDM_Init()
 BDM_RC_OPERATION_NOT_SUPPORTED                = 57,    //!< Operation not supported for target
 BDM_RC_CF_DATA_INVALID                        = 58,    //!< CF target returned data invalid response (whatever that means!)
 BDM_RC_CF_OVERRUN                             = 59,    //!< CF target returned overrun response
 BDM_RC_MASS_ERASE_DISABLED                    = 60,    //!< ARM Device has mass erase disabled
 BDM_RC_FLASH_NOT_READY                        = 61,    //!< ARM - Flash failed to become ready
 BDM_RC_VDD_INCORRECT_LEVEL                    = 62,    //!< Target Vdd not at expected level (only applicable when internally controlled)

 // Used by programmer
 PROGRAMMING_RC_OK                             = 0,     //!<  0 Success
 PROGRAMMING_RC_ERROR_FIRST_MESSAGE            = 101,
 PROGRAMMING_RC_ERROR_ILLEGAL_PARAMS           = 101,   //!<  1 Programming parameters incorrect
 PROGRAMMING_RC_ERROR_WRONG_SDID               = 102,   //!<  2 Incorrect target device
 PROGRAMMING_RC_ERROR_FAILED_VERIFY            = 103,   //!<  3 Verification of Flash failed
 PROGRAMMING_RC_ERROR_BDM                      = 104,   //!<  4 General BDM error
 PROGRAMMING_RC_ERROR_NOT_BLANK                = 105,   //!<  5 Device is not blank/failed erase
 PROGRAMMING_RC_ERROR_BDM_NO_DEVICES           = 106,   //!<  6 No USBDM devices found
 PROGRAMMING_RC_ERROR_BDM_OPEN                 = 107,   //!<  7 Failed to open USBDM device
 PROGRAMMING_RC_ERROR_BDM_CONNECT              = 108,   //!<  8 Failed to connect to target
 PROGRAMMING_RC_ERROR_BDM_TARGET               = 109,   //!<  9 Failed to set target type
 PROGRAMMING_RC_ERROR_BDM_WRITE                = 110,   //!< 10 Failed to write to target
 PROGRAMMING_RC_ERROR_BDM_READ                 = 111,   //!< 11 Failed to read from target
 PROGRAMMING_RC_ERROR_BDM_RESET                = 112,   //!< 12 Failed to reset target
 PROGRAMMING_RC_ERROR_TRIM                     = 113,   //!< 13 Trimming target clock failed
 PROGRAMMING_RC_ERROR_SECURED                  = 114,   //!< 14 Target is secured and cannot be programmed
 PROGRAMMING_RC_ERROR_FAILED_FLASH_COMMAND     = 115,   //!< 15 Flash command failed
 PROGRAMMING_RC_ERROR_NO_VALID_FCDIV_VALUE     = 116,   //!< 16 Failed to find a suitable FCDIV value (clock problem?)
 PROGRAMMING_RC_ERROR_CHECKSUM                 = 117,   //!< 17 Checksum of SREC invalid
 PROGRAMMING_RC_ERROR_FAILED_CLOCK             = 118,   //!< 18 Failed setup of target clock (connection lost)
 PROGRAMMING_RC_ERROR_INTERNAL_CHECK_FAILED    = 119,   //!< 19 Failed an internal software check - should be impossible!
 PROGRAMMING_RC_ERROR_FILE_OPEN_FAIL           = 120,   //!< 20 Failed to open S1S9 file
 PROGRAMMING_RC_ERROR_PPAGE_FAIL               = 121,   //!< 21 Access to PPAGE register failed
 PROGRAMMING_RC_ERROR_EPAGE_FAIL               = 122,   //!< 22 Access to EPAGE register failed
 PROGRAMMING_RC_ERROR_SPEED_APPROX             = 123,   //!< 23 Can only approximate the target bus speed
 PROGRAMMING_RC_ERROR_CHIP_UNSUPPORTED         = 124,   //!< 24 This chip and/or operation is supported due to target hardware bug
 PROGRAMMING_RC_ERROR_TCL_SCRIPT               = 125,   //!< 25 Execution of TCL script returned a error
 PROGRAMMING_RC_ERROR_TCL_UNSECURE_SCRIPT      = 126,   //!< 26 Execution of TCL script returned a error
 PROGRAMMING_RC_ERROR_TCL_PREPROGRAM_SCRIPT    = 127,   //!< 27 Execution of TCL script returned a error
 PROGRAMMING_RC_ERROR_TCL_POSTPROGRAM_SCRIPT   = 128,   //!< 28 Execution of TCL script returned a error
 PROGRAMMING_RC_ERROR_OUTSIDE_TARGET_FLASH     = 129,   //!< 29 Image is outside target Flash memory
 PROGRAMMING_RC_ERROR_ILLEGAL_SECURITY         = 130,   //!< 30 Illegal Security value (will lock chip forever)
 PROGRAMMING_RC_FLEXNVM_CONFIGURATION_FAILED   = 131,   //!< 31 Failed to program FlexNVM Configuration values.

 // File Loader errors
 SFILE_RC_OK                                   = 0,    //!< No error
 SFILE_RC_FIRST_MESSAGE                        = 201,
 SFILE_RC_CHECKSUM                             = 201,  //!< S-record has incorrect checksum
 SFILE_RC_ILLEGAL_LINE                         = 202 , //!< S-record has invalid/unsupported record
 SFILE_RC_FILE_OPEN_FAILED                     = 203 , //!< Hex file failed to open (fopen() failed)
 SFILE_RC_ELF_FORMAT_ERROR                     = 204 , //!< ELF file does not have the expected format
 SFILE_RC_UNKNOWN_FILE_FORMAT                  = 205 , //!< File is not recognised as ELF or SREC
 SFILE_RC_ELF_WRONG_TARGET                     = 206 , //!< ELF is intended for another target
 SFILE_RC_IMAGE_OVERLAPS                       = 207 , //!< File being loaded overlaps existing contents (will still be loaded)

};

#endif /* USBDMERRORMESSAGES_H_ */
