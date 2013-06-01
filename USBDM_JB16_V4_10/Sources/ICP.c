/*! \file 
 *  \brief ICP Boot Code
 *
 *  The following code is placed in a region of flash that is not
 *  erased by default.  It detects the need for ICP and also 
 *  redirects the vector table to the user vector table. \n
 *  Loosely Based on Freescale Application Note: AN2399SW
 *
 *  \warning
 *  It is very important that this code does not use any library
 *  routines as they will end up in the USER ROM area which may be replaced. \n
 *  This code must be stand-alone.
 *
    \verbatim
    JB16 ICP Code
    
    Copyright (C) 2008  Peter O'Donoghue


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
    \endverbatim

Change History
+====================================================================
|   4 Oct  2012 | Updated ICP detection                         - pgo
|  26 Oct  2008 | V1.2                                          - pgo
|  26 Oct  2008 | Removed reference to external library!!       - pgo
|  ?? ???  ???? | Started coding                                - pgo
+====================================================================
*/

#include <hidef.h>      // Various Macros
#include "Derivative.h" // Peripheral declarations
#include "Common.h"
#include "Configure.h"
#include "ICP.h"


#define USER_FLASH_START (0xBA00) //!< Start of User Area in Flash memory
#define USER_FLASH_END   (0xF7FF) //! <End of  User Area in Flash memory
#define FLBPR_VALUE      ((USER_FLASH_END+1)>>8) //!< ICP Area is protected

/*
** All this code goes in ICP boot area
*/
#pragma CODE_SEG  BOOT_ROM
#pragma CONST_SEG BOOT_CONST
#pragma DATA_SEG  __SHORT_SEG BOOT_RAM

//! Magic number to indicate ICP reboot
#define MAGIC_REBOOT_KEY1 (0xAA55UL)
#define MAGIC_REBOOT_KEY2 (0x0FF0UL)

/*! Checked during illegal instruction reset.
**   If set to MAGIC_REBOOT_KEY then ICP boot will occur
*/
static U16 icpKeyBytes1;
static U16 icpKeyBytes2;


#define MON_USB_ICP  (0xFA19) //!< Enter USB ICP

/*! Causes reboot to ICP mode
** 
**  Called from user code
**  Doesn't return!
*/ 
static void rebootToICP(void) {
U16 delayCount;

   icpKeyBytes1 = MAGIC_REBOOT_KEY1;
   icpKeyBytes2 = MAGIC_REBOOT_KEY2;
   UADDR = 0x00;  // Disable USB
   
   //  Take PTE.4 low to similate a USB disconnect
   //  - A bit of a hack but necessary if there is an
   //    external 1k5 PUP on USB-
   DDRE  |=  (1<<4);
   PTE   &= ~(1<<4);
   
   for (delayCount=0; delayCount++<100;)   
      asm {
         nop
      }
      
   reset();  // Force Reset
}

/*!  Detects ICP mode
**
** ICP mode is indicted by a PTA.0 being low during reset
**   OR
** Software initiated ICP by illegal instruction reset + correct magic number
**
** This is failsafe ICP option during development.  
** PTA.0 is not accessible in final version (shrink wrapped).
*/
static char detectICP(void) {
U8 doICP;

   // Forced Reboot to ICP 
   // Indicated by Illegal instruction reset + correct Magic Number
   if (RSR_ILOP && (icpKeyBytes1 == MAGIC_REBOOT_KEY1) &&
                   (icpKeyBytes2 == MAGIC_REBOOT_KEY2)) {
      icpKeyBytes1 = 0;  // No more ICP reboots
      icpKeyBytes2 = 0;
      return 1;
   }
   icpKeyBytes1 = 0;  // No more ICP reboots
   icpKeyBytes2 = 0;
   
#ifdef ENABLE_ICP_PIN
   // Enable pull-up
   ENABLE_ICP_PIN();
#endif
   asm {
      nop
      nop
   }
   doICP  = TEST_ICP_PIN(); // Check pin
   
#ifdef DISABLE_ICP_PIN
   // Restore reset state
   DISABLE_ICP_PIN(); 
#endif
   
   return doICP;
}

/*! Check if user region of flash is invalid
**
** First checks for invalid reset vector then does a simple checksum over the user flash.
** See PRM file for example of automatic calculation of checksum
*/
#pragma MESSAGE DISABLE C1404 // Disable warnings about return expected
static U8 flashInvalid(){
   asm{
      lda      #1                       // Set up for failed test
      
      // Check for valid user reset vector
      ldx      *(@userVectorTable+49)   // load the user code reset address
      pshx
      pulh
      ldx      *(@userVectorTable+50)
      cphx     #USER_FLASH_START        // exit if not pointing within flash
      blo      ex
      cphx     #USER_FLASH_END
      bhi      ex

      // Do checksum over user flash memory
      ldhx     #USER_FLASH_START
      clra
      
      // Sum the flash memory
   loop:
      sta      _COPCTL // Feed the watchdog
      add      ,x
      aix      #1
      cphx     #USER_FLASH_END
      blo      loop
      
      sub      ,x   // Subtract the checksum
                    // 0 => OK
   ex:
   }
}
#pragma MESSAGE DEFAULT C1404 // Restore warnings about return expected

#pragma NO_RETURN
#pragma MESSAGE DISABLE C1404 // Disable warnings about return expected
//!  Calls user ICP detection routine via fixed location vector
//!
static char _userDetectICP(void){
   asm{
      sta _COPCTL         // Feed the watchdog
      jmp userVectorTable // First entry is userDetectICP()
   }
}
#pragma MESSAGE DEFAULT C1404 // Restore warnings about return expected

#pragma NO_RETURN
/*! Checks if ICP is required
**   If so, ICP code in ROM is called
**   otherwise user reset code is called via user vector table
**
** @return Never exits
*/
static void myStartup(void) {

   if (detectICP() || flashInvalid() || _userDetectICP()) {
      CONFIG = CONFIG_URSTD_MASK | CONFIG_COPD_MASK;
//      DDRD |= 0x01;        // Turn on LED
//      PTD  &= ~0x01;
//      PTD  |= 0x01;      // Turn off LED
      asm{
         jmp MON_USB_ICP   // Enter ICP ROM
      };                             
   }

   FLBPR=FLBPR_VALUE; // Protect vectors and boot loader

   // Jmp to user code via user reset vector
   asm{
      sta   _COPCTL             // Feed the watchdog
      jmp   userVectorTable:48  // jmp via reset vector
   }
}

//!  Type for vector table entry
typedef void (* const vector)(void);
#define VECTOR(x) (vector)(x)

#pragma MESSAGE DISABLE C1805 // Disable warnings about non-standard conversion
//! Hardware vector table - mostly redirects to user vector (jump) table 
//!!
const vector vectorTable[17]@0xFFE0-2 = {
   // Fixed vector used to provide access to routine in user flash area 
   VECTOR(rebootToICP),       // Force ICP reboot
   
   // Vectors apart from reset pass to user code via user vector table
   VECTOR(userVectorTable+1),      // Keyboard
   VECTOR(userVectorTable+2),      // SCI Transmit
   VECTOR(userVectorTable+3),      // SCI Receive
   VECTOR(userVectorTable+4),      // SCI Error
   VECTOR(userVectorTable+5),      // TIM2 Overflow
   VECTOR(userVectorTable+6),      // TIM2 Ch0 & Ch1
   VECTOR(userVectorTable+7),      // TIM2 Ch1
   VECTOR(userVectorTable+8),      // TIM2 Ch0
   VECTOR(userVectorTable+9),      // TIM1 Overflow
   VECTOR(userVectorTable+10),     // TIM1 Ch0 & Ch1
   VECTOR(userVectorTable+11),     // TIM1 Ch1
   VECTOR(userVectorTable+12),     // TIM1 Ch0
   VECTOR(userVectorTable+13),     // IRQ
   VECTOR(userVectorTable+14),     // USB
   VECTOR(userVectorTable+15),     // SWI
   
   // Reset is handled by ICP boot code
   myStartup                       // Reset
};
#pragma CONST_SEG DEFAULT
#pragma MESSAGE DEFAULT C1805 // Restore warnings about non-standard conversion

//! ICP version (2 hex digits, major.minor)
#define ICP_VERSION_SW (1<<4|2) // 1.2
 
const U8 ICP_Version_SW @ ICP_VERSION_SW_LOCATION = ICP_VERSION_SW; 
const U8 ICP_Version_HW @ ICP_VERSION_HW_LOCATION = VERSION_HW;     
