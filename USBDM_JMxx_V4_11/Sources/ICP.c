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
    JM60 ICP Code
    
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

\verbatim

Change History
+======================================================================
|  31 July 2010 | Added SOPT1_BKGDPE_MASK to SOPT1 for JS16       - pgo
|  22 May  2009 | Extended ICP structure with Flash start address - pgo
|  22 May  2009 | Changed ROM size to 32 K (req. JM32 at least)   - pgo
|   4 Feb  2009 | Made GET_VERSION more compatible  (v1.4)        - pgo
|  24 Oct  2008 | Added ICP structure & changes (still compat.)   - pgo
|  23 July 2008 | Started coding                                  - pgo
+======================================================================

\endverbatim
*/

#include <hidef.h>      // Various Macros
#include "Common.h"
#include "Configure.h"
#include "ICP.h"
#include "ICP_USB.h"
//#include "SCI_Debug.h"

#pragma MESSAGE DISABLE C4301 // Disable warnings about Inline functions
/*
** All this code goes in ICP boot area
*/
#pragma CODE_SEG  BOOT_ROM
#pragma CONST_SEG BOOT_CONST
#pragma DATA_SEG  __SHORT_SEG BOOT_RAM

extern char far __SEG_START_BOOT_ROM[];  // located above user flash

//! Magic number to indicate ICP reboot
#define MAGIC_REBOOT_KEY (0xAA55U)

//! Checked during reboot.
//! If set to MAGIC_REBOOT_KEY then ICP boot will occur
uint16_t icpKeyBytes;

/*! Flash memory protection boundary
 * 
 *  NVPROT is copied from Flash to Register on reset
 */
static const uint8_t NVPROT_INIT @0x0000FFBD = ((FLASH_PROTECT_START-1)>>8)&0xFE;

/*! Security options and Vector redirection
 * 
 *  These settings :\n
 *    - Disable security
 *    - Relocate vector table to top of unprotected Flash memory
 *
 *  NVOPT is copied from Flash to Register on reset
 */
static const uint8_t NVOPT_INIT  @0x0000FFBF = NVOPT_KEYEN_MASK|NVOPT_SEC01_MASK; 

/*! Bootloader Checksum Bypass
 * 
 *  This settings :\n
 *    - Disables Checksum (to be compatible with JM16)
 *
 */
static const uint8_t CHECKSUM_BYPASS  @0x0000FFBA = 0x00; 

#pragma CODE_SEG  BOOT_ROM
//! Causes reboot to ICP mode
//!
//! Called from user code.
//! Doesn't return!
//!
#pragma NO_RETURN
static void rebootToICP(void) {

   icpKeyBytes = MAGIC_REBOOT_KEY;
   (void)icpReset();
}

//!  Detects ICP mode
//!
//!  Booting to ICP mode is indicated in two ways:
//!  - ICP_PIN being held low during power-on.  This is failsafe ICP option during development.
//!    ICP_PIN is may not be accessible in final version [e.g.shrink wrapped].
//!  - Software initiated ICP by illegal instruction reset + correct magic number.
//!
static char detectICP(void) {
uint8_t doICP;

   // Check for software triggered Reboot to ICP 
   // Indicated by Illegal instruction reset + correct Magic Number
   //
   if (icpKeyBytes == MAGIC_REBOOT_KEY) {
      icpKeyBytes = 0;    // No more ICP reboots
	  if (SRS_ILOP) 
         return 1;
   }
   
#if (CPU==JMxx)
   // Hardware triggered ICP (pin pulled low)
   //   
#ifdef ICP_USES_IRQ
   IRQSC = IRQSC_IRQPE_MASK;
   // IRQ pin LOW is used for ICP
   asm {
	   clra
	   dbnza *-0
	   bih	noICP
	   inca
   noICP:
	   sta  doICP   
   }
   IRQSC = 0;
#else
   ICP_PIN_PER = 1;    // Enable pull-up
   ICP_PIN_DDR = 0;    // Make input
   asm {
      nop
      nop
   }
   doICP     = !ICP_PIN;    // Do ICP if tied low
   
//   // Restore reset state
//   ICP_PIN_PER = 0;         // Disable pull-up
#endif
#else //if !((CPU==JM32) || (CPU==JM60))
   doICP = FALSE;
#endif
   
   return doICP;
}

//! Check if user region of flash is invalid
//! 
//! First checks for invalid reset vector then does a simple checksum over the user flash. \n
//! See PRM file for example of automatic calculation of checksum needed.
//!
#pragma MESSAGE DISABLE C1404 // Disable warnings about return expected
static uint8_t flashInvalid(void) {
   asm {
      lda      #1                     // Set up for failed test
      
      // Check for valid user reset vector
      ldhx     userVectorTable:0      // Reset vector in user vector table

      cphx     ICP_data.flashStart    // Exit if not pointing within flash
      blo      ex
      cphx     #@__SEG_START_BOOT_ROM // This prevents erased Flash looking valid!
      bhs      ex

      // Do checksum over user flash memory
      ldhx     ICP_data.flashStart 
      clra
      
      // Sum the flash memory
   loop:
      add      ,x
      aix      #1
      cphx     #@__SEG_START_BOOT_ROM-1
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
static uint8_t _userDetectICP(void){

   // Jmp to userDetectICP() via user ICP data structure
   asm {
      ldhx ICP_data.userDetectICP
      jmp  0,x
   }
}
#pragma MESSAGE DEFAULT C1404 // Restore warnings about return expected

//#pragma NO_ENTRY 
//#pragma NO_EXIT 
//#pragma NO_FRAME 
//#pragma NO_RETURN
//! Checks if ICP is required.
//!
//! If so, ICP code in ROM is called.  Otherwise user reset code is called via user vector table. \n
//! Most of the action is driven from the USB polling function.  
//!
//!  @return Never exits
//!
void myStartup(void) {
#if !defined(SOPT1_BKGDPE_MASK) || defined(DISABLE_BKGD)
#undef SOPT1_BKGDPE_MASK
#define SOPT1_BKGDPE_MASK (0) // Only exists on some CPUs or BKGD pin in use as GPIO
#endif
   SOPT1 = SOPT1_STOPE_MASK|SOPT1_BKGDPE_MASK; // Disable COP, enable STOP instr. & BKGD pin
   if (detectICP() || flashInvalid() || _userDetectICP()) {
	   asm {
          ldhx  #$1FF  // Need to set realistic stack for minimal ICP USB
          txs
	   }
	  LED_INIT();
//	  debugSCIInit();
//	  debugPuts("ICP\r");
      startICP_USB();     // Poll USB events forever
   }
   // Jmp to user code via user reset vector
   userVectorTable[0]();
}

//! Vector for rebootToICP() routine
static const vector forceICPVector @ICP_FORCE_LOCATION = (vector)rebootToICP;

//! Reset Vector
//! All other vectors are re-mapped
static const vector resetVector @0xFFFE = myStartup;

#pragma CONST_SEG DEFAULT

const uint8_t ICP_Version_SW @ ICP_VERSION_SW_LOCATION = ICP_VERSION_SW; 
const uint8_t ICP_Version_HW @ ICP_VERSION_HW_LOCATION = VERSION_HW;     
