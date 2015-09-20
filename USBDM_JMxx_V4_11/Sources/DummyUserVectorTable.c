/*! \file
    \brief Relocated User vector table
    
    The vector table on the HCS08JM processor can be relocated to just
    under the protected region of Flash memory.
    
    This file contains the relocated table.
       
    \verbatim
    JMxx ICP Code
    
    Copyright [C] 2008  Peter O'Donoghue

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    [at your option] any later version.

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
+=======================================================================
+=======================================================================

\endverbatim
*/
#include "Common.h"
#include "Configure.h"
#include "ICP.h"

/*! Location of user vector table in User Flash
**
**  See UserVectors.c
**
*/
#define USER_VECTORTABLE_LOCATION  (FLASH_PROTECT_START-sizeof(userVectorTable)-2)

/*! The following structure may be found through \ref versionOffset so
**  it may be accessed from ICP.
*/
#define ICP_DATA_LOCATION (USER_VECTORTABLE_LOCATION-sizeof(ICP_dataType))

/* The following structure is at a fixed location in the ROM
**  so it may be accessed from ICP.
*/
/*! Various ICP data
**
*/
extern uint8_t far __SEG_START__PRESTART[];  // located at bottom of Flash

//! Structure to describe ICP data
const ICP_dataType ICP_data @ICP_DATA_LOCATION =
{  SERIAL_NO, '\0',           //! Serial # + guaranteed string terminator 
   __SEG_START__PRESTART,     //!< Start of User flash area
   VERSION_HW,                //!< Hardware version
   VERSION_SW,                //!< Software version
   userDetectICP,             //!< Check if User code wants ICP on boot
};

/*! \brief Detect In-Circuit Programming (ICP) mode.
 *
 * This routine is called DIRECTLY from the ICP boot code - minimal C setup (stack only)!
 *
 * @return 0 => ICP not required, \n
 *         1 => ICP required
 */
uint8_t userDetectICP(void) {
   return 0;  // ICP not required (rely on bootstrap in ICP code)
}

/*! Dummy ISR for unexpected interrupts
**
**  It is good practice to set unused vectors to a dummy routine.
*/
static interrupt void dummyISR(void) {
   asm {
   loop:
      bgnd
      bra   loop
   }
}

void main() {
    // Re-enter bootloader
	forceICPReset();
}

/*
 * External interrupt routines
 */
extern void _Startup(void); // Low-level C startup entry point

/*! User vector table
**
**  The vector table is relocated to the top of User Flash
**
*/
const vector userVectorTable[30] @USER_VECTORTABLE_LOCATION = {
   _Startup,               // 30. pseudo-Reset - Real reset vector is not relocated.
   dummyISR,               // 29. RTC
   dummyISR,               // 28. I2C
   dummyISR,               // 27. ACMP
   dummyISR,               // 26. ADC
   dummyISR,               // 25. KBI
   dummyISR,               // 24. SCI2 Tx
   dummyISR,               // 23. SCI2 Rx
   dummyISR,               // 22. SCI2 Error
   dummyISR,               // 24. SCI1 Tx
   dummyISR,               // 23. SCI1 Rx
   dummyISR,               // 19. SCI1 Error
   dummyISR,               // 18. TPM2 Overflow
   dummyISR,               // 17. TPM2 Ch 1
   dummyISR,               // 16. TPM2 Ch 0
   dummyISR,               // 15. TPM1 Overflow
   dummyISR,               // 10. TPM1 Ch 5 // One of these channels is used!
   dummyISR,               // 10. TPM1 Ch 4
   dummyISR,               // 10. TPM1 Ch 3
   dummyISR,               // 10. TPM1 Ch 2
   dummyISR,               // 10. TPM1 Ch 1
   dummyISR,               //  9. TPM1 Ch 0
   dummyISR,               //  8. Reserved
   dummyISR,               //  7. USB Status
   dummyISR,               //  6. SPI2
   dummyISR,               //  5. SPI1
   dummyISR,               //  4. MCG loss of lock
   dummyISR,               //  3. Low voltage detect
   dummyISR,               //  2. IRQ
   dummyISR,               //  1. SWI
                           //  0. Used for offset to Version# & Flash checksum
                           //     Real reset vector is not relocated.
};

// This offset allows the ICP information structure to be located relative to the
// end of the User flash area.
// Points to END of ICP structure [in case it needs to grow in future versions]
const uint8_t versionOffset@(FLASH_PROTECT_START-2) = sizeof(userVectorTable)+2;

