/*! \file
    \brief Relocated User vector table
    
    The vector table on the HCS08JM processor can be relocated to just
    under the protected region of Flash memory.
    
    This file contains the relocated table.
       
   \verbatim
    JB16 ICP Code
    
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

Change History
+====================================================================
|  23 July 2008 | Started coding                                - pgo
+====================================================================
*/
#include "Common.h"
#include "ICP.h"
#include "Commands.h" 
#include "bdmCommon.h"
#include "usb.h"

static interrupt void dummyISR(void) {

   asm {
   
   }
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
#define JMPOPCODE (0xCC)
const UserVector userVectorTable[17] @USER_VECTORTABLE_LOCATION = {
   { JMPOPCODE, (void(*)())userDetectICP  }, // User Level ICP detection
   { JMPOPCODE, bdm_resetSense            }, // Keyboard
   { JMPOPCODE, dummyISR                  }, // SCI Transmit
   { JMPOPCODE, dummyISR                  }, // SCI Receive
   { JMPOPCODE, dummyISR                  }, // SCI Error
   { JMPOPCODE, dummyISR                  }, // TIM2 Overflow
   { JMPOPCODE, bdm_targetVddSense        }, // TIM2 Ch0 & Ch1
   { JMPOPCODE, bdm_targetVddSense        }, // TIM2 Ch1
   { JMPOPCODE, bdm_targetVddSense        }, // TIM2 Ch0
   { JMPOPCODE, dummyISR                  }, // TIM1 Overflow
   { JMPOPCODE, bdm_targetVddSense        }, // TIM1 Ch0 & Ch1
   { JMPOPCODE, bdm_targetVddSense        }, // TIM1 Ch1
   { JMPOPCODE, bdm_targetVddSense        }, // TIM1 Ch0
   { JMPOPCODE, dummyISR                  }, // IRQ
   { JMPOPCODE, usb_isr                   }, // USB
   { JMPOPCODE, dummyISR                  }, // SWI
   { JMPOPCODE, _Startup                  }  // Reset - don't change
};
