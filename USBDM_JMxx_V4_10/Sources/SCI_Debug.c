/*! \file
    \brief Simple SCI code JM60
    
   \verbatim
   JMxx SCI Code
    
   Copyright (C) 2010  Peter O'Donoghue

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
+============================================================================================
| 29 Sep 2010 | Created                                                           V4.2 - pgo 
+============================================================================================
   \endverbatim
*/

#include <termio.h>
#include <string.h>
#include "Common.h"
#include "Configure.h"
#include "SCI_Debug.h"

#if ((DEBUG&SCI_DEBUG) != 0)

// This defaults to SCI on JS16 or SCI2 on JMxx
//
#ifndef SCID // Don't redefine if there is only a single SCI
#ifdef USE_SCI1
// SCI2
#define SCID              SCI1D

#define SCIS1             SCI1S1
#define SCIS1_RDRF        SCI1S1_RDRF
#define SCIS1_RDRF_MASK   SCI1S1_RDRF_MASK
#define SCIS1_TDRE        SCI1S1_TDRE
#define SCIS1_TDRE_MASK   SCI1S1_TDRE_MASK

#define SCIBD             SCI1BD

#define SCIC1             SCI1C1
#define SCIC1_LOOPS_MASK  SCI1C1_LOOPS_MASK
#define SCIC1_PE_MASK     SCI1C1_PE_MASK
#define SCIC1_PT_MASK     SCI1C1_PT_MASK
#define SCIC1_M_MASK      SCI1C1_M_MASK

#define SCIC2             SCI1C2
#define SCIC2_TE_MASK     SCI1C2_TE_MASK
#define SCIC2_RE_MASK     SCI1C2_RE_MASK
#define SCIC2_TIE         SCI1C2_TIE
#define SCIC2_TIE_MASK    SCI1C2_TIE_MASK
#define SCIC2_RIE_MASK    SCI1C2_RIE_MASK
#define SCIC2_SBK         SCI1C2_SBK

#define SCIC3             SCI1C3
#define SCIC3_T8_MASK     SCI1C3_T8_MASK
#define SCIC3_FEIE_MASK   SCI1C3_FEIE_MASK
#define SCIC3_NEIE_MASK   SCI1C3_NEIE_MASK
#define SCIC3_ORIE_MASK   SCI1C3_ORIE_MASK
#define SCIC3_PEIE_MASK   SCI1C3_PEIE_MASK
#else
// SCI2
#define SCID              SCI2D

#define SCIS1             SCI2S1
#define SCIS1_RDRF        SCI2S1_RDRF
#define SCIS1_RDRF_MASK   SCI2S1_RDRF_MASK
#define SCIS1_TDRE        SCI2S1_TDRE
#define SCIS1_TDRE_MASK   SCI2S1_TDRE_MASK

#define SCIBD             SCI2BD

#define SCIC1             SCI2C1
#define SCIC1_LOOPS_MASK  SCI2C1_LOOPS_MASK
#define SCIC1_PE_MASK     SCI2C1_PE_MASK
#define SCIC1_PT_MASK     SCI2C1_PT_MASK
#define SCIC1_M_MASK      SCI2C1_M_MASK

#define SCIC2             SCI2C2
#define SCIC2_TE_MASK     SCI2C2_TE_MASK
#define SCIC2_RE_MASK     SCI2C2_RE_MASK
#define SCIC2_TIE         SCI2C2_TIE
#define SCIC2_TIE_MASK    SCI2C2_TIE_MASK
#define SCIC2_RIE_MASK    SCI2C2_RIE_MASK
#define SCIC2_SBK         SCI2C2_SBK

#define SCIC3             SCI2C3
#define SCIC3_T8_MASK     SCI2C3_T8_MASK
#define SCIC3_FEIE_MASK   SCI2C3_FEIE_MASK
#define SCIC3_NEIE_MASK   SCI2C3_NEIE_MASK
#define SCIC3_ORIE_MASK   SCI2C3_ORIE_MASK
#define SCIC3_PEIE_MASK   SCI2C3_PEIE_MASK
#endif
#endif

#define BAUDDIVIDER(x)  ((BUS_FREQ/16)/(x))

//! debugSCITx() -  Send a ch to the SCI
//!
//! @entry ch - char to Tx
//!
void debugTx(char ch) {
   while (SCIS1_TDRE==0) {
   }
   SCID = ch;      // SCI Idle - write directly to SCI
}

//! debugSCIRx() -  Gets a character from the SCI
//!
//! @return 
//!  -  -ve => no ch ready \n
//!  -  +ve => char from queue
//!
int debugRx(void) {
   if (SCIS1_RDRF==0) {
	   return -1;
   }
   return SCID;
}

//! debugPuts() -  Send a string to the SCI
//!
//! @entry s - String to send
//!
void debugPuts(const char * s) {
   while (*s != '\0') {
	   debugTx(*s++);
   }
}
void debugSCIInit(void) {
	
   SCIBD = (U16)BAUDDIVIDER(115200)&0x3FFFU;
   SCIC1 = 0x00;
   SCIC2 = SCIC2_RE_MASK|SCIC2_TE_MASK; // Enable Rx/Tx
   SCIC3 = SCIC3_T8_MASK; // Extra stop bit
}
#endif
