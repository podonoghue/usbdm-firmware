/*! \file
    \brief USBDM - JTAG Sequence Execution
    
   \verbatim
   USBDM
   Copyright (C) 2007  Peter O'Donoghue

   Turbo BDM Light
   Copyright (C) 2005  Daniel Malik

   Turbo BDM Light ColdFire
   Copyright (C) 2005  Daniel Malik

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
   +=======================================================================================
   | 15 May 2012 | Added JTAG_READ_MEM, JTAG_WRITE_MEM for DSC                 V4.9   - pgo
   | 28 Mar 2011 | Added JTAG routines for ARM                                 V4.6   - pgo
   | 28 Mar 2011 | Added JTAG_SET_PADDING                                      V4.6   - pgo
   |  1 Dec 2010 | Added JTAG_SET_BUSY & deleted fixed busy actions            V4.3   - pgo
   | 21 Jun 2010 | Added T_MC56F800xx Type                                     V3.5   - pgo
   |  2 Jun 2010 | Created                                                     V3.4   - pgo
   +=======================================================================================
   \endverbatim
*/
#include <stdio.h>
#include "Configure.h"
#include "Common.h"
#include "Commands.h"
#include "BDM.h"
#include "CmdProcessing.h"
#include "JTAGSequence.h"
#include "Commands.h"
#include "Configure.h"
#include "BDM_CF.h"
#include "ARM.h"

static U8 getValueByte(U8 value);

#define STOP_ON_END         (0<<0)
#define STOP_ON_ELSE        (1<<1)
#define STOP_ON_ELSE_IF     (1<<2)
#define STOP_ON_END_IF      (1<<3)
#define STOP_ON_END_REPEAT  (1<<4)
#define STOP_ON_SUB         (1<<5)

#if (HW_CAPABILITY&CAP_JTAG_HW)
//! Search for delimiters
//!
//! @param sentinel - Mask indicating values to accept
//!
//! @return Ptr to found final search position (sentinel if found)
//!
//! @note Searches are qualified by loop depth.
//! @note Assumes correct nesting of loops and conditionals
//! @note Stops unconditionally on JTAG_SUBx or JTAG_END
//!
static const U8 *skipSequence(const U8 *sequence, U8 sentinel) {
   U8 temp;
   U8 loopDepth = 0;

//   print("skipSequence() \n");
   for(;;) {
      U8 opcode   = *sequence;
      U8 numBits  = (opcode&JTAG_NUM_BITS_MASK);  // In case needed
      if (numBits == 0)
         numBits = 32;

      switch (opcode&JTAG_COMMAND_MASK) {
         case JTAG_MISC0: // Misc commands
         case JTAG_MISC1:
         case JTAG_MISC2:
            switch (opcode) {
               // Unconditionally stop search on any of the following
               case JTAG_END:
                  return sequence;

               case JTAG_SUBA:
               case JTAG_SUBB:
               case JTAG_SUBC:
               case JTAG_SUBD:
                  if (sentinel&STOP_ON_SUB)
                     return sequence;
                  break;

               case JTAG_END_SUB:
                  if (sentinel&STOP_ON_SUB)
                     return ++sequence;

               // 8-bit in line parameter
               case JTAG_SHIFT_IN_DP:
               case JTAG_SHIFT_OUT_DP:
               case JTAG_SHIFT_IN_OUT_DP:
//               case JTAG_SHIFT_IN_VARA:
//               case JTAG_SHIFT_IN_VARB:
               case JTAG_SHIFT_OUT_VARA:
               case JTAG_SHIFT_OUT_VARB:
               case JTAG_SET_ERROR:
               case JTAG_PUSH8:
               case JTAG_REPEAT8:
                  sequence++; // Skip inline value
                  break;

               // 16-bit in line parameter
               case JTAG_PUSH16:
                  sequence+=2; // Skip inline value
                  break;

               // 32-bit in line parameter
               case JTAG_PUSH32:
                  sequence+=4; // Skip inline value
                  break;

               // Increase nesting level - no params
               case JTAG_IF_ITER_EQ:
               case JTAG_IF_ITER_NEQ:
               case JTAG_IF_VARA_EQ:
               case JTAG_IF_VARB_EQ:
               case JTAG_IF_VARA_NEQ:
               case JTAG_IF_VARB_NEQ:
               case JTAG_REPEAT:
                  loopDepth++;
                  break;

               // Reduce nesting level - no params
               case JTAG_END_IF:
                  if (loopDepth>0)
                     loopDepth--;
                  else {
                     if (sentinel&STOP_ON_END_IF)
                        return sequence;
                  }
                  break;
               case JTAG_END_REPEAT:
                  if (loopDepth>0)
                     loopDepth--;
                  else {
                     if (sentinel&STOP_ON_END_REPEAT)
                        return sequence;
                  }
                  break;

               // No parameters
               case JTAG_ELSE:
                  if (sentinel&STOP_ON_ELSE)
                     return ++sequence;

               // No parameters
               case JTAG_SET_BUSY:
               case JTAG_SAVE_SUB:
               case JTAG_TEST_LOGIC_RESET:
               case JTAG_MOVE_DR_SCAN:
               case JTAG_MOVE_IR_SCAN:
               case JTAG_SET_STAY_SHIFT:
               case JTAG_SET_EXIT_SHIFT_DR:
               case JTAG_SET_EXIT_SHIFT_IR:
               case JTAG_SET_EXIT_IDLE:
               case JTAG_SET_IN_FILL_0:
               case JTAG_SET_IN_FILL_1:
               case JTAG_CALL_SUBA:
               case JTAG_CALL_SUBB:
               case JTAG_CALL_SUBC:
               case JTAG_CALL_SUBD:
               case JTAG_NOP:
               case JTAG_RETURN:
               case JTAG_CONTINUE:
               case JTAG_BREAK:
               case JTAG_SAVE_DP_VARC:
               case JTAG_SAVE_DP_VARD:
               case JTAG_RESTORE_DP_VARC:
               case JTAG_RESTORE_DP_VARD:
               case JTAG_PUSH_DP_8:
               case JTAG_PUSH_DP_16:
               case JTAG_PUSH_DP_32:
               case JTAG_LOAD_VARA:
               case JTAG_LOAD_VARB:
               case JTAG_DEBUG_ON:
               case JTAG_DEBUG_OFF:
               case JTAG_SHIFT_OUT_DP_VARA:
               case JTAG_SKIP_DP:
               case JTAG_READ_MEM:
               case JTAG_WRITE_MEM:
               default:
                  break;
               case JTAG_SHIFT_IN_OUT_VARA:
               case JTAG_SHIFT_IN_OUT_VARB:
                  temp = (U8)BITS_TO_BYTES(*++sequence);
                  sequence += temp; // Skip over inline data
                  break;
                  
               case JTAG_ARM_READAP:
            	   sequence += 3; // numWords, addr-16
                   break;
               case JTAG_ARM_WRITEAP:
            	   sequence += 3; // numWords, addr-16
                   break;
               case JTAG_ARM_WRITEAP_I:
            	   sequence += 6; // addr-16, data-32
               break;
               case JTAG_SET_PADDING:  // #4x16-bits - sets HDR HIR TDR TIR
            	   sequence += 8; // Skip numBits
            	   break;
            }
            break;
         case JTAG_SHIFT_OUT_Q(0) :
         case JTAG_SHIFT_IN_OUT_Q(0) :
            sequence += BITS_TO_BYTES(numBits); // Skip over inline data
            break;
         case JTAG_REPEAT_Q(0):
            loopDepth++;
            break;
         case JTAG_SHIFT_IN_Q(0) :
         case JTAG_PUSH_Q(0):
         default:
            break;
      }
      sequence++;
   };
//   return sequence;
}

//! Maximum size subroutine that can be cached
#define MAX_CACHE (42)

#pragma DATA_SEG __SHORT_SEG Z_PAGE
static const U8 *sequence;    // JTAG sequence to execute
static const U8 *dataOutPtr;  // Data to send to device

#pragma DATA_SEG DEFAULT
static const U8 *subPtrs[]  = {NULL, NULL, NULL, NULL};
static       U8 subroutineCache[MAX_CACHE];

typedef struct {
   const U8* startOfLoop;
   U16       iterator;
} RepeatInformation;

typedef struct {
   const U8           *returnAddress;
   RepeatInformation  *repeatTOS;
} SubroutineInformation;


U8 initJTAGSequence(void) {

   (void)memset(subPtrs, (int)NULL, sizeof(subPtrs));
   return BDM_RC_OK;
}

U8                   complete          = false;
U8                   inFill            = JTAG_WRITE_1;
U8                   exitAction        = JTAG_EXIT_IDLE;
U8                   rc                = BDM_RC_OK;
U32                  variables[4]      = {0,0,0,0};
U16                  iterator          = 0;
const U8             *startOfIteration = NULL;
U32                  tempValue = 0;
U8                   *dataInPtr;
const U8             *dscInstructionPtr;

RepeatInformation       repeatStack[6] = {{NULL,0}};
RepeatInformation       *repeatTOS = repeatStack;

SubroutineInformation   subroutineStack[4] = {{NULL, NULL}};
SubroutineInformation   *subroutineTOS = subroutineStack;
U8 opcode;
U8 numBits;
U8 regNo;
int adjustment;

#define INLINE_TARGET_INSTRUCTION_EXECUTION

#ifdef INLINE_TARGET_INSTRUCTION_EXECUTION
// OK - I give up.  To get acceptable performance 
// I have target specific code to do target
// code execution.
// It replaces interpreted SUBA with compiled code.

// Common JTAG Commands
#define JTAG_IDCODE_COMMAND         (0x02)
#define JTAG_BYPASS_COMMAND         (~0x00)

// Commands to Master JTAG
#define JTAG_MASTER_COMMAND_LENGTH  (8)
#define JTAG_TLM_SELECT_COMMAND     (0x05)

#define TLM_REGISTER_LENGTH         (4)
#define TLM_MASTER_SELECT_MASK      (0x01)
#define TLM_SLAVE_SELECT_MASK       (0x02)

// Command to Core JTAG
#define JTAG_CORE_COMMAND_LENGTH    (4)
#define CORE_ENABLE_ONCE_COMMAND    (0x06)
#define CORE_DEBUG_REQUEST_COMMAND  (0x07)

// EONCE Command register details
//-------------------------------------------------------------------
#define ONCE_CMD_LENGTH  (8)

// The following bit masks may be combined
#define ONCE_CMD_READ   (1<<7)
#define ONCE_CMD_WRITE  (0<<7)
#define ONCE_CMD_GO     (1<<6)
#define ONCE_CMD_EXIT   (1<<5)

// Register field - some commonly used regs here
#define OPDBR_ADDRESS   (0x04)
#define OTX_ADDRESS     (0x07)   // tx to target OTX/ORX register
#define OTX1_ADDRESS    (0x09)
#define ORX_ADDRESS     (0x0B)   // rx from target OTX/ORX register
#define ORX1_ADDRESS    (0x0D)
#define ONCE_CMD_NOREG  (0x1F)   // used for no register

#define TARGET_STATUS_EXECUTE   (0x01)
#define TARGET_STATUS_STOP      (0x05)
#define TARGET_STATUS_EX_ACCESS (0x09)
#define TARGET_STATUS_DEBUG     (0x0D)

//volatile static const U8 dummy[1024] = {0};

#define MAX_DSC_RETRY (100)
//! This routine is the hard-coding of the most time-critical JTAG subroutine
//! for writing to MC56F800x registers
//! It called as the JTAG_SUBA routine when T_MC56F80xx target is active
//!
//! @param dataOutPtr ptr to current position in JTAG instruction sequence
//!
//! @return error code
//!
//! Parameters taken from DP
//!   2, // # of instructions
//!      // Length  Instruction data...
//!           2,    0x87,0x44, 0xAA,0x34,
//!           3,    0xE7,0x7F, 0xD4,0x7C, 0xFF,0xFF,
U8 executeDSCTargetInstructionSequence(const U8 **dataOutPtr) {
static const U8 enableONCECommand[] = {CORE_ENABLE_ONCE_COMMAND};
static const U8 writeOpcode[]       = {OPDBR_ADDRESS|ONCE_CMD_WRITE};
static const U8 writeOpcodeAndGo[]  = {OPDBR_ADDRESS|ONCE_CMD_WRITE|ONCE_CMD_GO};
U8 onceStatus;
U8 numberOfInstructions;
U8 retry;
const U8 *dataOutPtrX = *dataOutPtr;

   USBDM_JTAG_SelectShift(JTAG_SHIFT_IR);
   numberOfInstructions = *dataOutPtrX++;
//   numberOfInstructions = dummy[12];
   do {
      U8 numWords = *dataOutPtrX++;
      USBDM_JTAG_Write(JTAG_CORE_COMMAND_LENGTH, JTAG_EXIT_SHIFT_DR, enableONCECommand);
      do {
         if (numWords>1) {
            USBDM_JTAG_Write(ONCE_CMD_LENGTH, JTAG_EXIT_SHIFT_DR, writeOpcode);
            USBDM_JTAG_Write(16, JTAG_EXIT_SHIFT_DR, dataOutPtrX);
         }
         else {
            USBDM_JTAG_Write(ONCE_CMD_LENGTH, JTAG_EXIT_SHIFT_DR, writeOpcodeAndGo);
            USBDM_JTAG_Write(16, JTAG_EXIT_SHIFT_IR, dataOutPtrX);
         }
         dataOutPtrX += 2;
      } while (--numWords>0);
   } while (--numberOfInstructions>0);
   USBDM_JTAG_Write(JTAG_CORE_COMMAND_LENGTH, JTAG_EXIT_SHIFT_IR, enableONCECommand);
   USBDM_JTAG_Write(JTAG_CORE_COMMAND_LENGTH, JTAG_EXIT_SHIFT_IR, enableONCECommand);
   retry = MAX_DSC_RETRY;
   do {
      USBDM_JTAG_ReadWrite(JTAG_CORE_COMMAND_LENGTH, JTAG_EXIT_SHIFT_IR, enableONCECommand, &onceStatus);
   }  while ((onceStatus != TARGET_STATUS_DEBUG) && (retry-->0));
   // Move to IDLE
   USBDM_JTAG_Write(JTAG_CORE_COMMAND_LENGTH, JTAG_EXIT_IDLE, enableONCECommand);
   *dataOutPtr = dataOutPtrX;
//   return BDM_RC_OK;
   if (onceStatus == TARGET_STATUS_DEBUG)
      return BDM_RC_OK;
   if (onceStatus == TARGET_STATUS_STOP)
      return BDM_RC_TARGET_BUSY;
   if (onceStatus == TARGET_STATUS_EXECUTE)
      return BDM_RC_TARGET_BUSY;
   if (onceStatus == TARGET_STATUS_EX_ACCESS)
      return BDM_RC_TARGET_BUSY;
   return BDM_RC_NO_CONNECTION;
}

//! Skip of DSC instruction sequence
//!
//! @param dataOutPtr ptr to current position in JTAG instruction sequence
//!
//! @return error code
//!
//! Parameters taken from DP
//!   2, // # of instructions
//!      // Length  Instruction data...
//!           2,    0x87,0x44, 0xAA,0x34,
//!           3,    0xE7,0x7F, 0xD4,0x7C, 0xFF,0xFF,
void skipDSCTargetInstructionSequence(const U8 **dataOutPtr) {
	U8 numberOfInstructions;
	const U8 *dataOutPtrX = *dataOutPtr;
	
	numberOfInstructions = *dataOutPtrX++;
    do {
    	dataOutPtrX += 2* *dataOutPtrX++; // Skip instruction
    } while (--numberOfInstructions>0);
    *dataOutPtr = dataOutPtrX;
}

/*
 * Entry
 *    +-----------------------+  
 *    |  # of memory elements | <- dataOutPtr
 *    +-----------------------+
 *    |  DSC instructions     |
 *    | ..................... |
 *    +-----------------------+
 *    |  Size of elements     |
 *    +-----------------------+
 */
U8 DSC_readMemory(const U8 **dataOutPtr) {
    U8               numberOfElements;
    const U8        *dscInstructionPtr;
    U8 	             memoryElementSize;
    const U8        *dataOutPtrX = *dataOutPtr;
    static const U8  readOnceCommand[] = {ONCE_CMD_READ|OTX_ADDRESS}; 
    U8               buffer[4];

	numberOfElements   = *dataOutPtrX++;               // Number of memory elements to read
	dscInstructionPtr  = dataOutPtrX;                  // Save pointer to DSC instruction sequence
	dataOutPtrX       += 7;                            // Skip over DSC code
	memoryElementSize  = *dataOutPtrX++;               // Size of each memory element
    *dataOutPtr = dataOutPtrX;
	do {
		const U8 *temp = dscInstructionPtr;
		rc = executeDSCTargetInstructionSequence(&temp);    // Execute DSC code (memory value now in OTX/OTX1)
		if (rc != BDM_RC_OK) {
			return rc;
		}
		// Read EONCE reg OTX/OTX1
        jtag_transition_shift(JTAG_SHIFT_DR);                             // Move to SCAN-DR (EONCE)
        jtag_write(JTAG_EXIT_SHIFT_DR, ONCE_CMD_LENGTH, readOnceCommand); // Exit & re-enter SCAN-DR after read OTX command
        jtag_read(JTAG_EXIT_IDLE, 32, buffer);                            // Read data value
        
        // Save relevant portion of data
        switch(memoryElementSize) {
        case  8: *dataInPtr++ = buffer[3]; break;
        case 16: *dataInPtr++ = buffer[2];
        	     *dataInPtr++ = buffer[3]; break;
        case 32: *dataInPtr++ = buffer[0];
				 *dataInPtr++ = buffer[1];
				 *dataInPtr++ = buffer[2];
                 *dataInPtr++ = buffer[3]; break;
        default: return BDM_RC_ILLEGAL_PARAMS;
        }
    } while (--numberOfElements>0);
	return BDM_RC_OK;
}

/*
 * Entry
 *    +-----------------------+
 *    |                       |  <- dataOutPtr
 *    +--                   --+
 *    |                       |
 *    +--  Memory Address   --+
 *    |                       |
 *    +--                  ---+
 *    |                       |
 *    +-----------------------+
 *    |  # of memory elements |
 *    +-----------------------+
 *    |   Memory Space        |
 *    +-----------------------+
 */
U8 DSC_fastReadMemory(const U8 **dataOutPtr) {
	static const uint8_t loadAddressSequence[] = {
		/*  0 */      2,
		// Point R4 at otx (otx1 requires +1 offset)
		/*  1 */      3,  0xE4,0x1C, 0xFF,0xFE, 0x00,0xFF,  // move #X:otx,R4
		// Load starting memory address into R0
		/*  8 */      3,  0xE4,0x18,              // move #$332211,R0
		/* 11 */      //  0x22,0x11, 0x00,0x33,
	};
	// X Memory Reads
	// Target code to transfer an 8/16/32-bit value from X:(R0)+ into otx/otx1
	// Assumes R4 points at otx
	//
	static const uint8_t readXMem8[]  = {
	      2, 1, 0xF8,0xA0,             // readxb    moveu.bp  X:(R0)+,A
	         1, 0xD0,0x1C,             //           move.w    A1,X:(R4)
	};
	static const uint8_t readXMem16[] = {
	      2, 1, 0xF6,0x80,             // readxw    move.w    X:(R0)+,A0
	         1, 0xD6,0x9C,             //           move.w    A0,X:(R4)
	};
	static const uint8_t readXMem32[] = {
	      2, 1, 0xF0,0x20,             // readxl    move.l    X:(R0)+,A
	         1, 0xD0,0x3C,             //           move.l    A10,X:(R4)
	};
	// P Memory Reads
	// Target code to transfer a 16-bit value from P:(R0)+ into otx1
	static const uint8_t readPMem16[] = {
	      2, 1, 0x83,0x68,             // readpw    move.w    P:(R0)+,A1
	         1, 0xD0,0x1C,             //           move.w    A1,X:(R4)
	};
	uint8_t               buffer[sizeof(loadAddressSequence)+4];
	uint8_t               numberOfElements;
	uint8_t               memorySpace;
	uint8_t               rc;
	const uint8_t        *temp;
	const uint8_t        *dataOutPtrX = *dataOutPtr;
	static const uint8_t  readOnceCommand[] = {ONCE_CMD_READ|OTX_ADDRESS};

	// Execute DSC code - load mem address into R0, otx address into R4
	(void)memcpy(buffer, loadAddressSequence, sizeof(loadAddressSequence));
	buffer[sizeof(loadAddressSequence)+2] = *dataOutPtrX++;
	buffer[sizeof(loadAddressSequence)+3] = *dataOutPtrX++;
	buffer[sizeof(loadAddressSequence)+0] = *dataOutPtrX++;
	buffer[sizeof(loadAddressSequence)+1] = *dataOutPtrX++;
	temp = buffer;
	rc = executeDSCTargetInstructionSequence(&temp);
	if (rc != BDM_RC_OK) {
		return rc;
	}
	numberOfElements = *dataOutPtrX++;  // Number of memory elements to read
	memorySpace      = *dataOutPtrX++;  // Memory space (including size)
	do {
		// Select DSC code sequence
		switch(memorySpace) {
		   case MS_XByte: temp = readXMem8;  break;
		   case MS_XWord: temp = readXMem16; break;
		   case MS_XLong: temp = readXMem32; break;
		   case MS_PWord: temp = readPMem16; break;
		   default: return BDM_RC_ILLEGAL_PARAMS;
		}
		// Execute DSC code (write data to memory)
		rc = executeDSCTargetInstructionSequence(&temp);
		if (rc != BDM_RC_OK) {
			return rc;
		}
		*dataOutPtr = dataOutPtrX;

		// Read EONCE reg OTX/OTX1
		jtag_transition_shift(JTAG_SHIFT_DR);                             // Move to SCAN-DR (EONCE)
		jtag_write(JTAG_EXIT_SHIFT_DR, ONCE_CMD_LENGTH, readOnceCommand); // Exit & re-enter SCAN-DR after read OTX command
		jtag_read(JTAG_EXIT_IDLE, 32, buffer);                            // Read data value

		// Save relevant portion of data
		switch(memorySpace&MS_SIZE) {
           case MS_Byte: 
           	  *dataInPtr++ = buffer[3]; break;
           case MS_Word: 
           	  *dataInPtr++ = buffer[3];
           	  *dataInPtr++ = buffer[2]; break;
           case MS_Long: 
           	  *dataInPtr++ = buffer[3];
           	  *dataInPtr++ = buffer[2];
           	  *dataInPtr++ = buffer[1];
           	  *dataInPtr++ = buffer[0]; break;
           default: return BDM_RC_ILLEGAL_PARAMS;
		}
	} while (--numberOfElements>0);
	return BDM_RC_OK;
}

#if 1
/*
 *    +-----------------------+
 *    |                       |  <- dataOutPtr
 *    +--                   --+
 *    |                       |
 *    +--  Memory Address   --+
 *    |                       |
 *    +--                  ---+
 *    |                       |
 *    +-----------------------+
 *    |  # of memory elements |
 *    +-----------------------+
 *    |   Memory Space        |
 *    +-----------------------+
 *    | ..................... |
 *    = ...... data ......... =
 *    | ..................... |
 *    +-----------------------+
 */
U8 DSC_fastWriteMemory(const uint8_t **dataOutPtr) {
   static const uint8_t loadAddressSequence[] = {
   /*  0 */      2,
                     // Point R4 at otx (otx1 requires +1 offset)
   /*  1 */      3,  0xE4,0x1C, 0xFF,0xFE, 0x00,0xFF,  // move #X:otx,R4
                     // Load starting memory address into R0
   /*  8 */      3,  0xE4,0x18,              // move #$332211,R0
   /* 11 */      //  0x22,0x11, 0x00,0x33,
   };
   static const uint8_t writeXMem8[]  = {
         3,
         1, 0xB4,0x0C,                 // writexb  move.w      X:(R4+1),X0
         1, 0x5E,0x28,                 //          lsrr.w      #8,X0
         1, 0xD4,0xA0,                 //          move.bp     X0,X:(R0)+
   };
   static const uint8_t writeXMem16[] = {
         2,
         1, 0xB4,0x0C,                 // writexw  move.w      X:(R4+1),X0
         1, 0xD4,0x00,                 //          move.w      X0,X:(R0)+
   };
   static const uint8_t writeXMem32[] = {
         2, 
         1, 0xF0,0x3C,                 // writexl  move.l      X:(R4),A
         1, 0xD0,0x20,                 //          move.l      A10,X:(R0)+
   };
   static const uint8_t writePMem16[] = {
         2, 
         1, 0xB4,0x0C,                 // writepw  move.w      X:(R4+1),X0
         1, 0x84,0x60,                 //          move.w      X0,P:(R0)+
   };
   uint8_t               buffer[sizeof(loadAddressSequence)+4];
   uint8_t               numberOfElements;
   uint8_t               memorySpace;
   uint8_t               rc;
   const uint8_t        *temp;
   const uint8_t        *dataOutPtrX = *dataOutPtr;
   static const uint8_t  writeOnceCommand[] = {ONCE_CMD_WRITE|ORX_ADDRESS};

   // Execute DSC code - load mem address into R0, otx address into R4
   (void)memcpy(buffer, loadAddressSequence, sizeof(loadAddressSequence));
   buffer[sizeof(loadAddressSequence)+2] = *dataOutPtrX++;
   buffer[sizeof(loadAddressSequence)+3] = *dataOutPtrX++;
   buffer[sizeof(loadAddressSequence)+0] = *dataOutPtrX++;
   buffer[sizeof(loadAddressSequence)+1] = *dataOutPtrX++;
   temp = buffer;
   rc = executeDSCTargetInstructionSequence(&temp);
   if (rc != BDM_RC_OK) {
	   return rc;
   }
   numberOfElements = *dataOutPtrX++;  // Number of memory elements to read
   memorySpace      = *dataOutPtrX++;  // Memory space (including size)

   do {
      // Fill relevant portion of buffer with data
      switch(memorySpace&MS_SIZE) {
      case MS_Byte:
         buffer[0] = *dataOutPtrX++; break;
      case MS_Word:
         buffer[1] = *dataOutPtrX++;
         buffer[0] = *dataOutPtrX++; break;
      case MS_Long:
         buffer[3] = *dataOutPtrX++;
         buffer[2] = *dataOutPtrX++;
         buffer[1] = *dataOutPtrX++;
         buffer[0] = *dataOutPtrX++; break;
      default: return BDM_RC_ILLEGAL_PARAMS;
      }
      // Write EONCE reg OTX/OTX1
      USBDM_JTAG_SelectShift(JTAG_SHIFT_DR);                                   // Move to SCAN-DR (EONCE)
      USBDM_JTAG_Write(ONCE_CMD_LENGTH, JTAG_EXIT_SHIFT_DR, writeOnceCommand); // Exit & re-enter SCAN-DR after read OTX command
      USBDM_JTAG_Write(32,              JTAG_EXIT_IDLE,     buffer);           // Write data value

      // Select DSC code sequence
      switch(memorySpace) {
         case MS_XByte: temp = writeXMem8;  break;
         case MS_XWord: temp = writeXMem16; break;
         case MS_XLong: temp = writeXMem32; break;
         case MS_PWord: temp = writePMem16; break;
         default: return BDM_RC_ILLEGAL_PARAMS;
      }
      // Execute DSC code (write data to memory)
      rc = executeDSCTargetInstructionSequence(&temp);
      if (rc != BDM_RC_OK) {
         return rc;
      }
   } while (--numberOfElements>0);
   *dataOutPtr = dataOutPtrX;
   return BDM_RC_OK;
}
#else
/*
 *    Entry
 *    +-----------------------+
 *    |    JTAG_WRITE_MEM     |
 *    +-----------------------+
 *    |                       |<- *dataOutPtr
 *    +--                   --+
 *    |                       |
 *    +--  Memory Address   --+
 *    |                       |
 *    +--                  ---+
 *    |                       |
 *    +-----------------------+
 *    |  # of memory elements |
 *    +-----------------------+
 *    |   Memory Space        |
 *    +-----------------------+
 *    | ..................... |
 *    = ...... data ......... =
 *    | ..................... |
 *    +-----------------------+
 */
U8 DSC_fastWriteMemory(const uint8_t **dataOutPtr) {
	   static const uint8_t loadAddressSequence[] = {
	         // Load address into R0
	         1,
	         3, 0xE4,0x18, 0x22,0x11, 0x00,0x33, // move.l      #$332211,R0
	   };
	   static const uint8_t writeXByte[] = {
	         // Write byte to X:(R0)+
	         2,
	         2, 0x87,0x50, 0x00,0xFF,            // move.w      #$00FF,A
	         1, 0xD0,0xA0,                       // move.bp     A1,X:(R0)+
	   };
	   static const uint8_t writeXWord[] = {
	         // Write word to X:(R0)+
	         2,
	         2, 0x87,0x50, 0x22,0x11,            // move.w      #$2211,A1
	         1, 0xD0,0x00,                       // move.w      A1,X:(R0)+
	   };
	   static const uint8_t writeXLong[] = {
	         // Write longword to X:(R0)+
	         2,
	         3, 0xE4,0x10, 0x22,0x11, 0x44,0x33, //  move.l      #$44332211,A
	         1, 0xD0,0x20,                       //  move.l      A10,X:(R0)+
	   };
	   static const uint8_t writePWord[] = {
	         // Write word to P:(R0)+
	         2,
	         2, 0x87,0x50, 0x22,0x11,            // move.w      #$2211,A1
	         1, 0x83,0x60,                       // move.w      A1,P:(R0)+
	   };
	   uint8_t               buffer[sizeof(writeXLong)];
	   uint8_t               numberOfElements;
	   uint8_t               memorySpace;
	   uint8_t               rc;
	   uint8_t              *temp;
	   const uint8_t        *dataOutPtrX = *dataOutPtr;
	   
	   // Copy address
	   (void)memcpy(buffer, loadAddressSequence, sizeof(loadAddressSequence));
	   buffer[6] = *dataOutPtrX++;
	   buffer[7] = *dataOutPtrX++;
	   buffer[4] = *dataOutPtrX++;
	   buffer[5] = *dataOutPtrX++;
	   temp = buffer;
	   // Execute DSC code - copy address to R0
	   rc = executeDSCTargetInstructionSequence(&temp); 

	   numberOfElements = *dataOutPtrX++;  // Number of memory elements to read
	   memorySpace      = *dataOutPtrX++;  // Memory space (including size)

	   // Copy DSC instructions to buffer
	   switch(memorySpace) {
	      case MS_XByte: (void)memcpy(buffer, writeXByte, sizeof(writeXByte)); break;
	      case MS_XWord: (void)memcpy(buffer, writeXWord, sizeof(writeXWord)); break;
	      case MS_XLong: (void)memcpy(buffer, writeXLong, sizeof(writeXLong)); break;
	      case MS_PWord: (void)memcpy(buffer, writePWord, sizeof(writePWord)); break;
	      default: return BDM_RC_ILLEGAL_PARAMS;
	   }
	   do {
	      temp = buffer;
	      // Fill relevant portion of buffer with data
	      switch(memorySpace&MS_SIZE) {
			  case MS_Byte:
				 buffer[5] = *dataOutPtrX++;
			  break;
			  case MS_Word:
				 buffer[5] = *dataOutPtrX++;
				 buffer[4] = *dataOutPtrX++;
			  break;
			  case MS_Long:
				 buffer[5] = *dataOutPtrX++;
				 buffer[4] = *dataOutPtrX++;
				 buffer[7] = *dataOutPtrX++;
				 buffer[6] = *dataOutPtrX++;
			  break;
			  default: return BDM_RC_ILLEGAL_PARAMS;
	      }
	      // Execute DSC code (write immediate data to memory)
	      rc = executeDSCTargetInstructionSequence(&temp); 
	      if (rc != BDM_RC_OK) {
	         return rc;
	      }
	   } while (--numberOfElements>0);
	   *dataOutPtr = dataOutPtrX;
	   return BDM_RC_OK;
}
#endif
/*
 * Entry
 *    +-----------------------+  <- dataOutPtr
 *    |  # of memory elements |
 *    +-----------------------+
 *    |  DSC instructions     |
 *    | ..................... |
 *    +-----------------------+
 *    |  Size of elements     |
 *    +-----------------------+
 *    | ..................... |
 *    = ...... data ......... =
 *    | ..................... |
 *    +-----------------------+
 */
U8 DSC_writeMemory(const U8 **dataOutPtr) {
    U8               numberOfElements;
    const U8        *dscInstructionPtr;
    U8 	             memoryElementSize;
    const U8        *dataOutPtrX = *dataOutPtr;
    static const U8  writeOnceCommand[] = {ONCE_CMD_WRITE|ORX_ADDRESS}; 
    U8               buffer[4];

	dscInstructionPtr  = dataOutPtrX;    // Save pointer to DSC instruction sequence
	dataOutPtrX       += 10;             // Skip over DSC code + padding
	memoryElementSize  = *dataOutPtrX++; // Size of each memory element
	numberOfElements   = *dataOutPtrX++; // Number of memory elements to read
	do {
		const U8 *temp = dscInstructionPtr;

        // Fill relevant portion of buffer with data
        switch(memoryElementSize) {
        case 8:  buffer[1] = *dataOutPtrX++; break;
        case 16: buffer[0] = *dataOutPtrX++;
                 buffer[1] = *dataOutPtrX++; break;
        case 32: buffer[0] = *dataOutPtrX++;
				 buffer[1] = *dataOutPtrX++;
				 buffer[2] = *dataOutPtrX++;
                 buffer[3] = *dataOutPtrX++; break;
        default: return BDM_RC_ILLEGAL_PARAMS;
        }
		// Write EONCE reg OTX/OTX1
        jtag_transition_shift(JTAG_SHIFT_DR);                             // Move to SCAN-DR (EONCE)
        jtag_write(JTAG_EXIT_SHIFT_DR, ONCE_CMD_LENGTH, writeOnceCommand); // Exit & re-enter SCAN-DR after read OTX command
        jtag_write(JTAG_EXIT_IDLE, 32, buffer);                            // Write data value
        
		rc = executeDSCTargetInstructionSequence(&temp);    // Execute DSC code (copy value in OTX/OTX1 to memory)
		if (rc != BDM_RC_OK) {
			return rc;
		}
    } while (--numberOfElements>0);
    *dataOutPtr = dataOutPtrX;
	return BDM_RC_OK;
}

#if (TARGET_CAPABILITY & CAP_ARM_JTAG) && !defined(MINIMAL_MEMORY_USE)
// ARM JTAG Commands
#define ARM_JTAG_MASTER_IR_LENGTH   (4)     // IR length for commands below

#define JTAG_DP_DPACC_SEL_LENGTH    (35)
#define JTAG_DP_DPACC_SEL_COMMAND   (0x0A)  // JTAG-DP DP Access Register (DPACC)
#define JTAG_DP_APACC_SEL_LENGTH    (35)
#define JTAG_DP_APACC_SEL_COMMAND   (0x0B)  // JTAG-DP AP Access Register (APACC)

#define DP_AP_WRITE           (0x0)
#define DP_AP_READ            (0x1)

#define DP_CTRL_STAT_REG   (0x2) //!< R/W access DP STATUS/CONTROL registers
#define DP_SELECT_REG      (0x4) //!< R/W access AP SELECT register
#define DP_RDBUFF_REG      (0x6) //!< RAX/WI access to RDBUFF register

// Responses from DP/AP access
#define ACK_OK_FAULT       (0x02) //!< Access completed (either OK or FAULT)
#define ACK_WAIT           (0x01) //!< Access incomplete - try again

//! How many times to retry a MEM-AP access
#define MAX_ARM_RETRY      (30)

//! This routine is a hard-coding of a time-critical ARM JTAG subroutine
//! Reads multiple from AP register (sets AP_SELECT & APACC)
//!
//! @param  inline parameters
//!   numWords // 8-bit number of words to read from address \n
//!   address  // 16-bit abbreviated address in AP space \n
//!    A[15:8]  => DP-AP-SELECT[31:24] (AP Select) \n
//!    A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
//!    A[3:2]   => APACC[3:2]          (Register select within bank)
//!
//! @param dataInPtr ptr to Input buffer to return values \n
//!    4*numWords => data-32 read \n
//!    4          => Control/Status register value
//!
//! @return error code
//!
U8 ARM_readAP(const U8 **sequence, U8 **dataInPtr) {
#if 1
   const U8 writeSelect        = DP_AP_WRITE|DP_SELECT_REG;
   const U8 readRdBuff         = DP_AP_READ|DP_RDBUFF_REG;
   const U8 readStatus         = DP_AP_READ|DP_CTRL_STAT_REG;
   const U8 dummyValue[4]      = {0,0,0,0};
   const U8 dpAccSelectCommand = JTAG_DP_DPACC_SEL_COMMAND;
   const U8 apAccSelectCommand = JTAG_DP_APACC_SEL_COMMAND;
   U8 reg32RnW;
   U8 selectValue[4];
   U8 ack;
   U8 increment;
   U8 retry = MAX_ARM_RETRY;

   U8 numWords    = *(*sequence)++;
   selectValue[0] = *(*sequence)++;
   selectValue[1] = 0;
   selectValue[2] = 0;
   selectValue[3] = **sequence&0xF0;
   reg32RnW       = DP_AP_READ|((*(*sequence)++&0x0C)>>1);
   
   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   USBDM_JTAG_SelectShift(JTAG_SHIFT_IR);
   USBDM_JTAG_Write(ARM_JTAG_MASTER_IR_LENGTH, JTAG_EXIT_SHIFT_DR, &dpAccSelectCommand);

   // Write operation/read status, stay in SHIFT-DR
   USBDM_JTAG_ReadWrite(3, JTAG_STAY_SHIFT, &writeSelect, &ack);
   // Write data, exit & enter SHIFT-IR afterwards
   USBDM_JTAG_Write(32, JTAG_EXIT_SHIFT_IR, selectValue);
   if (ack != ACK_OK_FAULT)
	   return BDM_RC_ACK_TIMEOUT;

   // Write APACC_SEL command to IR, move to JTAG_SHIFT_DR
   USBDM_JTAG_Write(ARM_JTAG_MASTER_IR_LENGTH, JTAG_EXIT_SHIFT_DR, &apAccSelectCommand);

   increment = 0; // Don't increment address for 1st read (discard data)
   while (numWords > 0) {
      // Write operation/read status, stay in SHIFT-DR
      USBDM_JTAG_ReadWrite(3,  JTAG_STAY_SHIFT, &reg32RnW, &ack);
      if (ack != ACK_OK_FAULT) {
    	  // Complete failed transaction
    	  USBDM_JTAG_Write(32, JTAG_EXIT_SHIFT_DR, dummyValue);
	      if (ack != ACK_WAIT)
	    	  return BDM_RC_NO_CONNECTION;
		  if (retry-- > 0)
			 continue;
		  return BDM_RC_ACK_TIMEOUT;
      }
      if (numWords==1) {
         // Read data, exit & move to SHIFT-IR afterwards for last word
         USBDM_JTAG_Read(32, JTAG_EXIT_SHIFT_IR, *dataInPtr);
      }
      else {
          // Read data, exit & re-enter SHIFT-DR afterwards if not last word
    	  USBDM_JTAG_Read(32, JTAG_EXIT_SHIFT_DR, *dataInPtr);
      }
      (*dataInPtr) += increment;
      increment = 4;
      numWords--;
      retry = MAX_ARM_RETRY;
   }
   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   USBDM_JTAG_Write(ARM_JTAG_MASTER_IR_LENGTH, JTAG_EXIT_SHIFT_DR, &dpAccSelectCommand);

   // Write operation/read status, stay in SHIFT-DR
   USBDM_JTAG_ReadWrite(3,  JTAG_STAY_SHIFT, &readStatus, &ack);
   // Read last data, exit & re-enter SHIFT-DR
   USBDM_JTAG_Read(32, JTAG_EXIT_SHIFT_DR, *dataInPtr);
   (*dataInPtr) += 4;
   if (ack != ACK_OK_FAULT)
	   return BDM_RC_ACK_TIMEOUT;

   // Write operation/read status, stay in SHIFT-DR
   USBDM_JTAG_ReadWrite(3,  JTAG_STAY_SHIFT, &readRdBuff, &ack);
   // Read Control/Status data, exit & re-enter SHIFT-DR
   USBDM_JTAG_Read(32, JTAG_EXIT_IDLE, *dataInPtr);
   (*dataInPtr) += 4;
   if (ack != ACK_OK_FAULT)
	   return BDM_RC_ACK_TIMEOUT;
#endif
   return BDM_RC_OK;
}

//! This routine is a hard-coding of a time-critical ARM JTAG subroutine
//! Write multiple to AP register (sets AP_SELECT & APACC)
//!
//! @param  inline parameters
//!   numWords // 8-bit number of words to write to address \n
//!   address  // 16-bit abbreviated address \n
//!    A[15:8]  => DP-AP-SELECT[31:24] (AP Select) \n
//!    A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
//!    A[3:2]   => APACC[3:2]          (Register select within bank)
//!
//! @param dataOutPtr ptr to buffer to output to ARM device \n
//!    4*numWords => data-32 to write
//! @param dataInPtr ptr to Input buffer to return values \n
//!    4          => Control/Status register value
//!
//! @return error code
//!
U8 ARM_writeAP(const U8 **sequence, U8 **dataInPtr, U8 **dataOutPtr) {
#if 1
   const U8 writeSelect        = DP_AP_WRITE|DP_SELECT_REG;
   const U8 readRdBuff         = DP_AP_READ|DP_RDBUFF_REG;
   const U8 readStatus         = DP_AP_READ|DP_CTRL_STAT_REG;
   const U8 dummyValue[4]      = {0,0,0,0};
   const U8 dpAccSelectCommand = JTAG_DP_DPACC_SEL_COMMAND;
   const U8 apAccSelectCommand = JTAG_DP_APACC_SEL_COMMAND;
   U8 reg32RnW;
   U8 selectValue[4];
   U8 ack;
   U8 retry = MAX_ARM_RETRY;

   U8 numWords    = *(*sequence)++;
   selectValue[0] = *(*sequence)++;
   selectValue[1] = 0;
   selectValue[2] = 0;
   selectValue[3] = **sequence&0xF0;
   reg32RnW       = DP_AP_WRITE|((*(*sequence)++&0x0C)>>1);

   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   USBDM_JTAG_SelectShift(JTAG_SHIFT_IR);
   USBDM_JTAG_Write(ARM_JTAG_MASTER_IR_LENGTH, JTAG_EXIT_SHIFT_DR, &dpAccSelectCommand);

   // Write operation/read status, stay in SHIFT-DR
   USBDM_JTAG_ReadWrite(3, JTAG_STAY_SHIFT, &writeSelect, &ack);
   // Write data, exit & enter SHIFT-IR afterwards
   USBDM_JTAG_Write(32, JTAG_EXIT_SHIFT_IR, selectValue);
   if (ack != ACK_OK_FAULT) {
	   return BDM_RC_ACK_TIMEOUT;
   }
   // Write APACC_SEL command to IR, move to JTAG_SHIFT_DR
   USBDM_JTAG_Write(ARM_JTAG_MASTER_IR_LENGTH, JTAG_EXIT_SHIFT_DR, &apAccSelectCommand);

   while (numWords > 0) {
      // Write operation/read status, stay in SHIFT-DR
      USBDM_JTAG_ReadWrite(3,  JTAG_STAY_SHIFT, &reg32RnW, &ack);
      if (ack != ACK_OK_FAULT) {
    	  // Complete failed transaction
    	  USBDM_JTAG_Write(32, JTAG_EXIT_SHIFT_DR, dummyValue);
	      if (ack != ACK_WAIT)
	    	  return BDM_RC_NO_CONNECTION;
		  if (retry-- > 0)
			 continue;
		  return BDM_RC_ACK_TIMEOUT;
      }
      if (numWords==1) {
         // Write data, exit & move to SHIFT-IR afterwards for last word
         USBDM_JTAG_Write(32, JTAG_EXIT_SHIFT_IR, *dataOutPtr);
      }
      else {
          // Write data, exit & re-enter SHIFT-DR afterwards if not last word
    	  USBDM_JTAG_Write(32, JTAG_EXIT_SHIFT_DR, *dataOutPtr);
      }
      (*dataOutPtr) += 4;
      numWords--;
      retry = MAX_ARM_RETRY;
   }
   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   USBDM_JTAG_Write(ARM_JTAG_MASTER_IR_LENGTH, JTAG_EXIT_SHIFT_DR, &dpAccSelectCommand);
   
   // Write operation/read status, stay in SHIFT-DR
   USBDM_JTAG_ReadWrite(3,  JTAG_STAY_SHIFT, &readStatus, &ack);
   // Complete transaction
   USBDM_JTAG_Write(32, JTAG_EXIT_SHIFT_DR, dummyValue);
   if (ack != ACK_OK_FAULT) {
	   return BDM_RC_ACK_TIMEOUT;
   }
   // Write operation/read status, stay in SHIFT-DR
   USBDM_JTAG_ReadWrite(3,  JTAG_STAY_SHIFT, &readRdBuff, &ack);
   // Read Control/Status data, exit & re-enter SHIFT-DR
   USBDM_JTAG_Read(32, JTAG_EXIT_IDLE, *dataInPtr);
   (*dataInPtr) += 4;
   if (ack != ACK_OK_FAULT) {
	   return BDM_RC_ACK_TIMEOUT;
   }
#endif
   return BDM_RC_OK;
}

//! This routine is a hard-coding of a time-critical ARM JTAG subroutine
//! Write immediate value to AP register (sets AP_SELECT & APACC)
//!
//! @param  inline parameters
//!   address  // 16-bit abbreviated address
//!    A[15:8]  => DP-AP-SELECT[31:24] (AP Select) \n
//!    A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
//!    A[3:2]   => APACC[3:2]          (Register select within bank)
//!    4        => data-32 to write
//!
//! @return Error code
//!
//! @note Expects immediate ACK_OK_FAULT on AP transaction (non-memory)
//!
U8 ARM_writeAPI(const U8 **sequence) {

#if 1
	const U8 *address;
	const U8 *data;
	
	address = *sequence;
	(*sequence) += 2;
	data    = *sequence;
	(*sequence) += 4;
	
	return arm_writeAPReg(address, data);   
   
#else   
   const U8 writeSelect        = DP_AP_WRITE|DP_SELECT_REG;
   const U8 readRdBuff         = DP_AP_READ|DP_RDBUFF_REG;
   const U8 dummyValue[4]      = {0,0,0,0};
   const U8 dpAccSelectCommand = JTAG_DP_DPACC_SEL_COMMAND;
   const U8 apAccSelectCommand = JTAG_DP_APACC_SEL_COMMAND;
   U8 reg32RnW;
   U8 selectValue[4];
   U8 ack;

   selectValue[0] = *(*sequence)++;
   selectValue[1] = 0;
   selectValue[2] = 0;
   selectValue[3] = **sequence&0xF0;
   reg32RnW       = DP_AP_WRITE|((*(*sequence)++&0x0C)>>1);

   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   USBDM_JTAG_SelectShift(JTAG_SHIFT_IR);
   USBDM_JTAG_Write(ARM_JTAG_MASTER_IR_LENGTH, JTAG_EXIT_SHIFT_DR, &dpAccSelectCommand);

   // Write operation/read status, stay in SHIFT-DR
   USBDM_JTAG_ReadWrite(3, JTAG_STAY_SHIFT, &writeSelect, &ack);
   // Write data, exit & enter SHIFT-IR afterwards
   USBDM_JTAG_Write(32, JTAG_EXIT_SHIFT_IR, selectValue);
   if (ack != ACK_OK_FAULT)
	   return BDM_RC_ACK_TIMEOUT;
   
   // Write APACC_SEL command to IR, move to JTAG_SHIFT_DR
   USBDM_JTAG_Write(ARM_JTAG_MASTER_IR_LENGTH, JTAG_EXIT_SHIFT_DR, &apAccSelectCommand);

   // Write operation/read status, stay in SHIFT-DR
   USBDM_JTAG_ReadWrite(3,  JTAG_STAY_SHIFT, &reg32RnW, &ack);
   // Write data, exit & move to SHIFT-IR afterwards for last word
   USBDM_JTAG_Write(32, JTAG_EXIT_SHIFT_IR, *sequence);
   (*sequence) += 4;
   if (ack != ACK_OK_FAULT)
	   return BDM_RC_ACK_TIMEOUT;
   
   // Write DPACC_SEL command to IR, move to JTAG_SHIFT_DR
   USBDM_JTAG_Write(ARM_JTAG_MASTER_IR_LENGTH, JTAG_EXIT_SHIFT_DR, &dpAccSelectCommand);

   // Write operation/read status, stay in SHIFT-DR
   USBDM_JTAG_ReadWrite(3,  JTAG_STAY_SHIFT, &readRdBuff, &ack);
   // Complete transaction
   USBDM_JTAG_Write(32, JTAG_EXIT_IDLE, dummyValue);
   if (ack != ACK_OK_FAULT)
	   return BDM_RC_ACK_TIMEOUT;

   return BDM_RC_OK;
#endif   
}
#endif // MINIMAL_MEMORY_USE (omit ARM-JTAG code)

#endif  // INLINE_TARGET_INSTRUCTION_EXECUTION

#pragma MESSAGE DISABLE C4301 // Disable warnings about inline expansion

//! Obtains an 8-bit data value from the instruction stream or
//! data stream as required.
//! 
#pragma INLINE
static U8 getValueByte(U8 value) {
   if ((value == 0) && (dataOutPtr != NULL)) {
      // If zero get value from dataOutPtr
      value = *dataOutPtr++;
   }
   return value;
}

//! Execute JTAG instruction sequence
//!
//! @param sequenceStart  - start of sequence
//! @param dataInStart    - buffer for dataIn
//!
U8 processJTAGSequence(const U8 *sequenceStart, 
                             U8 *dataInStart) {
   complete          = false;
   inFill            = JTAG_WRITE_1;
   exitAction        = JTAG_EXIT_IDLE;
   rc                = BDM_RC_OK;                         
   iterator          = 0;
   startOfIteration  = NULL;
   tempValue         = 0;
   dataInPtr         = dataInStart;                         // Save start of dataIn
   dataInPtr++;                                             // Leave space for in length
   sequence          = sequenceStart;                       // Point to command sequence
   dataOutPtr        = skipSequence(sequence, STOP_ON_END); // Point to data out sequence
   if (*dataOutPtr == JTAG_END)
      dataOutPtr++;
   do {
      opcode      = *sequence++;
      regNo       = opcode & 0x03;                // In case needed
      if (opcode <= 80) // MISC commands
         switch (opcode) {
			case JTAG_SET_BUSY:
			   // Flag this may take a while
			   setBDMBusy();
			   break;
            case JTAG_DEBUG_ON:
               break;
            case JTAG_DEBUG_OFF:
               break;
            case JTAG_TEST_LOGIC_RESET:
               USBDM_JTAG_Reset();
               break;
            case JTAG_MOVE_DR_SCAN:
               USBDM_JTAG_SelectShift(JTAG_SHIFT_DR);
               break;
            case JTAG_MOVE_IR_SCAN:
               USBDM_JTAG_SelectShift(JTAG_SHIFT_IR);
               break;
            case JTAG_SET_STAY_SHIFT:
               exitAction = JTAG_STAY_SHIFT;
               break;
            case JTAG_SET_EXIT_SHIFT_DR:
               exitAction = JTAG_EXIT_SHIFT_DR;
               break;
            case JTAG_SET_EXIT_SHIFT_IR:
               exitAction = JTAG_EXIT_SHIFT_IR;
               break;
            case JTAG_SET_EXIT_IDLE:
               exitAction = JTAG_EXIT_IDLE;
               break;
            case JTAG_SET_IN_FILL_0:
               inFill = JTAG_WRITE_0;
               break;
            case JTAG_SET_IN_FILL_1:
               inFill = JTAG_WRITE_1;
               break;
            case JTAG_SAVE_SUB:
               // Save subroutine to cache
               adjustment = subroutineCache-sequenceStart;
               if (sequence-sequenceStart > sizeof(subroutineCache))
                  rc = BDM_RC_JTAG_TOO_LARGE;
               else {
                  (void)memcpy(subroutineCache, sequenceStart, sequence-sequenceStart);
                  subPtrs[0] += adjustment;
                  subPtrs[1] += adjustment;
                  subPtrs[2] += adjustment;
                  subPtrs[3] += adjustment;
               }
               break;
            case JTAG_SUBA:
            case JTAG_SUBB:
            case JTAG_SUBC:
            case JTAG_SUBD:
               subPtrs[regNo] = sequence;
               // Skip over subroutine
               sequence = skipSequence(sequence, STOP_ON_SUB);
               break;
            case JTAG_CALL_SUBA:
               if  (cable_status.target_type == T_MC56F80xx) {
                  rc = executeDSCTargetInstructionSequence(&dataOutPtr);
                  break;
               }
               goto doSubs;
               // Fall through otherwise
            case JTAG_CALL_SUBB:
                if  (cable_status.target_type == T_MC56F80xx) {
                   rc = DSC_readMemory(&dataOutPtr);
                   break;
                }
                goto doSubs;
                // Fall through otherwise
            case JTAG_CALL_SUBC:
                if  (cable_status.target_type == T_MC56F80xx) {
                   rc = DSC_writeMemory(&dataOutPtr);
                   break;
                }
                goto doSubs;
                // Fall through otherwise
            case JTAG_CALL_SUBD:
			doSubs:
               // Save iterators from parent
               repeatTOS->iterator    = iterator;
               repeatTOS->startOfLoop = startOfIteration;
               repeatTOS++;
               subroutineTOS->repeatTOS     = repeatTOS;
               subroutineTOS->returnAddress = sequence;
               subroutineTOS++;
               sequence       = subPtrs[regNo];
               break;
            case JTAG_READ_MEM:
                if  (cable_status.target_type != T_MC56F80xx) {
                   rc = BDM_RC_ILLEGAL_COMMAND;
                   break;
                }
            	rc = DSC_fastReadMemory(&dataOutPtr);
            	break;
            case JTAG_WRITE_MEM:
                if  (cable_status.target_type != T_MC56F80xx) {
                   rc = BDM_RC_ILLEGAL_COMMAND;
                   break;
                }
                rc = DSC_fastWriteMemory(&dataOutPtr);
            	break;
            case JTAG_BREAK:
               iterator = 1;
               // Fall through
            case JTAG_CONTINUE:
               sequence = skipSequence(sequence, STOP_ON_END_REPEAT|STOP_ON_SUB);
               break;
            case JTAG_END_REPEAT:
               // Check if unmatched
//                  if (((subroutineTOS != subroutineStack) && (repeatTOS == (subroutineTOS-1)->repeatTOS )) ||
//                      (repeatTOS == repeatStack)) {
//                     rc = BDM_RC_JTAG_UNMATCHED_REPEAT;
//                  }
//                  else 
               if (--iterator<=0) {
                  // End of loop - cleanup
                  repeatTOS--;
                  iterator         = repeatTOS->iterator;
                  startOfIteration = repeatTOS->startOfLoop;
               }
               else {
                  sequence = startOfIteration;
               }
               break;
            case JTAG_ELSE: // Skip to JTAG_END_IF
               sequence = skipSequence(sequence, STOP_ON_END_IF|STOP_ON_SUB);
               break;
            case JTAG_END_IF:
               break;
            case JTAG_NOP:
               break;
            case JTAG_SKIP_DP:   // Skip over N bytes in data Out sequence
               dataOutPtr += tempValue;
               break;
            case JTAG_END:
               complete = true;
               break;
            case JTAG_RETURN:
               // End of subroutine
               // Fall through
            case JTAG_END_SUB:
//                  if (subroutineTOS == subroutineStack) {
//                     // Stack underflow
//                     rc = BDM_RC_JTAG_STACK_ERROR;
//                  }
//                  else 
               {
                  subroutineTOS--;
                  sequence  = subroutineTOS->returnAddress;
                  repeatTOS = subroutineTOS->repeatTOS;
                  // restore parent iterator state
                  repeatTOS--;
                  iterator         = repeatTOS->iterator;
                  startOfIteration = repeatTOS->startOfLoop;

               }
               break;
            case JTAG_SHIFT_IN_DP : // Shift sequence in (8-bit count)/TL
               numBits = getValueByte(*sequence++);
//                  if (dataInPtr == NULL) {
//                     rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
//                  }
//                  else if (numBits == 0) {
//                     rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
//                  }
//                  else 
               {
                  USBDM_JTAG_Read(numBits, exitAction|inFill, dataInPtr);
                  dataInPtr += BITS_TO_BYTES(numBits);
               }
               break;
		   case JTAG_SHIFT_OUT_DP_VARA:
				 USBDM_JTAG_Write((U8)tempValue, exitAction, dataOutPtr);
				 dataOutPtr += BITS_TO_BYTES(tempValue);
                  break;
            case JTAG_SHIFT_OUT_DP : // Shift sequence out - sequence taken from dataOutPtr
               numBits = getValueByte(*sequence++);
//                  if (dataOutPtr == NULL) {
//                     // Requires data out ptr.
//                     rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
//                  }
//                  else if (numBits == 0) {
//                     rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
//                  }
//                  else 
               {
                  USBDM_JTAG_Write(numBits, exitAction, dataOutPtr);
                  dataOutPtr += BITS_TO_BYTES(numBits);
               }
               break;
            case JTAG_SHIFT_IN_OUT_DP : // Shift sequence in & out at same time - sequence taken from dataOutPtr
               numBits = getValueByte(*sequence++);
//                  if (dataOutPtr == NULL) {
//                     // Requires data out ptr.
//                     rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
//                  }
//                  else if (dataInPtr == NULL) {
//                     rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
//                  }
//                  else if (numBits == 0) {
//                     rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
//                  }
//                  else 
               {
                  USBDM_JTAG_ReadWrite(numBits, exitAction, dataOutPtr, dataInPtr);
                  dataOutPtr     += BITS_TO_BYTES(numBits);
                  dataInPtr      += BITS_TO_BYTES(numBits);
               }
               break;
            case JTAG_IF_VARA_EQ:  // IF statement testing variable A/B == value
            case JTAG_IF_VARB_EQ:
               if (variables[regNo] != tempValue) { // Fail => skip to ELSE/END_IF clause
                  sequence = skipSequence(sequence, STOP_ON_ELSE|STOP_ON_END_IF|STOP_ON_SUB);
               }
               if (*sequence == JTAG_ELSE)
                  sequence++;
               break;
            case JTAG_IF_VARA_NEQ: // IF statement testing variable A/B != value
            case JTAG_IF_VARB_NEQ:
               if (variables[regNo] == tempValue) { // Fail => skip to ELSE clause
                  sequence = skipSequence(sequence, STOP_ON_ELSE|STOP_ON_END_IF|STOP_ON_SUB);
               }
               if (*sequence == JTAG_ELSE)
                  sequence++;
               break;
            case JTAG_IF_ITER_EQ:
               if (iterator != tempValue) { // Fail => skip to ELSE clause
                  sequence = skipSequence(sequence, STOP_ON_ELSE|STOP_ON_END_IF|STOP_ON_SUB);
               }
               if (*sequence == JTAG_ELSE)
                  sequence++;
               break;
            case JTAG_IF_ITER_NEQ:
               if (iterator == tempValue) { // Fail => skip to ELSE clause
                  sequence = skipSequence(sequence, STOP_ON_ELSE|STOP_ON_END_IF|STOP_ON_SUB);
               }
               if (*sequence == JTAG_ELSE)
                  sequence++;
               break;
            case JTAG_REPEAT8:
               tempValue = *sequence++;
               // Fall through
            case JTAG_REPEAT:
               repeatTOS->iterator    = iterator;
               repeatTOS->startOfLoop = startOfIteration;
               repeatTOS++;
               iterator               = (U16)tempValue; // Assumes value set previously (e.g. push)
               startOfIteration       = sequence;
               break;
            case JTAG_SHIFT_OUT_VARA: // Shift out VarA/B to TDI, TDO discarded
            case JTAG_SHIFT_OUT_VARB:
            case JTAG_SHIFT_OUT_VARC:
            case JTAG_SHIFT_OUT_VARD:
               numBits = getValueByte(*sequence++);
//                  if ((numBits == 0) || (numBits > 32)) {
//                     rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
//                  }
//                  else 
               {
//                     U32 temp = variables[regNo];
//                     USBDM_JTAG_Write(numBits, exitAction, ((U8*)&temp+4-BITS_TO_BYTES(numBits)));
                  USBDM_JTAG_Write(numBits, exitAction, 
                     ((U8*)(variables+regNo)+sizeof(variables[0])-BITS_TO_BYTES(numBits)));
               }
               break;
            case JTAG_SHIFT_IN_OUT_VARA: // Set variable from TDO with TDI values from sequence
            case JTAG_SHIFT_IN_OUT_VARB:
            case JTAG_SHIFT_IN_OUT_VARC:
            case JTAG_SHIFT_IN_OUT_VARD:
               numBits = getValueByte(*sequence++);
//                  if ((numBits == 0) || (numBits > 32)) {
//                     rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
//                  }
//                  else 
               {
                  USBDM_JTAG_ReadWrite(numBits, exitAction, sequence, 
                     ((U8*)(variables+regNo)+sizeof(variables[0])-BITS_TO_BYTES(numBits)));
                  sequence += BITS_TO_BYTES(numBits);
               }
               break;
            case JTAG_SET_ERROR:    // Set error variable & exit
               rc = (USBDM_ErrorCode)getValueByte(*sequence++);
               break;
            case JTAG_PUSH8:
               tempValue = *sequence++;
               break;
            case JTAG_PUSH16:
               tempValue  = *(U16*)sequence;
               sequence  += 2;
//                  tempValue = *sequence++;
//                  tempValue = (tempValue<<8)+*sequence++;
               break;
            case JTAG_PUSH32:
               tempValue  = *(U32*)sequence;
               sequence  += 4;
//                  tempValue = *sequence++;
//                  tempValue = (tempValue<<8)+*sequence++;
//                  tempValue = (tempValue<<8)+*sequence++;
//                  tempValue = (tempValue<<8)+*sequence++;
               break;
            case JTAG_PUSH_DP_8:
               tempValue = *dataOutPtr++;
               break;
            case JTAG_PUSH_DP_16:
               tempValue  = *(U16*)dataOutPtr;
               sequence  += 2;
//                  tempValue = *dataOutPtr++;
//                  tempValue = (tempValue<<8)+*dataOutPtr++;
               break;
            case JTAG_PUSH_DP_32:
               tempValue  = *(U32*)dataOutPtr;
               sequence  += 4;
//                  tempValue = *dataOutPtr++;
//                  tempValue = (tempValue<<8)+*dataOutPtr++;
//                  tempValue = (tempValue<<8)+*dataOutPtr++;
//                  tempValue = (tempValue<<8)+*dataOutPtr++;
               break;
            case JTAG_SAVE_DP_VARC:
            case JTAG_SAVE_DP_VARD:
               variables[regNo] = dataOutPtr-sequenceStart;
               break;
            case JTAG_RESTORE_DP_VARC:
            case JTAG_RESTORE_DP_VARD:
               dataOutPtr = variables[regNo]+sequenceStart;
               break;
            case JTAG_LOAD_VARA:
            case JTAG_LOAD_VARB:
               variables[regNo] = tempValue;
               break;
#if (TARGET_CAPABILITY & CAP_ARM_JTAG) && !defined(MINIMAL_MEMORY_USE)
            case JTAG_ARM_READAP:
                if  (cable_status.target_type == T_ARM_JTAG) {
                   rc = ARM_readAP(&sequence, &dataInPtr);
                   break;
                }
                rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
            	break;
            case JTAG_ARM_WRITEAP:
                if  (cable_status.target_type == T_ARM_JTAG) {
               	   rc = ARM_writeAP(&sequence, &dataInPtr, &dataOutPtr);
                   break;
                }
                rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
            	break;
            case JTAG_ARM_WRITEAP_I:
                if  (cable_status.target_type == T_ARM_JTAG) {
               	   rc = ARM_writeAPI(&sequence);
                   break;
                }
                rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
            	break;
#endif
            case JTAG_SET_PADDING:// #4x16-bits - sets HDR HIR TDR TIR
            	jtag_set_hdr(*(U16*)sequence); sequence  += 2;
            	jtag_set_hir(*(U16*)sequence); sequence  += 2;
            	jtag_set_tdr(*(U16*)sequence); sequence  += 2;
            	jtag_set_tir(*(U16*)sequence); sequence  += 2;
            	break;
            default:
               rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
               break;
         }
      else { 
         numBits = (opcode&JTAG_NUM_BITS_MASK);
         if (numBits == 0)
            numBits = 32;
         switch (opcode&JTAG_COMMAND_MASK) {
            case JTAG_PUSH_Q(0): // Push a 5-bit value
               tempValue = numBits;
               break;
            case JTAG_REPEAT_Q(0):
               repeatTOS->iterator    = iterator;
               repeatTOS->startOfLoop = startOfIteration;
               repeatTOS++;
               iterator     = numBits;
               startOfIteration       = sequence;
               if (iterator==1) {
                  // Value of 1 indicates use dataOut value
                  iterator = *dataOutPtr++;
               }
   //            if (iterator == 0) {
   //               rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
   //            }
               break;
            case JTAG_SHIFT_IN_Q(0) : // Shift sequence in (5-bit count)
   //            if (dataInPtr == NULL) {
   //               rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
   //            }
   //            else 
               {
                  USBDM_JTAG_Read(numBits, exitAction|inFill, dataInPtr);
                  dataInPtr   += BITS_TO_BYTES(numBits);
               }
               break;
            case JTAG_SHIFT_OUT_Q(0) : // Shift sequence out (5-bit count) - sequence taken inline
               USBDM_JTAG_Write(numBits, exitAction, sequence);
               sequence += BITS_TO_BYTES(numBits);
               break;
            case JTAG_SHIFT_IN_OUT_Q(0) : // Shift sequence in & out at same time - sequence taken inline
   //            if (dataInPtr == NULL) {
   //               rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
   //            }
   //            else 
               {
                  USBDM_JTAG_ReadWrite(numBits, exitAction, sequence, dataInPtr);
                  sequence   += BITS_TO_BYTES(numBits);
                  dataInPtr  += BITS_TO_BYTES(numBits);
               }
               break;
            default:
               rc = BDM_RC_JTAG_ILLEGAL_SEQUENCE;
               break;
            }
         }
   } while (!complete && (rc == BDM_RC_OK));
   
   *dataInStart = (U8)(dataInPtr-dataInStart); // # bytes input
   return rc;
}
#endif
