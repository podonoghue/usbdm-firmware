/*
 * JTAGSequence.hpp
 *
 *  Created on: 27/05/2010
 *      Author: podonoghue
 */

#ifndef JTAGSEQUENCE_HPP_
#define JTAGSEQUENCE_HPP_

#include "Common.h"
#include "Commands.h"

#define JTAG_COMMAND_MASK        (0x7<<5)

#define JTAG_MISC0               (0<<5)
#define JTAG_MISC1               (1<<5)
#define JTAG_MISC2               (2<<5)
//============================================================================================
// The following have no operands
#define JTAG_END                 ( 0)  // Mark end of sequence
#define JTAG_NOP                 ( 1)  // No-Operation
#define JTAG_END_SUB             ( 2)  // Mark end of subroutine (also acts as implicit JTAG_RETURN)
#define JTAG_TEST_LOGIC_RESET    ( 3)  // Reset TAP
#define JTAG_MOVE_DR_SCAN        ( 4)  // Move TAP to JTAG_SHIFT-DR (from IDLE)
#define JTAG_MOVE_IR_SCAN        ( 5)  // Move TAP to JTAG_SHIFT-IR (from IDLE)
#define JTAG_SET_STAY_SHIFT      ( 6)  // Set Stay in JTAG_SHIFT-DR/IR after shift
#define JTAG_SET_EXIT_SHIFT_DR   ( 7)  // Set exit to JTAG_SHIFT-DR w/o crossing RUN-TEST-IDLE after shift
#define JTAG_SET_EXIT_SHIFT_IR   ( 8)  // Set exit to JTAG_SHIFT-IR w/o crossing RUN-TEST-IDLE after shift
#define JTAG_SET_EXIT_IDLE       ( 9)  // Set exit to RUN-TEST/IDLE after shift
#define JTAG_SET_IN_FILL_0       (10)  // Shift in '0' during JTAG_SHIFT_OUT
#define JTAG_SET_IN_FILL_1       (11)  // Shift in '1' during JTAG_SHIFT_OUT (default)

#define JTAG_ELSE                (12)  // Else Marker for JTAG_IF..
#define JTAG_END_IF              (13)  // EndIf Marker for JTAG_IF..
#define JTAG_RETURN              (14)  // Return from subroutine - ignores iteration
#define JTAG_BREAK               (15)  // Break JTAG_REPEAT loop
#define JTAG_CONTINUE            (16)  // Continue next JTAG_REPEAT iteration
#define JTAG_END_REPEAT          (17)  // Marks end of JTAG_REPEAT loop

//============================================================================================
// The following have an 8-bit operand as the next byte
                                       // Operand
#define JTAG_SET_ERROR           (18)  // Error#    Set error variable & exit sequence if != 0

#define JTAG_DEBUG_ON            (19)
#define JTAG_DEBUG_OFF           (63)

//============================================================================================
// The following have no operands
#define JTAG_SUB(x)                 (20+(x))
#define JTAG_SUBA                JTAG_SUB(0)       // Mark start of subroutine A
#define JTAG_SUBB                JTAG_SUB(1)       // Mark start of subroutine B
#define JTAG_SUBC                JTAG_SUB(2)       // Mark start of subroutine C
#define JTAG_SUBD                JTAG_SUB(3)       // Mark start of subroutine D

#define JTAG_CALL_SUB(x)                 (24+(x))
#define JTAG_CALL_SUBA           JTAG_CALL_SUB(0)  // Call subroutine A
#define JTAG_CALL_SUBB           JTAG_CALL_SUB(1)  // Call subroutine B
#define JTAG_CALL_SUBC           JTAG_CALL_SUB(2)  // Call subroutine C
#define JTAG_CALL_SUBD           JTAG_CALL_SUB(3)  // Call subroutine D

//============================================================================================
// The following use a value previously set by JTAG_PUSH...
                                       // 5/8/16/32 bit operand from JTAG_PUSH...
#define JTAG_IF_VARA_EQ          (28)  // Value     IF statement testing variable A
#define JTAG_IF_VARA_EQ_Q(x)     JTAG_PUSH_Q(x), (JTAG_IF_VARA_EQ)  // 5-bit value
#define JTAG_IF_VARA_EQ_8(x)     JTAG_PUSH_8(x), (JTAG_IF_VARA_EQ)  // 8-bit value
#define JTAG_IF_VARA_EQ_16(x)    JTAG_PUSH_16(x),(JTAG_IF_VARA_EQ)  // 16-bit value
#define JTAG_IF_VARA_EQ_32(x)    JTAG_PUSH_32(x),(JTAG_IF_VARA_EQ)  // 32-bit value

#define JTAG_IF_VARB_EQ          (29)  // Value     IF statement testing variable A
#define JTAG_IF_VARB_EQ_Q(x)     JTAG_PUSH_Q(x), (JTAG_IF_VARB_EQ)  // 5-bit value
#define JTAG_IF_VARB_EQ_8(x)     JTAG_PUSH_8(x), (JTAG_IF_VARB_EQ)  // 8-bit value
#define JTAG_IF_VARB_EQ_16(x)    JTAG_PUSH_16(x),(JTAG_IF_VARB_EQ)  // 16-bit value
#define JTAG_IF_VARB_EQ_32(x)    JTAG_PUSH_32(x),(JTAG_IF_VARB_EQ)  // 32-bit value

#define JTAG_IF_ITER_NEQ         (30)  // Value     IF statement testing iteration number
#define JTAG_IF_ITER_NEQ_Q(x)    JTAG_PUSH_Q(x), (JTAG_IF_ITER_NEQ)  // 5-bit value
#define JTAG_IF_ITER_NEQ_8(x)    JTAG_PUSH_8(x), (JTAG_IF_ITER_NEQ)  // 8-bit value
#define JTAG_IF_ITER_NEQ_16(x)   JTAG_PUSH_16(x),(JTAG_IF_ITER_NEQ)  // 16-bit value
#define JTAG_IF_ITER_NEQ_32(x)   JTAG_PUSH_32(x),(JTAG_IF_ITER_NEQ)  // 32-bit value

#define JTAG_IF_ITER_EQ          (31)  // Value     IF statement testing iteration number
#define JTAG_IF_ITER_EQ_Q(x)     JTAG_PUSH_Q(x), (JTAG_IF_ITER_EQ)  // 5-bit value
#define JTAG_IF_ITER_EQ_8(x)     JTAG_PUSH_8(x), (JTAG_IF_ITER_EQ)  // 8-bit value
#define JTAG_IF_ITER_EQ_16(x)    JTAG_PUSH_16(x),(JTAG_IF_ITER_EQ)  // 16-bit value
#define JTAG_IF_ITER_EQ_32(x)    JTAG_PUSH_32(x),(JTAG_IF_ITER_EQ)  // 32-bit value

//============================================================================================
// The following have no operands
#define JTAG_LOAD_VAR(x)         (32+(x))    // Loads Variable from Temp
#define JTAG_LOAD_VARA           JTAG_LOAD_VAR(0)
#define JTAG_LOAD_VARA_Q(x)      JTAG_PUSH_Q(x),  JTAG_LOAD_VARA
#define JTAG_LOAD_VARA_8(x)      JTAG_PUSH_8(x),  JTAG_LOAD_VARA
#define JTAG_LOAD_VARA_16(x)     JTAG_PUSH_16(x), JTAG_LOAD_VARA
#define JTAG_LOAD_VARA_32(x)     JTAG_PUSH_32(x), JTAG_LOAD_VARA
#define JTAG_LOAD_VARA_DP_8      JTAG_PUSH_DP_8,  JTAG_LOAD_VARA

#define JTAG_LOAD_VARB           JTAG_LOAD_VAR(1)
#define JTAG_LOAD_VARB_Q(x)      JTAG_PUSH_Q(x),  JTAG_LOAD_VARB
#define JTAG_LOAD_VARB_8(x)      JTAG_PUSH_8(x),  JTAG_LOAD_VARB
#define JTAG_LOAD_VARB_16(x)     JTAG_PUSH_16(x), JTAG_LOAD_VARB
#define JTAG_LOAD_VARB_32(x)     JTAG_PUSH_32(x), JTAG_LOAD_VARB
#define JTAG_LOAD_VARB_DP_8      JTAG_PUSH_DP_8,  JTAG_LOAD_VARB

#define JTAG_SAVEDP(x)           (32+(x))
#define JTAG_SAVE_DP_VARC        JTAG_SAVEDP(2)   // Copy dataPtr to VARC/D
#define JTAG_SAVE_DP_VARD        JTAG_SAVEDP(3)

#define JTAG_RESTOREDP(x)        (36+(x))
#define JTAG_RESTORE_DP_VARC     JTAG_RESTOREDP(2)   // Restore dataPtr from VARC/D
#define JTAG_RESTORE_DP_VARD     JTAG_RESTOREDP(3)

//============================================================================================
// The following use a value previously set by JTAG_PUSH...
                                       // 5/8/16/32 bit operand from JTAG_PUSH...
#define JTAG_IF_VARA_NEQ         (36)  // Value     IF statement testing variable A
#define JTAG_IF_VARA_NEQ_Q(x)    JTAG_PUSH_Q(x), (JTAG_IF_VARA_NEQ)  // 5-bit value
#define JTAG_IF_VARA_NEQ_8(x)    JTAG_PUSH_8(x), (JTAG_IF_VARA_NEQ)  // 8-bit value
#define JTAG_IF_VARA_NEQ_16(x)   JTAG_PUSH_16(x),(JTAG_IF_VARA_NEQ)  // 16-bit value
#define JTAG_IF_VARA_NEQ_32(x)   JTAG_PUSH_32(x),(JTAG_IF_VARA_NEQ)  // 32-bit value

#define JTAG_IF_VARB_NEQ         (37)  // Value     IF statement testing variable B
#define JTAG_IF_VARB_NEQ_Q(x)    JTAG_PUSH_Q(x), (JTAG_IF_VARB_NEQ)  // 5-bit value
#define JTAG_IF_VARB_NEQ_8(x)    JTAG_PUSH_8(x), (JTAG_IF_VARB_NEQ)  // 8-bit value
#define JTAG_IF_VARB_NEQ_16(x)   JTAG_PUSH_16(x),(JTAG_IF_VARB_NEQ)  // 16-bit value
#define JTAG_IF_VARB_NEQ_32(x)   JTAG_PUSH_32(x),(JTAG_IF_VARB_NEQ)  // 32-bit value

//============================================================================================
// The following uses a value previously set by JTAG_PUSH...
                                     // 5/8/16/32 bit operand from JTAG_PUSH...
#define JTAG_REPEAT              (40)  // Value     Repeat a block N times
//#define JTAG_REPEAT_8(x)       JTAG_PUSH_8(x), (JTAG_REPEAT)  // 8-bit value
#define JTAG_REPEAT_16(x)        JTAG_PUSH_16(x),(JTAG_REPEAT)  // 16-bit value
#define JTAG_REPEAT_32(x)        JTAG_PUSH_32(x),(JTAG_REPEAT)  // 32-bit value

//============================================================================================
// The following use an 8-bit operand as next byte in sequence
#define JTAG_REPEAT8             (41)
#define JTAG_REPEAT_8(x)         (JTAG_REPEAT8),(x)   // 8-bit value

//============================================================================================
// The following push an 8/16/32-bit operand as the next 1/2/4 bytes in sequence
#define JTAG_PUSH8               (42)
#define JTAG_PUSH_8(x)           (JTAG_PUSH8),(x)                                               // Push an 8-bit #
#define JTAG_PUSH16              (43)
#define JTAG_PUSH_16(x)          (JTAG_PUSH16),((U8)(x>>8)),((U8)x)                             // Push a 16-bit #
#define JTAG_PUSH32              (44)
#define JTAG_PUSH_32(x)          (JTAG_PUSH32),((U8)(x>>24)),((U8)(x>>16)),((U8)(x>>8)),((U8)x) // Push a 32-bit #

//============================================================================================
// The following have an 8/16/32-bit operands from DP
#define JTAG_PUSH_DP_8           (45)
#define JTAG_PUSH_DP_16          (46)
#define JTAG_PUSH_DP_32          (47)

//==============================================================================================
// The following have an 8-bit operand as the next byte, if zero then value is taken from dataPtr
                                       // Operand
#define JTAG_SAVE_SUB            (48)       // Save subroutine

#define JTAG_SKIP_DP             (49)
#define JTAG_SKIP_DP_Q(x)        JTAG_PUSH_Q(x),  JTAG_SKIP_DP
#define JTAG_SKIP_DP_8(x)        JTAG_PUSH_8(x),  JTAG_SKIP_DP

#define JTAG_SHIFT_OUT_DP_VARA   (50)  // Shift out VARA bits, data taken from dataPtr
#define JTAG_SET_BUSY            (51)  // Set BDM USB interface to send BUSY response

#define JTAG_SHIFT_OUT_VAR(x)                 (52+(x))     // #Bits   Shift out variable x to TDI
#define JTAG_SHIFT_OUT_VARA      JTAG_SHIFT_OUT_VAR(0)     // #Bits   Shift out variable A to TDI
#define JTAG_SHIFT_OUT_VARB      JTAG_SHIFT_OUT_VAR(1)     // #Bits   Shift out variable B to TDI
#define JTAG_SHIFT_OUT_VARC      JTAG_SHIFT_OUT_VAR(2)     // #Bits   Shift out variable C to TDI
#define JTAG_SHIFT_OUT_VARD      JTAG_SHIFT_OUT_VAR(3)     // #Bits   Shift out variable D to TDI

#define JTAG_SHIFT_IN_OUT_VAR(x)                 (56+(x))  // #Bits     Set variable x from TDO, with TDI
#define JTAG_SHIFT_IN_OUT_VARA   JTAG_SHIFT_IN_OUT_VAR(0)  // #Bits     Set variable A from TDO, with TDI
#define JTAG_SHIFT_IN_OUT_VARB   JTAG_SHIFT_IN_OUT_VAR(1)  // #Bits     Set variable B from TDO, with TDI
#define JTAG_SHIFT_IN_OUT_VARC   JTAG_SHIFT_IN_OUT_VAR(2)  // #Bits     Set variable C from TDO, with TDI
#define JTAG_SHIFT_IN_OUT_VARD   JTAG_SHIFT_IN_OUT_VAR(3)  // #Bits     Set variable D from TDO, with TDI

#define JTAG_SHIFT_OUT_DP        (60)  // #Bits     Shift out N bits, data taken from dataPtr
#define JTAG_SHIFT_IN_DP         (61)  // #Bits     Shift in N bits
#define JTAG_SHIFT_IN_OUT_DP     (62)  // #Bits     Shift out & in N bits, data taken from dataPtr

//============================================================================================
#define JTAG_RESERVED_2          (2<<5)

//============================================================================================
// The following quick commands take a fixed operand (N=1-31,0=>32) as part of the opcode
                                                                   // Operand
#define JTAG_SHIFT_IN_Q(N)       ((3<<5)|((N)&JTAG_NUM_BITS_MASK)) // #Bits    Shift in N bits (fill with TDI=0/1)
#define JTAG_SHIFT_OUT_Q(N)      ((4<<5)|((N)&JTAG_NUM_BITS_MASK)) // #Bits    Shift out N bits (data taken in-line)
#define JTAG_SHIFT_IN_OUT_Q(N)   ((5<<5)|((N)&JTAG_NUM_BITS_MASK)) // #Bits    Shift out & in N bits (data taken in-line)
#define JTAG_NUM_BITS_MASK       (0x1F)   // Mask for number of bits (N) within above opcodes

//============================================================================================
// The following quick commands take a count (N=2-31,0=>32, 1=>TL) as part of the opcode or from dataptr
#define JTAG_REPEAT_Q(N)         ((6<<5)|((N)&JTAG_NUM_BITS_MASK))  // Repeat a block N times
#define JTAG_REPEAT_DP           JTAG_REPEAT_Q(1)                   // A repeat count of 1 means use 8-bit outDataPtr

//============================================================================================
// The following quick command take a value (N=0-31) as part of the opcode
#define JTAG_PUSH_Q(N)           ((7<<5)|((N)&JTAG_NUM_BITS_MASK))  // Push a 5-bit value

//============================================================================================
// ARM Specific commands
#define JTAG_ARM_READAP    (64) // #addr (16-bit address A[15:8]=AP#, A[7:4]=Bank#, A[3:2]=Reg# Read value from AP register
#define JTAG_ARM_WRITEAP   (65) // Write input data value to AP register
#define JTAG_ARM_WRITEAP_I (66) // Write immediate value to AP register

//=====================================================================================================
// The following have an 16-bit operand(s) as the next few bytes
//
#define JTAG_SET_PADDING    (67) // #4x16-bits - sets HDR HIR TDR TIR

//=====================================================================================================
// The following have an 16-bit operand(s) as the next few bytes
//
#define JTAG_READ_MEM     (68) // # Read DSC memory block
#define JTAG_WRITE_MEM    (69) // # Execute previously set DSC instructions

//! Calculate number of bytes required to hold N bits
#define BITS_TO_BYTES(N) (((N)+7)>>3)

U8 processJTAGSequence(const U8 *data, U8 *dataIn);
U8 initJTAGSequence(void);       

#define true TRUE
#define false FALSE
#define USBDM_JTAG_Reset()                jtag_transition_reset();
#define USBDM_JTAG_SelectShift(x)         jtag_transition_shift((x))
#define USBDM_JTAG_Read(x, y, z)          jtag_read((y), (x), (z))
#define USBDM_JTAG_Write(x, y, z)         jtag_write((y), (x), (z) )
#define USBDM_JTAG_ReadWrite(w, x, y, z)  jtag_read_write((x), (w), (y), (z))

#endif /* JTAGSEQUENCE_HPP_ */
