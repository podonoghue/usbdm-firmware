/*
 * ARM.h
 *
 *  Created on: 20/08/2012
 *      Author: PODonoghue
 */

#ifndef ARM_H_
#define ARM_H_

// Read registers
#define ARM_RD_DP_STATUS  (1)
#define ARM_RD_DP_SELECT  (2)
#define ARM_RD_DP_RDBUFF  (3)

// Write registers
#define ARM_WR_ABORT      (0)
#define ARM_WR_DP_CONTROL (1)
#define ARM_WR_DP_SELECT  (2)
#define ARM_WR_DP_RDBUFF  (3)

// Read AP register
#define ARM_RD_AP_REG0    (4)
#define ARM_RD_AP_REG1    (5)
#define ARM_RD_AP_REG2    (6)
#define ARM_RD_AP_REG3    (7)
//                        
// Write AP register      
#define ARM_WR_AP_REG0    (4)
#define ARM_WR_AP_REG1    (5)
#define ARM_WR_AP_REG2    (6)
#define ARM_WR_AP_REG3    (7)

#define ARM_RD_AHB_CSW ARM_RD_AP_REG0 // SWD command for reading AHB-CSW
#define ARM_RD_AHB_TAR ARM_RD_AP_REG1 // SWD command for reading AHB-TAR
#define ARM_RD_AHB_DRW ARM_RD_AP_REG3 // SWD command for reading AHB-DRW

#define ARM_WR_AHB_CSW ARM_WR_AP_REG0 // SWD command for writing AHB-CSW
#define ARM_WR_AHB_TAR ARM_WR_AP_REG1 // SWD command for writing AHB-TAR
#define ARM_WR_AHB_DRW ARM_WR_AP_REG3 // SWD command for writing AHB-DRW

extern U8 lastJtagIR_Value;

U8 arm_test(void);
U8 arm_readReg(U8 regNo, U8 *data);
U8 arm_writeReg(U8 regNo, const U8 *data);
U8 arm_readAPReg(const U8 *address, U8 *buff);
U8 arm_writeAPReg(const U8 *address, const U8 *buff);
U8 arm_abortAP(void);

U8 arm_CheckStickyPipelined(void);
U8 arm_CheckStickyUnpipelined(void);

#endif /* ARM_H_ */
