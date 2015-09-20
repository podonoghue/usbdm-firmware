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

extern uint8_t lastJtagIR_Value;

uint8_t arm_test(void);
uint8_t arm_readReg(uint8_t regNo, uint8_t *data);
uint8_t arm_writeReg(uint8_t regNo, const uint8_t *data);
uint8_t arm_readAPReg(const uint8_t *address, uint8_t *buff);
uint8_t arm_writeAPReg(const uint8_t *address, const uint8_t *buff);
uint8_t arm_abortAP(void);

uint8_t arm_CheckStickyPipelined(void);
uint8_t arm_CheckStickyUnpipelined(void);

#endif /* ARM_H_ */
