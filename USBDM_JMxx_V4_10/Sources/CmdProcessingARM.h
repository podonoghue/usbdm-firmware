/*
 * CmdProcessingARM.h
 *
 *  Created on: 20/08/2012
 *      Author: PODonoghue
 */

#ifndef CMDPROCESSINGARM_H_
#define CMDPROCESSINGARM_H_

#include <stdint.h>

uint8_t f_CMD_ARM_CONNECT(void);
uint8_t f_CMD_ARM_TARGET_STEP(void); 
uint8_t f_CMD_ARM_TARGET_GO(void); 
uint8_t f_CMD_ARM_TARGET_HALT(void); 
uint8_t f_CMD_ARM_WRITE_MEM(void);
uint8_t f_CMD_ARM_READ_MEM(void);
uint8_t f_CMD_ARM_WRITE_REG(void);
uint8_t f_CMD_ARM_READ_ALL_CORE_REGS(void);
uint8_t f_CMD_ARM_READ_REG(void);
uint8_t f_CMD_ARM_WRITE_DREG(void);
uint8_t f_CMD_ARM_READ_DREG(void);
uint8_t f_CMD_ARM_WRITE_CREG(void);
uint8_t f_CMD_ARM_READ_CREG(void);

#endif /* CMDPROCESSINGARM_H_ */
